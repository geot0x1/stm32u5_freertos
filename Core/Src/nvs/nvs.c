#include "nvs.h"
#include "crc_gen.h"
#include <string.h>

/*===========================================================================
 *  Module state
 *===========================================================================*/

static nvs_context_t gNvs;

#define DRV_WRITE(addr, data, len) gNvs.driver.write((addr), (data), (len))
#define DRV_READ(addr, data, len) gNvs.driver.read((addr), (data), (len))
#define DRV_ERASE(addr) gNvs.driver.erase_sector((addr))
#define SECTOR_SIZE gNvs.driver.sector_size
#define SECTOR_COUNT gNvs.driver.sector_count

/*===========================================================================
 *  Addressing helpers
 *===========================================================================*/

static inline uint32_t sector_addr(uint8_t idx)
{
    return gNvs.driver.base_addr + (uint32_t)idx * SECTOR_SIZE;
}

/* Round up to 8-byte boundary (matches STM32C0xx programming granularity). */
static inline uint32_t align8(uint32_t v)
{
    return (v + 7U) & ~7U;
}

static inline uint32_t entry_total_size(uint8_t key_len, uint8_t data_len)
{
    return align8(NVS_ENTRY_HDR_SIZE + (uint32_t)key_len + (uint32_t)data_len);
}

/*===========================================================================
 *  Sector header I/O
 *
 *  Header layout (8 bytes, single atomic write):
 *    Bytes 0-3: magic (NVS_MAGIC_WORD)
 *    Bytes 4-7: seq   (monotonically increasing)
 *===========================================================================*/

static int read_sector_hdr(uint32_t base, uint32_t *magic, uint32_t *seq)
{
    uint8_t hdr[8];
    DRV_READ(base, hdr, 8);
    *magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16)
        | ((uint32_t)hdr[3] << 24);
    *seq = (uint32_t)hdr[4] | ((uint32_t)hdr[5] << 8) | ((uint32_t)hdr[6] << 16)
        | ((uint32_t)hdr[7] << 24);
    return (*magic == NVS_MAGIC_WORD && *seq != 0xFFFFFFFFU);
}

/* Single 8-byte write — target must be erased. */
static void write_sector_hdr(uint32_t base, uint32_t seq)
{
    uint8_t hdr[8];
    hdr[0] = (uint8_t)(NVS_MAGIC_WORD);
    hdr[1] = (uint8_t)(NVS_MAGIC_WORD >> 8);
    hdr[2] = (uint8_t)(NVS_MAGIC_WORD >> 16);
    hdr[3] = (uint8_t)(NVS_MAGIC_WORD >> 24);
    hdr[4] = (uint8_t)(seq);
    hdr[5] = (uint8_t)(seq >> 8);
    hdr[6] = (uint8_t)(seq >> 16);
    hdr[7] = (uint8_t)(seq >> 24);
    DRV_WRITE(base, hdr, 8);
}

/*===========================================================================
 *  Entry header I/O
 *===========================================================================*/

static uint8_t read_entry_hdr(uint32_t addr, uint8_t *key_len, uint8_t *data_len, uint32_t *crc)
{
    uint8_t hdr[NVS_ENTRY_HDR_SIZE];
    DRV_READ(addr, hdr, NVS_ENTRY_HDR_SIZE);
    *key_len = hdr[1];
    *data_len = hdr[2];
    *crc = (uint32_t)hdr[4] | ((uint32_t)hdr[5] << 8) | ((uint32_t)hdr[6] << 16)
        | ((uint32_t)hdr[7] << 24);
    return hdr[0]; /* state */
}

static uint32_t compute_entry_crc(
    uint8_t key_len, uint8_t data_len, const uint8_t *key, const uint8_t *data)
{
    uint8_t buf[2U + NVS_MAX_KEY_LEN + NVS_MAX_DATA_LEN];
    uint32_t n = 0;
    buf[n++] = key_len;
    buf[n++] = data_len;
    memcpy(&buf[n], key, key_len);
    n += key_len;
    memcpy(&buf[n], data, data_len);
    n += data_len;
    return crc32_gen(buf, n, 0xFFFFFFFF);
}

/*===========================================================================
 *  Sector sorting (descending seq)
 *===========================================================================*/

static void get_sectors_by_seq_desc(uint8_t *out_idx, uint8_t *out_count)
{
    uint32_t seqs[16];
    uint8_t valid[16];
    uint8_t n = 0;

    for (uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        uint32_t magic, seq;
        if (read_sector_hdr(sector_addr(i), &magic, &seq))
        {
            valid[n] = i;
            seqs[n] = seq;
            n++;
        }
    }

    for (uint8_t i = 1; i < n; i++)
    {
        uint32_t s = seqs[i];
        uint8_t v = valid[i];
        int j = (int)i - 1;
        while (j >= 0 && seqs[j] < s)
        {
            seqs[j + 1] = seqs[j];
            valid[j + 1] = valid[j];
            j--;
        }
        seqs[j + 1] = s;
        valid[j + 1] = v;
    }

    memcpy(out_idx, valid, n);
    *out_count = n;
}

/*===========================================================================
 *  Scan write offset
 *===========================================================================*/

static uint32_t scan_write_offset(uint32_t base)
{
    uint32_t off = NVS_SECTOR_HDR_SIZE;

    while (off < SECTOR_SIZE)
    {
        uint8_t kl, dl;
        uint32_t crc;
        uint8_t st = read_entry_hdr(base + off, &kl, &dl, &crc);

        if (st != NVS_ENTRY_VALID)
        {
            break; /* 0xFF = erased = end of log */
        }

        uint32_t esz = entry_total_size(kl, dl);
        if (esz == 0 || off + esz > SECTOR_SIZE)
        {
            break;
        }

        off += esz;
    }

    return off;
}

/*===========================================================================
 *  GC helper — is this entry superseded?
 *
 *  An entry at (src_sector_idx, src_offset) is superseded if:
 *  a) A later entry for the same key exists in a higher-seq sector, OR
 *  b) A later entry for the same key exists later in the same sector.
 *===========================================================================*/

static int is_superseded(
    const uint8_t *key, uint8_t key_len, uint8_t src_sector_idx, uint32_t src_offset)
{
    uint32_t src_base = sector_addr(src_sector_idx);
    uint32_t src_magic, src_seq;
    read_sector_hdr(src_base, &src_magic, &src_seq);

    for (uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        uint32_t base = sector_addr(i);
        uint32_t magic, seq;
        if (!read_sector_hdr(base, &magic, &seq))
        {
            continue;
        }

        int same_sector = (i == src_sector_idx);

        /* Cross-sector: only check higher-seq sectors. */
        if (!same_sector && seq <= src_seq)
        {
            continue;
        }

        uint32_t off = NVS_SECTOR_HDR_SIZE;

        while (off < SECTOR_SIZE)
        {
            uint8_t kl, dl;
            uint32_t crc;
            uint8_t st = read_entry_hdr(base + off, &kl, &dl, &crc);

            if (st != NVS_ENTRY_VALID)
            {
                break;
            }

            /* Within the same sector: only check entries AFTER src_offset. */
            if (same_sector && off <= src_offset)
            {
                off += entry_total_size(kl, dl);
                continue;
            }

            if (kl == key_len)
            {
                uint8_t flash_key[NVS_MAX_KEY_LEN];
                DRV_READ(base + off + NVS_ENTRY_HDR_SIZE, flash_key, kl);
                if (memcmp(flash_key, key, kl) == 0)
                {
                    return 1;
                }
            }

            off += entry_total_size(kl, dl);
        }
    }

    return 0;
}

/*===========================================================================
 *  Activate next empty sector
 *===========================================================================*/

static nvs_err_t activate_next_sector(void)
{
    for (uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        uint32_t base = sector_addr(i);
        uint32_t magic, seq;

        if (!read_sector_hdr(base, &magic, &seq) && magic == 0xFFFFFFFFU)
        {
            gNvs.seq_counter++;
            write_sector_hdr(base, gNvs.seq_counter);
            gNvs.active_sector_addr = base;
            gNvs.write_offset = NVS_SECTOR_HDR_SIZE;
            return NVS_OK;
        }
    }

    return NVS_ERR_NO_SPACE;
}

/*===========================================================================
 *  Garbage collection
 *===========================================================================*/

static nvs_err_t nvs_gc(void)
{
    /* Find the valid sector with the lowest seq (oldest), excluding active. */
    uint32_t lowest_seq = 0xFFFFFFFFU;
    int target_idx = -1;

    for (uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        uint32_t base = sector_addr(i);
        uint32_t magic, seq;

        if (!read_sector_hdr(base, &magic, &seq))
        {
            continue;
        }
        if (base == gNvs.active_sector_addr)
        {
            continue;
        }

        if (seq < lowest_seq)
        {
            lowest_seq = seq;
            target_idx = (int)i;
        }
    }

    if (target_idx < 0)
    {
        return NVS_ERR_NO_SPACE;
    }

    uint32_t target_base = sector_addr((uint8_t)target_idx);

    /* Copy each live (non-superseded) entry to the active sector. */
    uint32_t off = NVS_SECTOR_HDR_SIZE;

    while (off < SECTOR_SIZE)
    {
        uint8_t kl, dl;
        uint32_t crc;
        uint8_t st = read_entry_hdr(target_base + off, &kl, &dl, &crc);

        if (st != NVS_ENTRY_VALID)
        {
            break;
        }

        uint32_t esz = entry_total_size(kl, dl);

        uint8_t flash_key[NVS_MAX_KEY_LEN];
        DRV_READ(target_base + off + NVS_ENTRY_HDR_SIZE, flash_key, kl);

        if (!is_superseded(flash_key, kl, (uint8_t)target_idx, off))
        {
            if (gNvs.write_offset + esz > SECTOR_SIZE)
            {
                return NVS_ERR_NO_SPACE;
            }

            uint8_t data_buf[NVS_MAX_DATA_LEN];
            uint8_t entry[NVS_ENTRY_HDR_SIZE + NVS_MAX_KEY_LEN + NVS_MAX_DATA_LEN + 8U];
            DRV_READ(target_base + off + NVS_ENTRY_HDR_SIZE + kl, data_buf, dl);

            memset(entry, 0xFF, esz);
            entry[0] = NVS_ENTRY_VALID;
            entry[1] = kl;
            entry[2] = dl;
            entry[3] = 0xFF;

            uint32_t c = compute_entry_crc(kl, dl, flash_key, data_buf);
            entry[4] = (uint8_t)(c);
            entry[5] = (uint8_t)(c >> 8);
            entry[6] = (uint8_t)(c >> 16);
            entry[7] = (uint8_t)(c >> 24);

            memcpy(&entry[NVS_ENTRY_HDR_SIZE], flash_key, kl);
            memcpy(&entry[NVS_ENTRY_HDR_SIZE + kl], data_buf, dl);

            DRV_WRITE(gNvs.active_sector_addr + gNvs.write_offset, entry, (uint16_t)esz);
            gNvs.write_offset += esz;
        }

        off += esz;
    }

    DRV_ERASE(target_base);
    return NVS_OK;
}

/*===========================================================================
 *  Public API — nvs_mount
 *===========================================================================*/

nvs_err_t nvs_mount(const nvs_flash_driver_t *driver)
{
    if (driver == NULL || driver->write == NULL || driver->read == NULL
        || driver->erase_sector == NULL || driver->sector_size == 0 || driver->sector_count == 0)
    {
        return NVS_ERR_INVALID_ARG;
    }

    gNvs.driver = *driver;
    gNvs.seq_counter = 0;

    uint32_t best_seq = 0;
    int best_idx = -1;

    for (uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        uint32_t magic, seq;
        if (read_sector_hdr(sector_addr(i), &magic, &seq))
        {
            if (seq > gNvs.seq_counter)
            {
                gNvs.seq_counter = seq;
            }
            if (seq >= best_seq)
            {
                best_seq = seq;
                best_idx = (int)i;
            }
        }
    }

    if (best_idx < 0)
    {
        /* First-time format: write sector 0 header. */
        gNvs.seq_counter = 1;
        write_sector_hdr(sector_addr(0), 1);
        gNvs.active_sector_addr = sector_addr(0);
        gNvs.write_offset = NVS_SECTOR_HDR_SIZE;
        return NVS_OK;
    }

    gNvs.active_sector_addr = sector_addr((uint8_t)best_idx);
    gNvs.write_offset = scan_write_offset(gNvs.active_sector_addr);
    return NVS_OK;
}

/*===========================================================================
 *  Public API — nvs_write
 *===========================================================================*/

nvs_err_t nvs_write(const char *key, const void *data, uint8_t len)
{
    if (key == NULL || data == NULL)
    {
        return NVS_ERR_INVALID_ARG;
    }

    uint8_t key_len = (uint8_t)strlen(key);
    if (key_len == 0 || key_len > NVS_MAX_KEY_LEN)
    {
        return NVS_ERR_INVALID_ARG;
    }
    if (len > NVS_MAX_DATA_LEN)
    {
        return NVS_ERR_INVALID_ARG;
    }

    uint32_t esz = entry_total_size(key_len, len);

    if (gNvs.write_offset + esz > SECTOR_SIZE)
    {
        nvs_err_t rc = activate_next_sector();
        if (rc != NVS_OK)
        {
            rc = nvs_gc();
            if (rc != NVS_OK)
            {
                return NVS_ERR_NO_SPACE;
            }

            rc = activate_next_sector();
            if (rc != NVS_OK)
            {
                return NVS_ERR_NO_SPACE;
            }
        }
    }

    /* Build and write the entire entry atomically to an erased location. */
    uint8_t entry[NVS_ENTRY_HDR_SIZE + NVS_MAX_KEY_LEN + NVS_MAX_DATA_LEN + 8U];
    memset(entry, 0xFF, esz);

    entry[0] = NVS_ENTRY_VALID;
    entry[1] = key_len;
    entry[2] = len;
    entry[3] = 0xFF;

    uint32_t crc = compute_entry_crc(key_len, len, (const uint8_t *)key, (const uint8_t *)data);
    entry[4] = (uint8_t)(crc);
    entry[5] = (uint8_t)(crc >> 8);
    entry[6] = (uint8_t)(crc >> 16);
    entry[7] = (uint8_t)(crc >> 24);

    memcpy(&entry[NVS_ENTRY_HDR_SIZE], key, key_len);
    memcpy(&entry[NVS_ENTRY_HDR_SIZE + key_len], data, len);

    DRV_WRITE(gNvs.active_sector_addr + gNvs.write_offset, entry, (uint16_t)esz);
    gNvs.write_offset += esz;

    return NVS_OK;
}

/*===========================================================================
 *  Public API — nvs_read
 *===========================================================================*/

nvs_err_t nvs_read(const char *key, void *buf, uint8_t buf_size, uint8_t *out_len)
{
    if (key == NULL || buf == NULL || out_len == NULL)
    {
        return NVS_ERR_INVALID_ARG;
    }

    uint8_t key_len = (uint8_t)strlen(key);
    if (key_len == 0 || key_len > NVS_MAX_KEY_LEN)
    {
        return NVS_ERR_INVALID_ARG;
    }

    uint8_t indices[16];
    uint8_t count;
    get_sectors_by_seq_desc(indices, &count);

    for (uint8_t s = 0; s < count; s++)
    {
        uint32_t base = sector_addr(indices[s]);
        uint32_t off = NVS_SECTOR_HDR_SIZE;

        uint32_t match_off = 0;
        uint8_t match_dl = 0;
        int found = 0;

        while (off < SECTOR_SIZE)
        {
            uint8_t kl, dl;
            uint32_t crc;
            uint8_t st = read_entry_hdr(base + off, &kl, &dl, &crc);

            if (st != NVS_ENTRY_VALID)
            {
                break; /* 0xFF = erased = end of log */
            }

            if (kl == key_len)
            {
                uint8_t flash_key[NVS_MAX_KEY_LEN];
                DRV_READ(base + off + NVS_ENTRY_HDR_SIZE, flash_key, kl);
                if (memcmp(flash_key, key, kl) == 0)
                {
                    match_off = off;
                    match_dl = dl;
                    found = 1;
                }
            }

            off += entry_total_size(kl, dl);
        }

        if (found)
        {
            uint8_t kl2, dl2;
            uint32_t stored_crc;
            read_entry_hdr(base + match_off, &kl2, &dl2, &stored_crc);

            uint8_t key_buf[NVS_MAX_KEY_LEN];
            uint8_t data_buf[NVS_MAX_DATA_LEN];
            DRV_READ(base + match_off + NVS_ENTRY_HDR_SIZE, key_buf, kl2);
            DRV_READ(base + match_off + NVS_ENTRY_HDR_SIZE + kl2, data_buf, dl2);

            uint32_t calc_crc = compute_entry_crc(kl2, dl2, key_buf, data_buf);
            if (calc_crc != stored_crc)
            {
                return NVS_ERR_CRC;
            }
            if (match_dl > buf_size)
            {
                return NVS_ERR_INVALID_ARG;
            }

            memcpy(buf, data_buf, match_dl);
            *out_len = match_dl;
            return NVS_OK;
        }
    }

    return NVS_ERR_NOT_FOUND;
}

/*===========================================================================
 *  Public API — nvs_delete
 *  Writes a zero-length tombstone entry. nvs_read returns the tombstone
 *  (dl=0) which callers can treat as absent.
 *===========================================================================*/

nvs_err_t nvs_delete(const char *key)
{
    if (key == NULL)
    {
        return NVS_ERR_INVALID_ARG;
    }

    uint8_t dummy = 0;
    return nvs_write(key, &dummy, 0);
}