/**
 * @file nvs.c
 * @brief Non-Volatile Storage (NVS) Implementation
 */

#include "nvs.h"
#include "assert_macro.h"
#include "crc_gen.h"
#include "serial_flash_mem.h"
#include "trace_logger.h"
#include <string.h>

#ifdef CONFIG_NVS_MODULE_DEBUG
#define NVS_LOG(x, ...) TRACE_LOG_PRINT(x, ##__VA_ARGS__)
#else
#define NVS_LOG(x, ...) (void)0
#endif

/**
 * Closing block status:
 * id = FFFF
 * len = 0
 * offset = same as block write address
 * part = FF
 *
 * Zero (cleared) block status:
 * id = 0
 * len != 0
 * part FF
 */

// Sector and data constraints
#define NVS_SECTOR_SIZE  (65536)  // 64KB sector size
#define MAXIMUM_DATA_LEN (2048)   // Maximum data payload per entry
// Dynamic calculation: MAXIMUM_VALID_ID = (sector_size / sizeof(NvsAte))
// For 65536 byte sector with 8 byte ATE: 65536/8 = 8192
// Using 7281 to leave safety margin for closing blocks and sector boundaries
#define MAXIMUM_VALID_ID ((NVS_SECTOR_SIZE / sizeof(NvsAte)) - 911)

// Address masks for sector and offset extraction
#define ADDR_SECT_MASK        (0xFFFF0000)
#define ADDR_SECTOR_BASE_MASK (0xFFFF0000)
#define BLOCK_OFFSET_MASK     (0x0000FFFF)
#define OFFSET_ADDR_MASK      (0x0000FFFF)
#define ADDR_PAGE_MASK        (0xFFFFFF00)

#define BLOCK_INIT_VALUE {.id = 0xFFFF, .offset = 0xFFFF, .len = 0xFFFF, .part = 0xFF, .crc8 = 0xFF}

#define NVS_ATE_SIZE (sizeof(NvsAte))

typedef enum
{
    META_BLOCK_NOT_VALID,
    META_BLOCK_IS_VALID,
    META_BLOCK_ERASED,
    META_BLOCK_ZERO,
    META_BLOCK_CLOSING,
} MetaBlockStatus;

typedef struct nvs_ate NvsAte;

struct nvs_ate
{
    uint16_t id;     /* data id */
    uint16_t offset; /* data offset within sector */
    uint16_t len;    /* data len within sector */
    uint8_t  part;   /* part of a multipart data - future extension */
    uint8_t  crc8;   /* crc8 check of the entry */
} __attribute__((packed));

BUILD_ASSERT(offsetof(struct nvs_ate, crc8) == sizeof(struct nvs_ate) - sizeof(uint8_t),
             "crc8 must be the last member");

static uint32_t        get_sector_address(Nvs *nvs, int sec_num);
static bool            is_meta_block_valid(const NvsAte *meta_ptr);
static bool            is_closing_block(const NvsAte *block, uint16_t offset);
static void            print_meta(NvsAte *meta);
static void            erase_sector(Nvs *nvs, uint16_t sec_num);
static bool            is_block_erased(NvsAte *block);
static bool            is_block_zero(NvsAte *block);
static MetaBlockStatus get_block_status(NvsAte *block, uint16_t offset);
static void            semaphore_lock(Nvs *nvs);
static void            semaphore_unlock(Nvs *nvs);
static int             semaphore_trylock(Nvs *nvs, uint32_t timeout);
static int             read_meta_block(uint32_t addr, NvsAte *block);
static void            write_meta_block(Nvs *nvs, uint16_t data_len, uint32_t data_offset);
static void            write_data(Nvs *nvs, void *data, uint16_t data_len);
static void            write_closing(Nvs *nvs);
static inline uint32_t sector_number_from_address(uint32_t addr);
static inline uint32_t next_sector(const Nvs *nvs, uint32_t sec_num);
static int find_write_addr(Nvs *nvs);
static int find_read_addr(Nvs *nvs);

static void nvs_erase_prv(Nvs *nvs)
{
    NVS_LOG("\r\n[NVS] Erasing entire FIFO area\r\n");
    for (int sec = nvs->first_sector; sec <= nvs->last_sector; sec++)
    {
        uint32_t sec_addr = sec * nvs->sector_size;
        serial_flash_mem_erase_64k_block(sec_addr);
        NVS_LOG("[NVS] Erased sector %d at addr 0x%lX\r\n", sec, sec_addr);
    }
    NVS_LOG("[NVS] Erase complete\r\n\r\n");
}

void nvs_erase(Nvs *nvs)
{
    semaphore_lock(nvs);
    nvs_erase_prv(nvs);
    semaphore_unlock(nvs);
}

int nvs_init(Nvs *nvs)
{
    ASSERT(nvs->page_size == 256);
    ASSERT((nvs->last_sector - nvs->first_sector) + 1 > 2);
    ASSERT(nvs->sector_size == NVS_SECTOR_SIZE);

    if (nvs->semaphore == NULL)
    {
        nvs->semaphore = xSemaphoreCreateRecursiveMutex();
        xSemaphoreGiveRecursive(nvs->semaphore);
    }
    ASSERT(nvs->semaphore != NULL);
	serial_flash_mem_init();
    return NVS_OK;
}

void nvs_mount(Nvs *nvs)
{
    semaphore_lock(nvs);

    NVS_LOG("\r\n[NVS] ========== NVS MOUNT START ==========\r\n");
    nvs->ready           = false;
    nvs->ate_read_addr   = 0;
    nvs->start_read_addr = 0;
    nvs->ate_write_addr  = 0;
    nvs->data_write_addr = 0;

    NVS_LOG("[NVS] Finding write head...\r\n");
    if (find_write_addr(nvs) == 0)
    {
        NVS_LOG("[NVS] Head found at: %lu\r\n", nvs->ate_write_addr);
        NVS_LOG("[NVS] Finding read tail...\r\n");
        find_read_addr(nvs);
    }
    else
    {
        nvs_erase(nvs);
        nvs->ate_write_addr  = (nvs->first_sector * nvs->sector_size) + nvs->sector_size - sizeof(NvsAte);
        nvs->data_write_addr = (nvs->first_sector * nvs->sector_size);
    }

    if (nvs->ate_read_addr == 0)
    {
        nvs->ate_read_addr   = nvs->ate_write_addr;
        nvs->start_read_addr = nvs->ate_write_addr;
    }

    NVS_LOG("\r\n[NVS] --- MOUNT COMPLETE ---\r\n");
    NVS_LOG("[NVS] Head is: %lu (sec %lu)\r\n", nvs->ate_write_addr, (nvs->ate_write_addr & ADDR_SECT_MASK) >> 16);
    NVS_LOG("[NVS] Tail is: %lu (sec %lu)\r\n", nvs->ate_read_addr, (nvs->ate_read_addr & ADDR_SECT_MASK) >> 16);
    NVS_LOG("[NVS] Tail start is: %lu\r\n", nvs->start_read_addr);
    NVS_LOG("[NVS] Data write addr: %lu\r\n", nvs->data_write_addr);
    NVS_LOG("[NVS] ========== NVS MOUNT END ==========\r\n\r\n");

    nvs->ready = true;

    semaphore_unlock(nvs);
    return;
}

static void semaphore_lock(Nvs *nvs)
{
    //! Block forever until it locks.
    while (semaphore_trylock(nvs, portMAX_DELAY) != pdTRUE)
        ;
}

static void semaphore_unlock(Nvs *nvs)
{
    ASSERT(nvs->semaphore != NULL);
    BaseType_t ret = xSemaphoreGiveRecursive(nvs->semaphore);
    ASSERT(ret == pdTRUE);
}

static int semaphore_trylock(Nvs *nvs, uint32_t timeout)
{
    ASSERT(nvs->semaphore != NULL);
    if (nvs->semaphore == NULL)
    {
        return pdFALSE;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        return xSemaphoreTakeRecursive(nvs->semaphore, timeout);
    }
    else
    {
        return xSemaphoreTakeRecursive(nvs->semaphore, 0);
    }
}

static void delete_next_sector(const Nvs *nvs)
{
    uint32_t sec           = sector_number_from_address(nvs->ate_write_addr);
    sec                    = next_sector(nvs, sec);
    uint32_t sec_base_addr = (sec * nvs->sector_size);
    serial_flash_mem_erase_64k_block(sec_base_addr);
}

static int nvs_write_prv(Nvs *nvs, void *data, size_t data_len)
{
    // Input validation
    if (data_len > MAXIMUM_DATA_LEN)
    {
        NVS_LOG("[NVS ERROR] Data length %zu exceeds MAXIMUM_DATA_LEN (%d)\r\n", data_len, MAXIMUM_DATA_LEN);
        return NVS_ERR;
    }

    //! We need at least one meta block, one closing block and we add one more to be safe.
    const int SPACE_NEEDED = data_len + (3 * sizeof(NvsAte));

    int available_size = 0;
    if (nvs->ate_write_addr > nvs->data_write_addr)
    {
        available_size = nvs->ate_write_addr - nvs->data_write_addr;
    }

    if (SPACE_NEEDED >= available_size)
    {
        //! Sector full - implement wrap logic with "bulk drop" of oldest sector
        //! 1. Erase the next sector (which will become our new write target)
        //! 2. Write closing block to current sector
        //! 3. Move write head to the newly erased sector
        //! 4. If read tail is in the same sector, advance it to preserve FIFO integrity
        NVS_LOG("\r\n[NVS] *** SECTOR FULL - WRAPPING ***\r\n");
        NVS_LOG("[NVS] Current sec: %lu, needed: %d, available: %d\r\n", 
                sector_number_from_address(nvs->ate_write_addr), SPACE_NEEDED, available_size);
        
        delete_next_sector(nvs);  // Erase next sector before switching
        write_closing(nvs);       // Close current sector

        uint32_t sec = sector_number_from_address(nvs->ate_write_addr);
        sec          = next_sector(nvs, sec);

        uint32_t sec_base_addr = (sec * nvs->sector_size);
        nvs->ate_write_addr    = sec_base_addr + nvs->sector_size - sizeof(NvsAte);
        nvs->data_write_addr   = nvs->ate_write_addr & ADDR_SECT_MASK;

        // Critical: Check if write head caught up with read tail (buffer overflow)
        if ((nvs->ate_write_addr & ADDR_SECT_MASK) == (nvs->ate_read_addr & ADDR_SECT_MASK))
        {
            //! Buffer is full - drop oldest 64KB sector by advancing read tail
            NVS_LOG("[NVS WARN] Buffer full - dropping oldest sector (bulk drop)\r\n");
            uint32_t read_sec = sector_number_from_address(nvs->ate_read_addr);
            read_sec          = next_sector(nvs, read_sec);

            nvs->ate_read_addr   = (read_sec * nvs->sector_size) + nvs->sector_size - sizeof(NvsAte);
            nvs->start_read_addr = nvs->ate_read_addr;
            NVS_LOG("[NVS] Read tail advanced to sec: %lu (addr: %lu)\r\n", read_sec, nvs->ate_read_addr);
        }
        NVS_LOG("[NVS] Write head moved to sec: %lu (addr: %lu)\r\n", 
                sector_number_from_address(nvs->ate_write_addr), nvs->ate_write_addr);
    }
    // CRITICAL: Capture data offset BEFORE write_data() updates data_write_addr
    uint32_t data_offset = nvs->data_write_addr;
    write_data(nvs, data, data_len);
    write_meta_block(nvs, data_len, data_offset);
    NVS_LOG("[NVS] Write OK: len=%d, addr=%lu\r\n", data_len, nvs->ate_write_addr + sizeof(NvsAte));
    return NVS_OK;
}

int nvs_write(Nvs *nvs, void *data, size_t data_len)
{
    if (nvs->ready == false)
    {
        return NVS_NOT_READY;
    }
    int ret = -1;
    semaphore_lock(nvs);
    ret = nvs_write_prv(nvs, data, data_len);
    semaphore_unlock(nvs);
    return ret;
}

static int nvs_read_prv(Nvs *nvs, void *data, size_t size)
{
    MetaBlockStatus status;
    uint32_t        consecutive_not_valid_count = 0;
    NVS_LOG("[NVS] Read attempt from addr: %lu\r\n", nvs->ate_read_addr);
    do
    {
        uint32_t       sec           = sector_number_from_address(nvs->ate_read_addr);
        const uint32_t SEC_BASE_ADDR = sec * nvs->sector_size;
        NvsAte         block         = BLOCK_INIT_VALUE;

        read_meta_block(nvs->ate_read_addr, &block);
        status = get_block_status(&block, nvs->ate_read_addr & OFFSET_ADDR_MASK);
        if (status != META_BLOCK_NOT_VALID)
        {
            consecutive_not_valid_count = 0;
        }
        
        if (status == META_BLOCK_ERASED)
        {
            if ((nvs->ate_read_addr == nvs->ate_write_addr))
            {
                NVS_LOG("[NVS] NVS EMPTY (read == write)\r\n");
                return NVS_EMPTY;
            }
            else
            {
                NVS_LOG("[NVS ERROR] ERASED TAIL - resetting read pointer\r\n");
                find_read_addr(nvs);
            }
        }

        if (status == META_BLOCK_IS_VALID)
        {
            uint32_t sector_base    = nvs->ate_read_addr & ADDR_SECT_MASK;
            uint32_t data_read_addr = sector_base + block.offset;
            if (block.len > size)
            {
                return NVS_ERR_READ_NO_SPACE;
            }
            serial_flash_mem_read(data_read_addr, data, block.len);

            NVS_LOG("VALID ate_read_addr: %lu\r\n", nvs->ate_read_addr);
            NVS_LOG("VALID start_read_addr: %lu\r\n", nvs->start_read_addr);
            nvs->ate_read_addr -= sizeof(NvsAte);
            return NVS_OK;
        }

        if (status == META_BLOCK_NOT_VALID)
        {
            // Clean-on-Read policy: Explicitly mark invalid blocks as deleted in flash
            // to prevent them from being read again in future operations.
            consecutive_not_valid_count++;
            NVS_LOG("[NVS] NOTVALID ate_read_addr: %lu - marking as deleted\r\n", nvs->ate_read_addr);
            
            // Mark the block as deleted by setting id = 0 (similar to nvs_delete_prv)
            block.id = 0;
            serial_flash_mem_write(nvs->ate_read_addr, &block, sizeof(NvsAte));
            
            nvs->ate_read_addr -= sizeof(NvsAte);
        }

        if (status == META_BLOCK_ZERO)
        {
            nvs->ate_read_addr -= sizeof(NvsAte);
        }
        if (status == META_BLOCK_CLOSING)
        {
            NVS_LOG("[NVS] CLOSING block found - jumping to next sector\r\n");
            sec                = next_sector(nvs, sec);
            nvs->ate_read_addr = (sec * nvs->sector_size) + nvs->sector_size - sizeof(NvsAte);
            NVS_LOG("[NVS] New read addr: %lu (sec %lu)\r\n", nvs->ate_read_addr, sec);
        }

        //! This is in case no closing block found and we are very below to sector
        // Fixed: Use else-if or better, check if we ALREADY jumped in this loop iteration
        // to avoid double jumps when wrapping from last_sector to first_sector.
        else if (nvs->ate_read_addr < SEC_BASE_ADDR + sizeof(NvsAte))
        {
            NVS_LOG("[NVS] Hit sector boundary without closing - jumping to next\r\n");
            sec                = next_sector(nvs, sec);
            nvs->ate_read_addr = (sec * nvs->sector_size) + nvs->sector_size - sizeof(NvsAte);
            NVS_LOG("[NVS] New read addr: %lu (sec %lu)\r\n", nvs->ate_read_addr, sec);
        }

        if (consecutive_not_valid_count > 8)
        {
            NVS_LOG("[NVS ERROR] Too many invalid blocks (%lu) - resetting tail !!!\r\n", consecutive_not_valid_count);
            find_read_addr(nvs);
            break;
        }
    } while ((status == META_BLOCK_ZERO) || (status == META_BLOCK_CLOSING) || (status == META_BLOCK_NOT_VALID));

    return NVS_ERR;
}

int nvs_read(Nvs *nvs, void *data, size_t size)
{
    if (nvs->ready == false)
    {
        return NVS_NOT_READY;
    }
    int ret = -1;
    semaphore_lock(nvs);
    ret = nvs_read_prv(nvs, data, size);
    semaphore_unlock(nvs);
    return ret;
}

static int nvs_delete_prv(Nvs *nvs)
{
    if (nvs->start_read_addr == nvs->ate_read_addr)
    {
        NVS_LOG("SAME AS ATE");
        return NVS_OK;
    }

    uint16_t guard = 16;
    while ((--guard > 0) && (nvs->start_read_addr != nvs->ate_read_addr))
    {
        NvsAte block = BLOCK_INIT_VALUE;
        read_meta_block(nvs->start_read_addr, &block);
        MetaBlockStatus status = get_block_status(&block, nvs->start_read_addr & OFFSET_ADDR_MASK);
        if ((status == META_BLOCK_IS_VALID) || (status == META_BLOCK_NOT_VALID))
        {
            uint32_t sector_base = nvs->start_read_addr & ADDR_SECT_MASK;
            block.id             = 0;
            serial_flash_mem_write(nvs->start_read_addr, &block, sizeof(NvsAte));
            NVS_LOG("Updated delete_addr: %lu\r\n", nvs->start_read_addr);
            nvs->start_read_addr -= sizeof(NvsAte);
        }
        else if (status == META_BLOCK_CLOSING)
        {
            uint32_t sec = sector_number_from_address(nvs->start_read_addr);
            sec          = next_sector(nvs, sec);
            NVS_LOG("Updated delete_addr: %lu\r\n", nvs->start_read_addr);
            nvs->start_read_addr = (sec * nvs->sector_size) + nvs->sector_size - sizeof(NvsAte);
        }
        else if (status == META_BLOCK_ZERO)
        {
            nvs->start_read_addr -= sizeof(NvsAte);
        }
        else if (status == META_BLOCK_ERASED)
        {
            NVS_LOG("in delete about to brake because of erased \r\n");
            NVS_LOG("Updated delete_addr: %lu\r\n", nvs->start_read_addr);
            break;
        }
    }
    return NVS_OK;
}

int nvs_delete(Nvs *nvs)
{
    if (nvs->ready == false)
    {
        return NVS_NOT_READY;
    }
    int ret = -1;
    semaphore_lock(nvs);
    ret = nvs_delete_prv(nvs);
    semaphore_unlock(nvs);
    return ret;
}

//------------------------------------------------------------------------------------------------------------

static void write_meta_block(Nvs *nvs, uint16_t data_len, uint32_t data_offset)
{
    NvsAte block = BLOCK_INIT_VALUE;

    const uint32_t SEC_BASE_ADDR = nvs->ate_write_addr & ADDR_SECT_MASK;
    uint16_t       id            = ((SEC_BASE_ADDR + nvs->sector_size) - nvs->ate_write_addr) / sizeof(NvsAte);

    ASSERT(id > 0);
    ASSERT(id < MAXIMUM_VALID_ID);

    block.id     = id;
    block.offset = data_offset & OFFSET_ADDR_MASK;  // Use passed offset, not nvs->data_write_addr!
    block.len    = data_len;
    block.part   = 0xFF;
    block.crc8   = crc8((uint8_t *)&block, sizeof(NvsAte) - 1);

    serial_flash_mem_write(nvs->ate_write_addr, &block, sizeof(NvsAte));

    nvs->ate_write_addr -= sizeof(NvsAte);
}

static void write_closing(Nvs *nvs)
{
    NvsAte block = BLOCK_INIT_VALUE;

    block.id     = 0xFFFF;
    block.offset = nvs->ate_write_addr & OFFSET_ADDR_MASK;
    block.len    = 0;
    block.part   = 0xFF;
    block.crc8   = crc8((uint8_t *)&block, sizeof(NvsAte) - 1);
    serial_flash_mem_write(nvs->ate_write_addr, &block, sizeof(NvsAte));
}

static void write_data(Nvs *nvs, void *data, uint16_t data_len)
{
    uint32_t addr_to_write = nvs->data_write_addr;
    int32_t  remaining     = data_len;
    uint8_t *data_ptr      = data;

    do
    {
        const uint32_t PAGE_SIZE_LEFT = ((addr_to_write & ADDR_PAGE_MASK) + nvs->page_size) - addr_to_write;
        uint32_t       write_len      = remaining;
        if (write_len > PAGE_SIZE_LEFT)
        {
            write_len = PAGE_SIZE_LEFT;
        }
        serial_flash_mem_write(addr_to_write, data_ptr, write_len);
        addr_to_write += write_len;
        data_ptr += write_len;
        remaining -= write_len;
    } while (remaining > 0);

    nvs->data_write_addr += data_len;
}

static MetaBlockStatus get_block_status(NvsAte *block, uint16_t offset)
{
    if (is_block_erased(block))
    {
        return META_BLOCK_ERASED;
    }
    if (is_meta_block_valid(block))
    {
        if (is_closing_block(block, offset))
        {
            return META_BLOCK_CLOSING;
        }
        else if (is_block_zero(block))
        {
            return META_BLOCK_ZERO;
        }
        else
        {
            return META_BLOCK_IS_VALID;
        }
    }
    else
    {
        if (is_block_zero(block))
        {
            return META_BLOCK_ZERO;
        }
        else
        {
            return META_BLOCK_NOT_VALID;
        }
    }
}

static int read_meta_block(uint32_t addr, NvsAte *block)
{
    ASSERT(addr % sizeof(NvsAte) == 0);
    return serial_flash_mem_read(addr, block, sizeof(NvsAte));
}

static void clear_address_pointers(Nvs *nvs)
{
    nvs->ate_write_addr  = 0;
    nvs->ate_read_addr   = 0;
    nvs->start_read_addr = 0;
    nvs->data_write_addr = 0;
}

static int find_write_addr(Nvs *nvs)
{
    //! Initialize both write and read pointers.
    nvs->ate_write_addr  = 0;
    nvs->ate_read_addr   = 0;
    nvs->start_read_addr = 0;
    nvs->data_write_addr = 0;

    //! Start from the beginning sequentially up to the last sector.
    for (uint32_t sec = nvs->first_sector; sec <= nvs->last_sector; sec++)
    {
        const uint32_t SEC_BASE_ADDR    = nvs->sector_size * sec;
        const uint32_t FIRST_BLOCK_ADDR = SEC_BASE_ADDR + nvs->sector_size - sizeof(NvsAte);
        uint32_t       block_addr       = FIRST_BLOCK_ADDR;

        uint32_t valid_blocks_count               = 0;
        uint32_t zero_blocks_count                = 0;
        uint32_t blocks_count                     = 0;
        uint32_t consecutive_invalid_blocks_count = 0;
        uint32_t erased_blocks_found              = 0;
        do
        {
            NvsAte block = BLOCK_INIT_VALUE;
            blocks_count++;
            read_meta_block(block_addr, &block);

            MetaBlockStatus status = get_block_status(&block, (block_addr & BLOCK_OFFSET_MASK));
            if (status != META_BLOCK_NOT_VALID)
            {
                consecutive_invalid_blocks_count = 0;
            }
            if (status == META_BLOCK_ERASED)
            {
                erased_blocks_found++;
                if (block_addr == FIRST_BLOCK_ADDR)
                {
                    //! First block is erased. This sector is our potential write head
                    //! if we have already found data in previous sectors or if this is the first sector.
                    nvs->ate_write_addr  = block_addr;
                    nvs->data_write_addr = SEC_BASE_ADDR;
                    return 0;
                }
                else
                {
                    nvs->ate_write_addr = block_addr;
                    return 0;
                }
                break;
            }
            else if (status == META_BLOCK_CLOSING)
            {
                break;
            }
            else if (status == META_BLOCK_ZERO)
            {
                zero_blocks_count++;
                nvs->data_write_addr = SEC_BASE_ADDR + block.offset + block.len;
            }
            else if (status == META_BLOCK_IS_VALID)
            {
                //! Assign the address of the first valid block within the head.
                //! This is in case only one sector is currently used.
                valid_blocks_count++;
                nvs->data_write_addr = SEC_BASE_ADDR + block.offset + block.len;
            }
            else if (status == META_BLOCK_NOT_VALID)
            {
                if (consecutive_invalid_blocks_count++ > 16)
                {
                    nvs->ate_write_addr  = 0;
                    nvs->data_write_addr = 0;
                    erase_sector(nvs, sec);
                    break;
                }
            }

            block_addr -= sizeof(NvsAte);

        } while ((block_addr >= SEC_BASE_ADDR) && (blocks_count < 7280));
    }
    return -1;
}

static int find_read_addr(Nvs *nvs)
{
    NVS_LOG("\r\n[NVS] === FIND READ ADDR START ===\r\n");
    NVS_LOG("[NVS] Write head at: %lu (sec %lu)\r\n", nvs->ate_write_addr, (nvs->ate_write_addr & ADDR_SECT_MASK) >> 16);
    nvs->ate_read_addr   = nvs->ate_write_addr;
    nvs->start_read_addr = nvs->ate_write_addr;

    //! The sector where write_addr is.
    const uint32_t WRITE_SEC     = sector_number_from_address(nvs->ate_write_addr);
    const uint32_t total_sectors = (nvs->last_sector - nvs->first_sector) + 1;

    //! Scan ALL sectors in chronological order, starting from next_sector(WRITE_SEC)
    //! (the oldest populated sector) and ending with WRITE_SEC itself (last resort).
    //!
    //! This guarantees FIFO oldest-first on remount:
    //!   Multi-sector case: the oldest non-write sector contains old data and is found first.
    //!   Single-sector / fresh-start: all non-write sectors are empty, so we fall through and
    //!   find the oldest ATE in the write sector itself on the final iteration.
    uint32_t sec = next_sector(nvs, WRITE_SEC);

    NVS_LOG("[NVS] Searching %lu sectors starting from sec %lu (oldest)\r\n", total_sectors, sec);
    for (uint32_t i = 0; i < total_sectors; i++)
    {
        const uint32_t SEC_BASE_ADDR    = nvs->sector_size * sec;
        const uint32_t FIRST_BLOCK_ADDR = SEC_BASE_ADDR + nvs->sector_size - sizeof(NvsAte);
        uint32_t       block_addr       = FIRST_BLOCK_ADDR;

        uint32_t blocks_count                     = 0;
        uint32_t consecutive_invalid_blocks_count = 0;
        do
        {
            NvsAte block = BLOCK_INIT_VALUE;
            blocks_count++;
            read_meta_block(block_addr, &block);

            MetaBlockStatus status = get_block_status(&block, (block_addr & BLOCK_OFFSET_MASK));
            if (status != META_BLOCK_NOT_VALID)
            {
                consecutive_invalid_blocks_count = 0;
            }

            if (status == META_BLOCK_ERASED)
            {
                break;
            }
            else if (status == META_BLOCK_IS_VALID)
            {
                NVS_LOG("[NVS] Found valid tail at: %lu (sec %lu)\r\n", block_addr, sec);
                nvs->ate_read_addr   = block_addr;
                nvs->start_read_addr = block_addr;
                NVS_LOG("[NVS] === FIND READ ADDR END: tail=%lu ===\r\n\r\n", nvs->ate_read_addr);
                return 0;
            }
            else if (status == META_BLOCK_CLOSING)
            {
                break;
            }
            else if (status == META_BLOCK_NOT_VALID)
            {
                if (++consecutive_invalid_blocks_count >= 16)
                {
                    break;
                }
            }

            block_addr -= sizeof(NvsAte);

        } while ((block_addr >= SEC_BASE_ADDR) && (blocks_count < 7280));

        NVS_LOG("[NVS] No valid tail in sec %lu, moving to next\r\n", sec);
        sec = next_sector(nvs, sec);
    }

    NVS_LOG("[NVS] === FIND READ ADDR END: tail=%lu (NVS empty) ===\r\n\r\n", nvs->ate_read_addr);
    return 0;
}

/**
 * @brief Checks if a block is erased.
 *
 * @param block Pointer to the block.
 * @return True if the block is erased, otherwise false.
 */
static bool is_block_erased(NvsAte *block)
{
    if ((block->id == 0xFFFF) && (block->offset == 0xFFFF) && (block->len == 0xFFFF) && (block->part == 0xFF) &&
        (block->crc8 == 0xFF))
    {
        return true;
    }
    return false;
}

/**
 * @brief Checks if a block contains all zeros.
 *
 * @param block Pointer to the block.
 * @return True if the block contains all zeros, otherwise false.
 */
static bool is_block_zero(NvsAte *block)
{
    if ((block->id == 0) && (block->part == 0xFF))
    {
        return true;
    }
    return false;
}

/**
 * @brief Checks if the given block is a closing block.
 *
 * This function determines whether the provided block is a closing block based on various
 * criteria, including its ID, offset, and length.
 *
 * @param block Pointer to the `NvsAte` block to be checked.
 * @param offset Offset value associated with the block.
 * @return Returns `true` if the block is a closing block, `false` otherwise.
 */
static bool is_closing_block(const NvsAte *block, uint16_t offset)
{
    if (block->id != 0xFFFF)
    {
        return false;
    }
    else if (block->offset != offset)
    {
        return false;
    }
    else if (block->len != 0)
    {
        return false;
    }
    else if (block->part != 0xFF)
    {
        return false;
    }
    else
    {
        return (true);
    }
}

/**
 * @brief Checks if the provided meta block is valid.
 *
 * This function verifies the validity of the provided meta block based on various criteria
 * such as CRC, ID, partition, and length.
 *
 * @param meta_ptr Pointer to the `NvsAte` meta block to be checked.
 * @return Returns `true` if the meta block is valid, `false` otherwise.
 */
static bool is_meta_block_valid(const NvsAte *meta_ptr)
{
    if (meta_ptr == NULL)
    {
        return false;
    }
    if (meta_ptr->crc8 != crc8((uint8_t *)meta_ptr, sizeof(NvsAte) - 1))
    {
        return false;
    }
    if ((meta_ptr->id > MAXIMUM_VALID_ID) && ((meta_ptr->id != 0xFFFF)))
    {
        return false;
    }
    if (meta_ptr->part != 0xFF)
    {
        return false;
    }
    if (meta_ptr->len > MAXIMUM_DATA_LEN)
    {
        return false;
    }
    return true;
}

static inline uint32_t next_sector(const Nvs *nvs, uint32_t sec_num)
{
    return (sec_num == nvs->last_sector) ? nvs->first_sector : (sec_num + 1);
}

static inline uint32_t sector_number_from_address(uint32_t addr)
{
    return (addr & ADDR_SECT_MASK) >> 16;
}

/**
 * @brief Erases the specified sector in the non-volatile storage.
 *
 * This function erases a 64k block at the specified sector address.
 *
 * @param nvs Pointer to the `Nvs` structure.
 * @param sec_num Sector number to be erased.
 */
static void erase_sector(Nvs *nvs, uint16_t sec_num)
{
    const uint32_t addr = get_sector_address(nvs, sec_num);
    serial_flash_mem_erase_64k_block(addr);
}

static uint32_t get_sector_address(Nvs *nvs, int sec_num)
{
    return sec_num * nvs->sector_size;
}

/**
 * @brief Prints the metadata information.
 *
 * This function prints the ID, offset, length, part, and CRC8 value of the given metadata structure.
 *
 * @param meta Pointer to the `NvsAte` structure containing the metadata.
 */
static void print_meta(NvsAte *meta)
{
    NVS_LOG("Meta:\r\n");
    NVS_LOG("id = %d\r\n", meta->id);
    NVS_LOG("offset = %d\r\n", meta->offset);
    NVS_LOG("len = %d\r\n", meta->len);
    NVS_LOG("part = %d\r\n", meta->part);
    NVS_LOG("crc8 = %d\r\n", meta->crc8);
}
