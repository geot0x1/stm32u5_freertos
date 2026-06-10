#ifndef NVS_H
#define NVS_H

#include <stdint.h>
#include <stddef.h>

/*===========================================================================
 *  Flash layout constraints (STM32C0xx)
 *
 *  - Programs in 8-byte (64-bit) double-words only.
 *  - Target block must be fully erased (0xFF) before programming.
 *  - No in-place byte updates are possible.
 *
 *  NVS design:
 *  - Sector header: 8 bytes (magic + seq). Written once, atomically.
 *  - Entries: written atomically with state = VALID from the start.
 *    No two-phase commit (avoids writing the same block twice).
 *  - Old entries (superseded by a newer write of the same key) are NOT
 *    marked deleted in-place. The reader scans forward and takes the last
 *    match, so superseded entries are transparently ignored.
 *  - GC copies only the latest entry for each key, then erases the old sector.
 *===========================================================================*/

/*===========================================================================
 *  Constants
 *===========================================================================*/

/** Sector header magic word: "NVS!" in little-endian */
#define NVS_MAGIC_WORD (0x4E565321U)

/** Entry state byte values */
#define NVS_ENTRY_VALID (0xFEU) /**< Entry is committed and readable     */

/** Size limits */
#define NVS_MAX_KEY_LEN (15U)
#define NVS_MAX_DATA_LEN (128U)

/**
 * Sector header: 8 bytes = magic(4) + seq(4).
 * Written as a single 8-byte flash write.
 */
#define NVS_SECTOR_HDR_SIZE (8U)

/**
 * Entry fixed header: 8 bytes.
 *  Byte 0   : state  (NVS_ENTRY_VALID or 0xFF = erased / end-of-log)
 *  Byte 1   : key_len
 *  Byte 2   : data_len
 *  Byte 3   : reserved (0xFF)
 *  Bytes 4-7: CRC32 over (key_len + data_len + key[] + data[])
 */
#define NVS_ENTRY_HDR_SIZE (8U)

/*===========================================================================
 *  Types
 *===========================================================================*/

typedef enum
{
    NVS_OK = 0,
    NVS_ERR_NOT_FOUND,
    NVS_ERR_NO_SPACE,
    NVS_ERR_FLASH,
    NVS_ERR_CRC,
    NVS_ERR_INVALID_ARG
} nvs_err_t;

/*===========================================================================
 *  Flash driver interface — injected at mount time
 *===========================================================================*/

typedef struct
{
    void (*write)(uint32_t addr, const void *data, uint16_t len);
    void (*read)(uint32_t addr, void *data, uint16_t len);
    void (*erase_sector)(uint32_t addr);

    /** Absolute flash address of the first NVS sector. */
    uint32_t base_addr;

    /** Size of one flash sector in bytes (must be a multiple of 8). */
    uint32_t sector_size;

    /** Number of sectors allocated to NVS. */
    uint8_t sector_count;
} nvs_flash_driver_t;

/*===========================================================================
 *  RAM context
 *===========================================================================*/

typedef struct
{
    uint32_t active_sector_addr;
    uint32_t write_offset;
    uint32_t seq_counter;
    nvs_flash_driver_t driver;
} nvs_context_t;

/*===========================================================================
 *  Public API
 *===========================================================================*/

nvs_err_t nvs_mount(const nvs_flash_driver_t *driver);
nvs_err_t nvs_write(const char *key, const void *data, uint8_t len);
nvs_err_t nvs_read(const char *key, void *buf, uint8_t buf_size, uint8_t *out_len);
nvs_err_t nvs_delete(const char *key);

#endif /* NVS_H */