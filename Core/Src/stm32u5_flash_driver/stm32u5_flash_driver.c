#include "stm32u5_flash_driver.h"
#include "stm32u5xx_hal.h"
#include <string.h>
#include <stdio.h>

#define STM32U5_FLASH_BASE_ADDR  (0x08000000UL)
#define STM32U5_BANK_SIZE        (0x40000UL)
#define STM32U5_PAGE_SIZE        (0x2000U)
#define STM32U5_DWORD_SIZE       (8U)
#define STM32U5_QUADWORD_SIZE    (16U)

#define NVS_PHYS_BASE_ADDR       (0x08070000UL)
#define NVS_PHYS_SECTOR_SIZE     (0x8000U)
#define NVS_SECTOR_COUNT         (2U)

/*
 * NVS issues 8-byte-aligned writes (STM32C0 granularity), but U5 flash
 * programs 16-byte quadwords, 16-byte aligned, once per erase cycle.
 * The driver therefore maps every logical 8-byte double-word onto its own
 * physical quadword: data in bytes 0-7, 0xFF padding in bytes 8-15.
 * NVS is given sectors of half the physical size so the mapped log
 * always fits inside the physical sector.
 */
#define NVS_LOGICAL_SECTOR_SIZE  (NVS_PHYS_SECTOR_SIZE / 2U)

static void stm32u5_flash_write(uint32_t addr, const void *data, uint16_t len);
static void stm32u5_flash_read(uint32_t addr, void *data, uint16_t len);
static void stm32u5_flash_erase_sector(uint32_t addr);
static uint32_t logical_to_physical(uint32_t logical_addr);
static void program_padded_quadword(uint32_t phys_addr, const uint8_t *data, uint16_t len);

static nvs_flash_driver_t driver =
{
    .write = stm32u5_flash_write,
    .read = stm32u5_flash_read,
    .erase_sector = stm32u5_flash_erase_sector,
    .base_addr = NVS_PHYS_BASE_ADDR,
    .sector_size = NVS_LOGICAL_SECTOR_SIZE,
    .sector_count = NVS_SECTOR_COUNT,
};

static uint32_t logical_to_physical(uint32_t logical_addr)
{
    uint32_t offset = logical_addr - NVS_PHYS_BASE_ADDR;
    uint32_t dword_index = offset / STM32U5_DWORD_SIZE;
    uint32_t byte_in_dword = offset % STM32U5_DWORD_SIZE;

    return NVS_PHYS_BASE_ADDR + (dword_index * STM32U5_QUADWORD_SIZE) + byte_in_dword;
}

static void program_padded_quadword(uint32_t phys_addr, const uint8_t *data, uint16_t len)
{
    uint32_t quadword[STM32U5_QUADWORD_SIZE / 4U];

    memset(quadword, 0xFF, sizeof(quadword));
    memcpy(quadword, data, len);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, phys_addr, (uint32_t)(uintptr_t)quadword) != HAL_OK)
    {
        printf("[FLASH] Program failed at 0x%08lx (error 0x%08lx)\n\r",
               phys_addr, HAL_FLASH_GetError());
    }
}

static void stm32u5_flash_write(uint32_t addr, const void *data, uint16_t len)
{
    const uint8_t *src = (const uint8_t *)data;
    uint16_t bytes_written = 0;

    if ((addr % STM32U5_DWORD_SIZE) != 0U)
    {
        printf("[FLASH] Rejected unaligned write at 0x%08lx\n\r", addr);
        return;
    }

    HAL_FLASH_Unlock();

    while (bytes_written < len)
    {
        uint16_t remaining = len - bytes_written;
        uint16_t chunk = remaining >= STM32U5_DWORD_SIZE ? STM32U5_DWORD_SIZE : remaining;

        program_padded_quadword(logical_to_physical(addr + bytes_written), &src[bytes_written], chunk);
        bytes_written += chunk;
    }

    HAL_FLASH_Lock();
    HAL_ICACHE_Invalidate();
}

static void stm32u5_flash_read(uint32_t addr, void *data, uint16_t len)
{
    uint8_t *dst = (uint8_t *)data;
    uint16_t bytes_read = 0;

    while (bytes_read < len)
    {
        uint32_t logical_addr = addr + bytes_read;
        uint16_t offset_in_dword = (uint16_t)(logical_addr % STM32U5_DWORD_SIZE);
        uint16_t chunk = (uint16_t)(STM32U5_DWORD_SIZE - offset_in_dword);
        uint16_t remaining = len - bytes_read;

        if (chunk > remaining)
        {
            chunk = remaining;
        }

        memcpy(&dst[bytes_read], (const void *)(uintptr_t)logical_to_physical(logical_addr), chunk);
        bytes_read += chunk;
    }
}

static void stm32u5_flash_erase_sector(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    uint32_t phys_addr = logical_to_physical(addr);
    uint32_t bank_offset = phys_addr - STM32U5_FLASH_BASE_ADDR;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = (bank_offset < STM32U5_BANK_SIZE) ? FLASH_BANK_1 : FLASH_BANK_2;
    erase_init.Page = (bank_offset % STM32U5_BANK_SIZE) / STM32U5_PAGE_SIZE;
    erase_init.NbPages = NVS_PHYS_SECTOR_SIZE / STM32U5_PAGE_SIZE;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
    HAL_ICACHE_Invalidate();

    if (status != HAL_OK)
    {
        printf("[FLASH] Erase failed at 0x%08lx (status %d, error 0x%08lx)\n\r",
               phys_addr, status, page_error);
    }
}

nvs_flash_driver_t stm32u5_flash_driver_get(void)
{
    return driver;
}
