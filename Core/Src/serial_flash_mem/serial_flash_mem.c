#include "serial_flash_mem.h"

void serial_flash_mem_init(void)
{
}

void serial_flash_mem_erase_64k_block(uint32_t address)
{
    (void)address;
}

void serial_flash_mem_write(uint32_t address, const void *data, size_t length)
{
    (void)address;
    (void)data;
    (void)length;
}

int serial_flash_mem_read(uint32_t address, void *data, size_t length)
{
    (void)address;
    (void)data;
    (void)length;
    return 0;
}
