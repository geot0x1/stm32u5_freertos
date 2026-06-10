#ifndef SERIAL_FLASH_MEM_H
#define SERIAL_FLASH_MEM_H

#include <stdint.h>
#include <stddef.h>

void serial_flash_mem_init(void);
void serial_flash_mem_erase_64k_block(uint32_t address);
void serial_flash_mem_write(uint32_t address, const void *data, size_t length);
int serial_flash_mem_read(uint32_t address, void *data, size_t length);

#endif
