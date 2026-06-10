add_library(serial_flash_mem STATIC)

target_sources(serial_flash_mem PRIVATE serial_flash_mem.h serial_flash_mem.c)
target_include_directories(serial_flash_mem PUBLIC .)
