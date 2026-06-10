add_library(crc8 STATIC)

target_sources(crc8 PRIVATE crc_gen.h crc_gen.c)
target_include_directories(crc8 PUBLIC .)
