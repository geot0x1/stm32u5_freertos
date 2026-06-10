add_library(nvs STATIC)

target_sources(nvs PRIVATE nvs.h nvs.c)
target_include_directories(nvs PUBLIC .)
target_link_libraries(nvs PUBLIC freertos_kernel PRIVATE serial_flash_mem trace_logger assert_macro crc8)
if (CONFIG_NVS_MODULE_DEBUG)
target_compile_definitions(nvs PRIVATE CONFIG_NVS_MODULE_DEBUG)
endif()