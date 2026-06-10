add_library(nvs nvs.c nvs_flash_driver.c)
target_include_directories(nvs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(nvs PRIVATE crc_gen trace_logger)