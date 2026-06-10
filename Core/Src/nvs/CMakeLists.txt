add_library(nvs nvs.c nvs_flash_driver.c)
target_include_directories(nvs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../crc_gen)
target_link_libraries(nvs PRIVATE crc8 trace_logger stm32u5_flash_driver)