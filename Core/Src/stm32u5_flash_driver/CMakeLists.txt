add_library(stm32u5_flash_driver STATIC)

target_sources(stm32u5_flash_driver PRIVATE stm32u5_flash_driver.h stm32u5_flash_driver.c)
target_include_directories(stm32u5_flash_driver PUBLIC . ${CMAKE_CURRENT_SOURCE_DIR}/../nvs)
target_link_libraries(stm32u5_flash_driver PUBLIC stm32cubemx)
