add_library(freertos_config INTERFACE)

target_sources(freertos_config INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSConfig.h)

target_include_directories(freertos_config INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(freertos_config INTERFACE stm32cubemx)