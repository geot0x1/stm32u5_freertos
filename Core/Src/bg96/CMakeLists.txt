add_library(bg96 STATIC)

target_sources(bg96
               PRIVATE
               bg96.c
               bg96.h)

target_include_directories(bg96
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(bg96 PRIVATE STM32_Drivers stm32cubemx freertos_kernel)