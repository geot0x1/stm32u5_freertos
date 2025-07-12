add_library(drv8825 STATIC)

target_sources(drv8825
               PRIVATE
               drv8825.c
               drv8825.h)

target_include_directories(drv8825
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(drv8825
                      PUBLIC
                      stm32cubemx
                      PRIVATE
                      freertos_kernel)