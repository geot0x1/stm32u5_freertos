add_library(fifo STATIC)

target_sources(fifo PRIVATE fifo.h fifo.c)
target_include_directories(fifo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
