cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(io_capture C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(io_capture
    src/main.c
)

pico_generate_pio_header(io_capture
    ${CMAKE_CURRENT_LIST_DIR}/src/io_capture.pio
)

target_include_directories(io_capture PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(io_capture
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_timer
)

pico_enable_stdio_usb(io_capture 1)
pico_enable_stdio_uart(io_capture 0)

pico_add_extra_outputs(io_capture)
