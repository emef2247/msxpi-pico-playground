cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(msx_bus_logger C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(msx_bus_logger
    msx_bus_logger/src/main.c
)

target_include_directories(msx_bus_logger PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(msx_bus_logger
    pico_stdlib
    hardware_timer
)

pico_enable_stdio_usb(msx_bus_logger 1)
pico_enable_stdio_uart(msx_bus_logger 0)

pico_add_extra_outputs(msx_bus_logger)