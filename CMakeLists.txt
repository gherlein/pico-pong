cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(main    
    src/main.c
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    includes
    pico-sx126x/includes
    semtech-pico-hal/includes
)
add_subdirectory(pico-sx126x)
add_subdirectory(semtech-pico-hal)
    
target_link_libraries(main
	pico_stdlib
    pico_unique_id 
	hardware_spi
    pico-sx126x
    semtech-pico-hal
	# other libs as needed
)

add_custom_target(flash
    COMMAND /bin/bash ../flash
    DEPENDS main
)

pico_add_extra_outputs(main)