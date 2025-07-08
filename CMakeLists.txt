# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Name project
#set(ProjectName freertos_template)

# Set FreeRTOS Kernel path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

project(midi_test_freertos C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(midi_test_freertos main.c )
pico_set_program_name(midi_test_freertos "freertos_template")
pico_set_program_version(midi_test_freertos "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(midi_test_freertos 0)
pico_enable_stdio_usb(midi_test_freertos 0)

target_sources(midi_test_freertos PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/sh1106_i2c.c
        ${CMAKE_CURRENT_LIST_DIR}/sh1106_i2c.h
        ${CMAKE_CURRENT_LIST_DIR}/font_inconsolata.h
        ${CMAKE_CURRENT_LIST_DIR}/sequencer_memory.c
        ${CMAKE_CURRENT_LIST_DIR}/sequencer_memory.h
)

# Add the standard library to the build
target_link_libraries(midi_test_freertos
        pico_stdlib
        hardware_adc
	hardware_dma
        hardware_i2c
        FreeRTOS-Kernel-Heap4
        pico_cyw43_arch_none
        pico_unique_id
        tinyusb_device
        tinyusb_board
)

# Add the standard include files to the build
target_include_directories(midi_test_freertos PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(midi_test_freertos PRIVATE
        configNUMBER_OF_CORES=2
)

pico_add_extra_outputs(midi_test_freertos)

