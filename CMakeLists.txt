cmake_minimum_required(VERSION 3.25)
project(stm-can ASM C CXX)

add_executable(stm_can
    src/main.cc
    system/startup_stm32f103c8tx.s)
target_compile_features(stm_can PRIVATE cxx_std_20)
target_include_directories(stm_can SYSTEM PRIVATE system)
target_link_options(stm_can PRIVATE -T "${CMAKE_CURRENT_SOURCE_DIR}/system/STM32F103C8TX_FLASH.ld")
set_target_properties(stm_can PROPERTIES LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/system/STM32F103C8TX_FLASH.ld")

if(CMAKE_SIZE)
    add_custom_command(TARGET stm_can
        POST_BUILD COMMAND "${CMAKE_SIZE}" stm_can)
endif()

if(CMAKE_OBJCOPY)
    add_custom_command(TARGET stm_can
        POST_BUILD COMMAND "${CMAKE_OBJCOPY}" -O binary stm_can stm_can.bin)
endif()

find_program(STFLASH_PROGRAM NAMES st-flash)
if(CMAKE_OBJCOPY AND STFLASH_PROGRAM)
    add_custom_target(flash ${STFLASH_PROGRAM} --reset --connect-under-reset write stm_can.bin 0x8000000)
    add_dependencies(flash stm_can)
endif()
