cmake_minimum_required(VERSION 3.25)
project(stm-can ASM C CXX)

option(BUILD_TARGET "Build target" "")

if(NOT ST_FLASH_PROGRAM)
    find_program(ST_FLASH_PROGRAM NAMES st-flash)
endif()
if(NOT ST_TRACE_PROGRAM)
    find_program(ST_TRACE_PROGRAM NAMES st-trace)
endif()

# Include nanopb.
set(NANOPB_SRC_ROOT_FOLDER "${CMAKE_SOURCE_DIR}/nanopb")
list(APPEND CMAKE_MODULE_PATH "${NANOPB_SRC_ROOT_FOLDER}/extra")
find_package(Nanopb REQUIRED)

# Generate protobuf header file.
nanopb_generate_cpp(TARGET protodef "${CMAKE_SOURCE_DIR}/protodef/vehicle_data.proto")

set(STM_LD_SCRIPT "${CMAKE_SOURCE_DIR}/system/STM32F103C8TX_FLASH.ld")

function(add_stm_executable target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "CLOCK;FREERTOS" "SOURCES")
    if(NOT DEFINED arg_CLOCK)
        set(arg_CLOCK 56)
    endif()

    add_executable(${target} "${arg_SOURCES}")
    target_link_libraries(${target} PRIVATE stm-base)

    if(CMAKE_SIZE)
        add_custom_command(TARGET ${target}
            POST_BUILD COMMAND "${CMAKE_SIZE}" ${target})
    endif()

    if(CMAKE_OBJCOPY)
        add_custom_command(TARGET ${target}
            POST_BUILD COMMAND "${CMAKE_OBJCOPY}" -O binary ${target} ${target}.bin)
    endif()

    if(CMAKE_OBJCOPY AND ST_FLASH_PROGRAM)
        add_custom_target(flash-${target} ${ST_FLASH_PROGRAM} --reset --connect-under-reset write ${target}.bin 0x8000000)
        add_dependencies(flash-${target} ${target})
    endif()

    # Add a trace target for SWD output.
    if(ST_TRACE_PROGRAM)
        add_custom_target(trace-${target}
            ${ST_TRACE_PROGRAM} -c${arg_CLOCK}
            USES_TERMINAL)
        add_dependencies(trace-${target} flash-${target})
    endif()

    if(DEFINED arg_FREERTOS)
        configure_file(${arg_FREERTOS} ${target}-include/FreeRTOSConfig.h)
        add_library(${target}-freertos OBJECT
            ${CMAKE_SOURCE_DIR}/freertos/croutine.c
            ${CMAKE_SOURCE_DIR}/freertos/event_groups.c
            ${CMAKE_SOURCE_DIR}/freertos/list.c
            ${CMAKE_SOURCE_DIR}/freertos/queue.c
            ${CMAKE_SOURCE_DIR}/freertos/stream_buffer.c
            ${CMAKE_SOURCE_DIR}/freertos/tasks.c
            ${CMAKE_SOURCE_DIR}/freertos/timers.c
            ${CMAKE_SOURCE_DIR}/freertos/portable/GCC/ARM_CM3/port.c
            ${CMAKE_SOURCE_DIR}/freertos/portable/MemMang/heap_4.c)
        target_compile_options(${target}-freertos PRIVATE -w)
        target_include_directories(${target}-freertos PUBLIC
            ${CMAKE_SOURCE_DIR}/freertos/include
            ${CMAKE_SOURCE_DIR}/freertos/portable/GCC/ARM_CM3
            ${CMAKE_CURRENT_BINARY_DIR}/${target}-include)
        target_link_libraries(${target} PRIVATE ${target}-freertos)
    endif()
endfunction()

function(target_add_freertos target config)
    configure_file(${config} ${target}-include/FreeRTOSConfig.h)
    add_library(${target}-freertos OBJECT
        ${CMAKE_SOURCE_DIR}/freertos/croutine.c
        ${CMAKE_SOURCE_DIR}/freertos/event_groups.c
        ${CMAKE_SOURCE_DIR}/freertos/list.c
        ${CMAKE_SOURCE_DIR}/freertos/queue.c
        ${CMAKE_SOURCE_DIR}/freertos/stream_buffer.c
        ${CMAKE_SOURCE_DIR}/freertos/tasks.c
        ${CMAKE_SOURCE_DIR}/freertos/timers.c
        ${CMAKE_SOURCE_DIR}/freertos/portable/GCC/ARM_CM3/port.c
        ${CMAKE_SOURCE_DIR}/freertos/portable/MemMang/heap_4.c)
    target_compile_options(${target}-freertos PRIVATE -w)
    target_include_directories(${target}-freertos PUBLIC
        ${CMAKE_SOURCE_DIR}/freertos/include
        ${CMAKE_SOURCE_DIR}/freertos/portable/GCC/ARM_CM3
        ${CMAKE_CURRENT_BINARY_DIR}/${target}-include)
    target_link_libraries(${target} PRIVATE ${target}-freertos)
endfunction()

# Create an OBJECT library for platform-independent code.
add_library(shared OBJECT
    src/bms_logic.cc
    src/charger/can_messages.cc
    src/dti.cc)
target_compile_features(shared PUBLIC cxx_std_20)
target_include_directories(shared PUBLIC src)

if(BUILD_TARGET STREQUAL "host")
    find_package(GTest REQUIRED)
    include(GoogleTest)
    enable_testing()

    add_executable(tests
        test/dti_test.cc
        test/util_test.cc)
    target_link_libraries(tests PRIVATE GTest::Main shared)
    gtest_discover_tests(tests)
elseif(BUILD_TARGET STREQUAL "stm32")
    # Create a library for the shared base STM code.
    add_library(stm-base STATIC
        src/hal.cc
        src/miniprintf.c
        system/startup_stm32f103c8tx.s)
    target_include_directories(stm-base SYSTEM PUBLIC system)
    target_link_libraries(stm-base PUBLIC nanopb shared)
    target_link_options(stm-base PUBLIC -T "${STM_LD_SCRIPT}")

    add_stm_executable(bms-segment
        SOURCES src/bms/segment.cc
        FREERTOS src/bms/segment_freertos_config.h
        CLOCK 8)
    add_stm_executable(bms-master
        SOURCES src/bms/master.cc src/can.cc src/max_adc.cc
        FREERTOS src/bms/master_freertos_config.h)
    add_stm_executable(charger
        SOURCES src/charger/charger.cc src/can.cc
        FREERTOS src/charger/freertos_config.h)
    add_stm_executable(logging-board
        SOURCES src/logging_board.cc)
    add_stm_executable(precharge
        SOURCES src/precharge/precharge.cc
        FREERTOS src/precharge/freertos_config.h)
    target_link_libraries(logging-board PRIVATE protodef)
else()
    message(FATAL_ERROR "Unknown target ${BUILD_TARGET}")
endif()
