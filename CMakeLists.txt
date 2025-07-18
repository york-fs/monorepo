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

function(add_stm_executable name frequency)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE protodef shared-stm)

    # Rebuild on any linker script changes.
    set_target_properties(${name} PROPERTIES LINK_DEPENDS "${STM_LD_SCRIPT}")

    if(CMAKE_SIZE)
        add_custom_command(TARGET ${name}
            POST_BUILD COMMAND "${CMAKE_SIZE}" ${name})
    endif()

    if(CMAKE_OBJCOPY)
        add_custom_command(TARGET ${name}
            POST_BUILD COMMAND "${CMAKE_OBJCOPY}" -O binary ${name} ${name}.bin)
    endif()

    if(CMAKE_OBJCOPY AND ST_FLASH_PROGRAM)
        add_custom_target(flash-${name} ${ST_FLASH_PROGRAM} --reset --connect-under-reset write ${name}.bin 0x8000000)
        add_dependencies(flash-${name} ${name})
    endif()

    # Add a trace target for SWD output.
    if(ST_TRACE_PROGRAM)
        add_custom_target(trace-${name}
            ${ST_TRACE_PROGRAM} -c${frequency}
            USES_TERMINAL)
        add_dependencies(trace-${name} flash-${name})
    endif()
endfunction()

# Create an OBJECT library for platform-independent code.
add_library(shared OBJECT
    src/bms_logic.cc
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
    # Create a library for shared STM code.
    add_library(shared-stm STATIC
        src/can.cc
        src/eeprom.cc
        src/hal.cc
        src/max_adc.cc
        src/miniprintf.c
        system/startup_stm32f103c8tx.s)
    target_include_directories(shared-stm SYSTEM PUBLIC system)
    target_link_libraries(shared-stm PUBLIC nanopb shared)
    target_link_options(shared-stm PUBLIC -T "${STM_LD_SCRIPT}")

    add_stm_executable(apps 56 src/apps.cc)
    add_stm_executable(bms 8 src/bms.cc)
    add_stm_executable(bms_master 56 src/bms_master.cc)
else()
    message(FATAL_ERROR "Unknown target ${BUILD_TARGET}")
endif()
