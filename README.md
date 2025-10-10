# stm-can

STM32 firmware for York Formula Student.

## Project structure

* `src/apps.cc` - Accelerator pedal position sensor firmware
* `src/bms.cc` - Battery management system firmware
* `src/bms_master.cc` - Battery management system master firmware
* `src/loggingboard.cc` - Logging board firmware
* `system/` - CMSIS and startup code for Cortex-M3
* `test/` - Host-runnable unit tests for platform independent code

## Building the firmware

### Using Docker (Recommended for macOS/Windows)

The easiest way to build the firmware is using the provided Docker environment, which includes all necessary dependencies:

    # Build the Docker image (first time only)
    docker compose build

    # Start a container and get a shell
    docker compose run --rm builder

    # Inside the container, build the firmware
    cmake --preset release -GNinja
    cmake --build build-release

The build artifacts will be available in the `build-release/` directory on your host machine.

### Building natively

Ensure a suitable ARM toolchain is in the `PATH` and run

    cmake --preset release -GNinja
    cmake --build build-release

### Flashing with stlink

If [stlink](https://github.com/texane/stlink) is installed, flash targets will be available for each executable. For
example, `flash-apps`.

**Note for macOS users:** Due to USB passthrough limitations, it's recommended to flash firmware outside the Docker container. Install `stlink` on your host machine using `sudo port install stlink`, then run:

    st-flash --reset --connect-under-reset write REPLACE_WITH_RELEVANT_FILE.bin 0x8000000

## Building the unit tests

    cmake --preset host -GNinja
    cmake --build build-host
    ./build-host/tests
