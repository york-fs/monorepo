# YFS Electronics Monorepo

Code and documentation repository for the York Formula Student electronics team. Kicad projects are included as
submodules but remain separate repositories to allow for separate development. The submodules can then be pinned to
a specific tag which matches the firmware and documentation versions.

## Project structure

* `docs/` - Manuals, flowcharts, block diagrams, etc.
* `kicad/` - KiCad PCB projects
* `src/` - STM32 firmware
* `system/` - CMSIS and startup code for Cortex-M3
* `test/` - Host-runnable unit tests for the platform independent code

## Building the STM32 firmware

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

    st-flash --reset write build-release/apps.bin 0x8000000

## Building the unit tests

    cmake --preset host -GNinja
    cmake --build build-host
    ./build-host/tests

## Building in docker

    docker run --rm -u $(id -u) -v $(pwd):/src ghcr.io/york-fs/monorepo:master cmake \
     --preset release \
     -B/src/build-release \
     -GNinja \
     /src
    docker run --rm -u $(id -u) -v $(pwd):/src ghcr.io/york-fs/monorepo:master cmake --build /src/build-release


## Building in podman

    podman run --rm -v $(pwd):/src ghcr.io/york-fs/monorepo:master cmake \
     --preset release \
     -B/src/build-release \
     -GNinja \
     /src
    podman run --rm -v $(pwd):/src ghcr.io/york-fs/monorepo:master cmake --build /src/build-release
