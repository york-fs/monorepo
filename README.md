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

Ensure a suitable ARM toolchain is in the `PATH` and run

    cmake --preset release -GNinja
    cmake --build build-release
    
### Flashing with stlink

If [stlink](https://github.com/texane/stlink) is installed, flash targets will be available for each executable. For
example, `flash-apps`.

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
