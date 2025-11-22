# syntax=docker/dockerfile:1

FROM ubuntu:24.04

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    clang-format \
    cmake \
    g++ \
    gcc \
    gcc-arm-none-eabi \
    git \
    libgtest-dev \
    ninja-build \
    protobuf-compiler \
    python3-protobuf \
 && rm -rf /var/lib/apt/lists/*
