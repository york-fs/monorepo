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
    latexmk \
    libgtest-dev \
    ninja-build \
    plantuml \
    protobuf-compiler \
    texlive \
    texlive-latex-extra \
    texlive-luatex \
 && rm -rf /var/lib/apt/lists/*
