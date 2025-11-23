#!/bin/bash
set -e

VOLUME_NAME=texlive
COMMAND="cd /src && latexmk -pdf -pdflatex=lualatex -silent manual"

if [[ -x "$(command -v podman)" ]]; then
    podman volume create --ignore $VOLUME_NAME

    podman run --rm \
      -v $(pwd):/src \
      -v $VOLUME_NAME:/var/lib/texmf/luatex-cache/ \
      ghcr.io/york-fs/monorepo:master \
      bash -c "$COMMAND"
else
    docker volume create $VOLUME_NAME

    docker run --rm \
      -v $VOLUME_NAME:/home/ubuntu/.texlive2023/ \
      ghcr.io/yeetari/latex:master \
      chmod 777 /home/ubuntu/.texlive2023

    docker run --rm \
      -u $(id -u) \
      -v $(pwd):/src \
      -v $VOLUME_NAME:/home/ubuntu/.texlive2023/ \
      ghcr.io/yeetari/latex:master \
      bash -c "$COMMAND"
fi
