#!/bin/bash

# Settings required for having intel integrated graphics acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 "

DOCKER_COMMAND="docker run"

$DOCKER_COMMAND -it \
    $DOCKER_GPU_ARGS \
    -v "$PWD/docker_ws:/home/ws/src" \
    --name=tec \
    tec\
    bash