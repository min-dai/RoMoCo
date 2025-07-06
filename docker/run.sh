#!/bin/bash
set -e
set -u

SCRIPTROOT="$( cd "$(dirname "$0")" ; pwd -P )"
cd "${SCRIPTROOT}/.."






docker run -it --rm \
    -v $PWD:/home/docker/biped_simulation\
    --user docker \
    --network host --ipc host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
    biped-sim



