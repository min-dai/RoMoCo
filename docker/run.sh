#!/bin/bash
set -e
set -u

SCRIPTROOT="$( cd "$(dirname "$0")" ; pwd -P )"
cd "${SCRIPTROOT}/.."





# docker run -it --rm \
#     -v $PWD:/home/docker/RoMoCo\
#     --user docker \
#     --network host --ipc host \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
#     biped-sim

# Allow X11 GUI access (if using GUI apps)
xhost +local:docker || true


# Launch container
docker run -it --rm \
    --user docker \
    --network host \
    --ipc host \
    -v "$PWD":/home/docker/RoMoCo \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    -e HOME=/home/docker \
    -w /home/docker/RoMoCo \
    biped-sim

