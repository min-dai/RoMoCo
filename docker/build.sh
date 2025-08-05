#!/bin/bash
set -e
set -u

# SCRIPTROOT="$( cd "$(dirname "$0")" ; pwd -P )"
# cd "${SCRIPTROOT}/.."

# docker build -t biped-sim -f docker/Dockerfile .




# Resolve the root of the RoMoCo project
SCRIPTROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROMOCO_ROOT="${SCRIPTROOT}/.."

cd "${ROMOCO_ROOT}"

# Build the Docker image using the Dockerfile in docker/
docker build  -t biped-sim -f docker/Dockerfile .
