#!/usr/bin/env bash

set -e

if [ -z "$AGNOCAST_VERSION" ]; then
    echo "AGNOCAST_VERSION environment variable is not set."
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO environment variable is not set."
    exit 1
fi

sudo apt-add-repository -y ppa:t4-system-software/agnocast
sudo apt-get install -yqq agnocast-{kmod,heaphook}-v"${AGNOCAST_VERSION}"

# Keep CI runtime defaults aligned with README.md.
AGNOCAST_MEMPOOL_SIZE="${AGNOCAST_MEMPOOL_SIZE:-134217728}"
LD_PRELOAD="/opt/ros/${ROS_DISTRO}/lib/libagnocast_heaphook.so"

if [ ! -f "${LD_PRELOAD}" ]; then
    echo "Expected heaphook library not found: ${LD_PRELOAD}"
    exit 1
fi

{
    echo "AGNOCAST_MEMPOOL_SIZE=${AGNOCAST_MEMPOOL_SIZE}"
    echo "LD_PRELOAD=${LD_PRELOAD}"
} >>"${GITHUB_ENV}"
