#!/usr/bin/env bash

set -e

if [ -z "$CCACHE_DIR" ]; then
    echo "CCACHE_DIR environment variable is not set."
    exit 1
fi

sudo apt-get -yqq install ccache
mkdir -p "${CCACHE_DIR}"
