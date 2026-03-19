#!/usr/bin/env bash

set -e

if [ -z "$AGNOCAST_VERSION" ]; then
    echo "AGNOCAST_VERSION environment variable is not set."
    exit 1
fi

sudo apt-get install -yqq agnocast-{kmod,heaphook}-v"${AGNOCAST_VERSION}"
