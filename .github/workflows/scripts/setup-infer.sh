#!/usr/bin/env bash

set -e

if [ -z "$FB_INFER_VERSION" ]; then
    echo "FB_INFER_VERSION environment variable is not set."
    exit 1
fi

export basename="infer-linux-x86_64-v${FB_INFER_VERSION}"
export filename="${basename}.tar.xz"
sudo apt-get -yqq install curl tar
curl -sSLO "https://github.com/facebook/infer/releases/download/v${FB_INFER_VERSION}/${filename}"
tar -xf "$filename"
rm "$filename"
sudo mv "$basename" /usr/local/infer
echo "PATH=/usr/local/infer/bin:$PATH" >>"$GITHUB_ENV"
echo "Infer reports version:"
/usr/local/infer/bin/infer --version
echo "Set up Facebook Infer successfully."
