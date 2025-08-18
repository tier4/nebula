#!/usr/bin/env bash

set -e

# Path to Apt's package cache file
PKGCACHE_FILE=/var/cache/apt/pkgcache.bin

# Maximum accepted age for the cache in seconds
MAX_CACHE_AGE_S=900 # 15 minutes

t_last_update_s=$(stat -c %Y "$PKGCACHE_FILE")

# Get the current time
t_now_s=$(date +%s)

# Calculate the age of the cache
cache_age_s=$((t_now_s - t_last_update_s))

# Check if the cache is older than the defined maximum age
if [ "$cache_age_s" -gt "$MAX_CACHE_AGE_S" ]; then
    echo "Apt package cache is older than $MAX_CACHE_AGE_S seconds. Running apt-get update..."
    sudo apt-get update
else
    echo "Apt package cache is recent enough. Using cached packages."
fi
