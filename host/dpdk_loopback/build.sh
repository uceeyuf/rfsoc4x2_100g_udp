#!/bin/sh
# Build dpdk_loopback against the installed DPDK (pkg-config libdpdk).
set -e
cd "$(dirname "$0")"
cc -O3 -march=native -Wall -Wno-address-of-packed-member \
    $(pkg-config --cflags libdpdk) dpdk_loopback.c -o dpdk_loopback \
    $(pkg-config --libs libdpdk) -lm -lpthread
echo "built $(pwd)/dpdk_loopback (DPDK $(pkg-config --modversion libdpdk))"
