#!/bin/sh
# Build dpdk_stream_rx against the installed DPDK (pkg-config libdpdk).
set -e
cd "$(dirname "$0")"
cc -O3 -march=native -Wall -Wno-address-of-packed-member \
    $(pkg-config --cflags libdpdk) dpdk_stream_rx.c -o dpdk_stream_rx \
    $(pkg-config --libs libdpdk) -lpthread
echo "built $(pwd)/dpdk_stream_rx (DPDK $(pkg-config --modversion libdpdk))"
