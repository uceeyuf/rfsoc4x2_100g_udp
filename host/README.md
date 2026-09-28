# Linux host (DPDK)

Two programs on the Mellanox ConnectX-4 with DPDK (mlx5 PMD), polled, no kernel in the path:

* `dpdk_stream_rx`: receives the DDR record stream and checks every sample (ramp test data);
  one line per second with packets, Gbit/s, lost packets, wrong samples and NIC drops.
* `dpdk_loopback`: 4K (or any size) video loopback through the FPGA UDP echo; 4 transmit cores,
  8 receive cores (RSS on IP addresses and UDP ports), every frame reassembled and compared
  byte by byte (`--ref stored`, the picture frames; `--ref gen`, a pattern computed from frame
  id and byte offset, no reference reads).

## Once

DPDK 24.11 or later with the mlx5 driver (`-Denable_drivers=bus/pci,bus/vdev,bus/auxiliary,common/mlx5,net/mlx5,mempool/ring,mempool/stack`);
the DPDK 23.11 of Ubuntu 24.04 limits the mlx5 receive queues of a ConnectX-4 without DevX to one descriptor.

```sh
sudo apt install build-essential pkg-config meson ninja-build python3-pyelftools rdma-core ibverbs-providers libibverbs-dev
# build DPDK 24.11: meson setup build --prefix=$HOME/dpdk --libdir=lib -Denable_drivers=...; ninja -C build install
export PKG_CONFIG_PATH=$HOME/dpdk/lib/pkgconfig
dpdk_stream_rx/build.sh
dpdk_loopback/build.sh
```

## Each boot

```sh
IF=enp2s0np0                                        # ConnectX-4 port (ip link)
sudo ethtool -s $IF speed 100000 autoneg off        # the CMAC runs 100G without auto-negotiation
sudo ethtool --set-fec $IF encoding rs              # ... and with RS-FEC
sudo ip link set $IF up mtu 9000
echo 1536 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages   # 3 GB of 2 MB pages
lspci -D | grep Mellanox                            # PCI address, e.g. 0000:02:00.0
```

The FPGA is `192.168.100.1` (alias addresses `.128`–`.159`); the programs use `192.168.100.2` and
answer ARP themselves while DPDK owns the port. For the kernel-stack tests (`tests/loopback_test.py`,
`tests/restart_check.tcl`) give the interface `192.168.100.2/24` instead.

## Run

```sh
sudo env LD_LIBRARY_PATH=$HOME/dpdk/lib dpdk_stream_rx/dpdk_stream_rx -l 0-8 -a 0000:02:00.0 -- --seconds 60
vivado -mode batch -source ../scripts/vio_speed.tcl -tclargs 1 0 2096      # start: 16 flows + IP rotation + jumbo
vivado -mode batch -source ../scripts/vio_speed.tcl -tclargs 0             # stop

sudo env LD_LIBRARY_PATH=... dpdk_loopback/dpdk_loopback -l 0-12 -a 0000:02:00.0 -- --sweep 120,240,360 --out out_4k
sudo env LD_LIBRARY_PATH=... dpdk_loopback/dpdk_loopback -l 0-12 -a 0000:02:00.0 -- --ref gen --fps 440 --out out_4k440
sudo env LD_LIBRARY_PATH=... dpdk_loopback/dpdk_loopback -l 0-12 -a 0000:02:00.0 -- --res 7680x4320 --sweep 30,45,60 --out out_8k
```

`dpdk_loopback`: `-l 0-12` = 1 main + 4 TX + 8 RX cores (`--txq`, `--rxq`); a sweep stops after two
failed rates; `out_*/summary.txt` has one line per rate and the ceiling, `sent.ppm` / `received.ppm`
are a frame and its echo at the highest passing rate. `dpdk_stream_rx`: `-l 0-8` = 1 main + 8 RX cores.
