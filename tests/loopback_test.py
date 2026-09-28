#!/usr/bin/env python3
"""
UDP loopback test: send packets to the FPGA echo ports 1234 / 1235 and check that every
packet comes back unchanged (source and destination ports swapped).

    python3 loopback_test.py            # 1000 packets per port
    python3 loopback_test.py 5000

Standard library only.
"""

import os
import socket
import sys
import time

FPGA_IP = "192.168.100.1"
PORTS   = (1234, 1235)
SIZES   = (18, 64, 256, 512, 1024, 1472)     # UDP payload bytes (1472 = 1500-byte MTU)
TIMEOUT = 0.5


def run_port(port, count):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 0))
    sock.settimeout(TIMEOUT)
    ok = bad = lost = 0
    nbytes = 0
    t0 = time.time()
    for i in range(count):
        size = SIZES[i % len(SIZES)]
        payload = i.to_bytes(4, "little") + os.urandom(size - 4)
        sock.sendto(payload, (FPGA_IP, port))
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            lost += 1
            continue
        if data == payload and addr == (FPGA_IP, port):
            ok += 1
            nbytes += len(data)
        else:
            bad += 1
    dt = time.time() - t0
    sock.close()
    return ok, bad, lost, nbytes, dt


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    print(f"UDP loopback  {FPGA_IP}  ports {PORTS}  {count} packets per port  payload sizes {SIZES}")
    all_pass = True
    for port in PORTS:
        ok, bad, lost, nbytes, dt = run_port(port, count)
        status = "PASS" if ok == count else "FAIL"
        all_pass &= ok == count
        print(f"  port {port}: sent {count}  echoed intact {ok}  corrupted {bad}  timeouts {lost}"
              f"  bytes {nbytes}  avg round trip {dt / count * 1e6:.0f} us  -> {status}")
    print("result:", "all passed" if all_pass else "FAILED")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
