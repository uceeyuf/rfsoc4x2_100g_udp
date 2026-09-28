"""Receive stream packets and check the ramp test data (ddr_record_loop TEST_RAMP=1).

Every sample is its index in the DDR record, so sample j of a packet must equal
start_index + j. Data left over from before a stop/start shows up as a mismatch.

    python tests/restart_check.py <header_bytes> <packets> [fpga_ip] [port]

Prints READY once registered, then one RESULT line. Used by tests/restart_check.tcl.
"""
import socket
import struct
import sys
import time
from array import array

hdr = int(sys.argv[1])
want = int(sys.argv[2])
fpga = sys.argv[3] if len(sys.argv) > 3 else "192.168.100.1"
port = int(sys.argv[4]) if len(sys.argv) > 4 else 1237

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 << 20)
s.bind(("0.0.0.0", port))
s.settimeout(0.2)
s.sendto(b"REG", (fpga, port))       # the FPGA streams to the sender of the last UDP packet
print("READY", flush=True)

pkts = []
deadline = time.time() + 20
while len(pkts) < want and time.time() < deadline:
    try:
        pkts.append(s.recv(9216))
    except socket.timeout:
        pass

bad = 0
first_bad = ""
for n, d in enumerate(pkts):
    idx, _total = struct.unpack_from("<II", d, 0)
    smp = array("I", d[hdr:])
    if smp.tolist() != list(range(idx, idx + len(smp))):
        bad += 1
        if not first_bad:
            first_bad = " first bad: packet %d start_index %d sample0 %d" % (n, idx, smp[0])

first = struct.unpack_from("<I", pkts[0], 0)[0] if pkts else -1
print("RESULT packets %d first_start_index %d bad %d%s" % (len(pkts), first, bad, first_bad), flush=True)
