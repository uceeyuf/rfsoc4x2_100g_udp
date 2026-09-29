#!/usr/bin/env python3
# Build the README GIF from dpdk_loopback --gif output: the frame being sent (left) and the echoed
# frame as received (right), with the counters at the moment it arrived.
#   python3 host/make_gif.py build/gif_4k360/gif docs/img/video_4k360.gif
import re
import sys
from PIL import Image, ImageDraw, ImageFont

src, dst = sys.argv[1], sys.argv[2]
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 12)
BG, FG, DIM = (30, 30, 30), (225, 225, 225), (150, 150, 150)

lines = open(f"{src}/meta.txt").read().splitlines()
res, fps, cycle, npk = re.match(r"(\S+) (\d+) fps cycle (\d+) packets (\d+)", lines[0]).groups()
fps, cycle = int(fps), int(cycle)
summary = lines[1]
rate = re.search(r"TX ([\d.]+) / RX ([\d.]+) Gbps", summary)
tx_g, rx_g = rate.groups() if rate else ("?", "?")

tx = [Image.open(f"{src}/tx_{c:02d}.ppm").convert("RGB") for c in range(cycle)]
W, H = tx[0].size
M, TOP, BOT = 10, 20, 40
cw, ch = 2 * W + 3 * M, TOP + H + BOT

frames = []
for ln in lines[2:]:
    k, fid, ok, sent, intact, bad, lat = map(int, ln.split())
    if ok != 1:
        continue
    rx = Image.open(f"{src}/rx_{k:03d}.ppm").convert("RGB")
    im = Image.new("RGB", (cw, ch), BG)
    d = ImageDraw.Draw(im)
    s_id = sent - 1
    d.text((M, 4), f"Sent  frame {s_id}  ({res}, {npk} packets)", font=FONT, fill=FG)
    d.text((2 * M + W, 4), f"Received  frame {fid}  intact (byte-exact)", font=FONT, fill=FG)
    im.paste(tx[s_id % cycle], (M, TOP))
    im.paste(rx, (2 * M + W, TOP))
    d.text((M, TOP + H + 6), "running ...  4K through the FPGA UDP echo, DPDK on Linux, every frame compared byte by byte",
           font=FONT, fill=DIM)
    d.text((M, TOP + H + 22),
           f"now: {res} @ {fps} fps   frames {sent} sent  {intact} intact {bad} corrupted   "
           f"TX {tx_g} / RX {rx_g} Gbps   latency {lat / 1000:.1f} ms", font=FONT, fill=FG)
    frames.append(im)

frames[0].save(dst, save_all=True, append_images=frames[1:], duration=100, loop=0, optimize=True)
print(f"{dst}: {len(frames)} frames, {cw}x{ch}")
