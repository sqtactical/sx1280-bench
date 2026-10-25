"""How long a receiver needs per broadcast: send N broadcasts at a fixed spacing and count what B delivers.

  python jt_gap.py --mode "FSK 1000" --size 80 --gaps 0.05,0.1,0.15,0.2,0.3 [--n 10]
"""
import argparse, os, time
from jt_speed import Replies, set_mode
from mesh_speed import Counter, connect

ap = argparse.ArgumentParser()
ap.add_argument("--mode", default="FSK 1000")
ap.add_argument("--size", type=int, default=80)
ap.add_argument("--gaps", default="0.05,0.1,0.15,0.2,0.3")
ap.add_argument("--n", type=int, default=10)
args = ap.parse_args()

replies = Replies()
a, b = connect("COM14"), connect("COM27")
try:
    counter = Counter(b)
    time.sleep(2)
    set_mode(a, b, replies, args.mode)
    for gap in (float(g) for g in args.gaps.split(",")):
        counter.rx.clear()
        t0 = time.time()
        for _ in range(args.n):
            a.sendData(os.urandom(args.size), portNum=256, wantAck=False)
            time.sleep(gap)
        sent_s = time.time() - t0
        time.sleep(4)
        print(f"{args.mode} {args.size}B gap {gap * 1000:4.0f} ms: received {len(counter.rx):2}/{args.n} (sent in {sent_s:.2f} s)",
              flush=True)
    set_mode(a, b, replies, "LORA")
finally:
    counter.close()
    a.close()
    b.close()
