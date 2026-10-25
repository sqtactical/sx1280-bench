"""Capture both nodes' logs while the Jentastic alt modem is active.

  python jt_trace.py --mode "FLRC 650 4" [--n 3] [--size 80]  -> jt_trace_A.log, jt_trace_B.log
"""
import argparse, os, time
import prof_run  # noqa: F401  (log-line listener fix)
from jt_speed import Replies, set_mode
from prof_run import connect_with_log

HERE = os.path.dirname(os.path.abspath(__file__))


class FileSink:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf8")

    def write(self, s):
        self.f.write(s)

    def flush(self):
        self.f.flush()


ap = argparse.ArgumentParser()
ap.add_argument("--mode", default="FLRC 650 4")
ap.add_argument("--n", type=int, default=3)
ap.add_argument("--size", type=int, default=80)
ap.add_argument("--gap", type=float, default=1.5)
args = ap.parse_args()

sa, sb = FileSink(os.path.join(HERE, "jt_trace_A.log")), FileSink(os.path.join(HERE, "jt_trace_B.log"))
replies = Replies()
a, b = connect_with_log("COM14", sa), connect_with_log("COM27", sb)
try:
    print(set_mode(a, b, replies, args.mode))
    for i in range(args.n):
        a.sendData(os.urandom(args.size), portNum=256, wantAck=False)
        time.sleep(args.gap)
    time.sleep(3)
    print(set_mode(a, b, replies, "LORA"))
finally:
    a.close()
    b.close()
    sa.f.close()
    sb.f.close()
