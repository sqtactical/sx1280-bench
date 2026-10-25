"""prof_run.py for Jentastic: profile each Nexus alt-modem mode (firmware built with -DPROFILE_PACKETS).

  python prof_jt.py --modes "FSK 1000;FLRC 650 2;LORA" [--n 20] [--size 200]  -> prof_jt_results.json
"""
import argparse, json, os, time
from mesh_speed import Counter, ack_test, ensure_keys, stream_test
from jt_speed import Replies, set_mode
from prof_run import LogSink, connect_with_log

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument("--modes", default="FSK 1000;FLRC 650 2;LORA")
ap.add_argument("--n", type=int, default=20)
ap.add_argument("--size", type=int, default=200)
ap.add_argument("--out", default="prof_jt_results.json")
args = ap.parse_args()

path = os.path.join(HERE, args.out)
results = {}
replies = Replies()
sa, sb = LogSink(), LogSink()
a, b = connect_with_log("COM14", sa), connect_with_log("COM27", sb)
try:
    bnum = b.myInfo.my_node_num
    counter = Counter(b)
    time.sleep(3)
    ensure_keys(a, b)
    for mode in args.modes.split(";"):
        set_mode(a, b, replies, mode)
        ack_test(a, bnum, 3, args.size, 15)
        time.sleep(4)
        res = {}
        for phase in ("stream", "ack"):
            sa.take(2), sb.take(2)
            sa.reset(), sb.reset()
            if phase == "stream":
                outcome = stream_test(a, counter, args.n, args.size, settle=5)
            else:
                lat = [x for x in ack_test(a, bnum, args.n, args.size, 15) if x is not None]
                outcome = {"acked": len(lat), "lat_median_ms": sorted(lat)[len(lat) // 2] * 1000 if lat else None}
            res[phase] = {"outcome": outcome, "A": sa.take(), "B": sb.take()}
            print(mode, phase, json.dumps(outcome), flush=True)
        results[mode] = res
        json.dump(results, open(path, "w"), indent=1)
    set_mode(a, b, replies, "LORA")
finally:
    counter.close()
    a.close()
    b.close()
