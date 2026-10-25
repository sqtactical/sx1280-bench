"""Collect the firmware's per-packet profile (build with -DPROFILE_PACKETS) from both nodes.

  python prof_run.py --presets GFSK_FAST,FLRC_FAST [--n 20] [--size 200]

For each preset: stream test (N broadcasts) and ACK test (N direct messages), each followed by the "PROF" block
both nodes print after 3 s without packets. Results go to prof_results.json.
"""
import argparse, json, os, re, threading, time
import mesh_speed
from mesh_speed import PRESETS, Counter, ack_test, connect, ensure_keys, set_preset, stream_test

import meshtastic.mesh_interface as _mi

_print_log_line = _mi.MeshInterface._printLogLine


def _print_log_line_safe(line, interface):
    # a listener left by a log-capturing interface also fires for interfaces opened without debugOut
    if interface.debugOut is not None:
        _print_log_line(line, interface)


_mi.MeshInterface._printLogLine = staticmethod(_print_log_line_safe)

HERE = os.path.dirname(os.path.abspath(__file__))
PROF_RE = re.compile(r"PROF (stage|thread)=(\S+) n=(\d+) avg_us=(\d+) max_us=(\d+) total_ms=(\d+)")


class LogSink:
    """debugOut target: keeps raw log lines and the latest PROF block"""

    def __init__(self):
        self.lock = threading.Lock()
        self.lines = 0
        self.block, self.blocks = None, []

    def write(self, s):
        with self.lock:
            for line in s.splitlines():
                if "PROF begin" in line:
                    self.block = {"window_ms": int(line.split("window_ms=")[1]), "stage": {}, "thread": {}, "log_lines": self.lines}
                elif "PROF end" in line and self.block is not None:
                    self.blocks.append(self.block)
                    self.block = None
                elif (m := PROF_RE.search(line)) and self.block is not None:
                    kind, name, n, avg, mx, tot = m.groups()
                    self.block[kind][name] = {"n": int(n), "avg_us": int(avg), "max_us": int(mx), "total_ms": int(tot)}
                elif line.strip():
                    self.lines += 1

    def flush(self):
        pass

    def take(self, timeout=15):
        """wait for the next PROF block and return it"""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.blocks:
                    return self.blocks.pop(0)
            time.sleep(0.2)
        return None

    def reset(self):
        with self.lock:
            self.blocks.clear()
            self.lines = 0


def connect_with_log(port, sink, tries=20):
    for _ in range(tries):
        try:
            return mesh_speed.meshtastic.serial_interface.SerialInterface(port, debugOut=sink)
        except Exception:
            time.sleep(2)
    raise RuntimeError(f"cannot connect to {port}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="COM14")
    ap.add_argument("--b", default="COM27")
    ap.add_argument("--presets", default="GFSK_FAST,FLRC_FAST")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--size", type=int, default=200)
    ap.add_argument("--out", default="prof_results.json")
    args = ap.parse_args()

    path = os.path.join(HERE, args.out)
    results = json.load(open(path)) if os.path.exists(path) else {}
    for name in args.presets.split(","):
        if any([set_preset(p, PRESETS[name]) for p in (args.a, args.b)]):
            time.sleep(45)
        sa, sb = LogSink(), LogSink()
        a, b = connect_with_log(args.a, sa), connect_with_log(args.b, sb)
        try:
            bnum = b.myInfo.my_node_num
            counter = Counter(b)
            time.sleep(3)
            ensure_keys(a, b)
            ack_test(a, bnum, 3, args.size, 30)
            time.sleep(4)
            res = {}
            for phase in ("stream", "ack"):
                sa.take(2), sb.take(2)
                sa.reset(), sb.reset()
                if phase == "stream":
                    outcome = stream_test(a, counter, args.n, args.size, settle=5)
                else:
                    lat = [x for x in ack_test(a, bnum, args.n, args.size, 30) if x is not None]
                    outcome = {"acked": len(lat), "lat_median_ms": sorted(lat)[len(lat) // 2] * 1000 if lat else None}
                res[phase] = {"outcome": outcome, "A": sa.take(), "B": sb.take()}
                print(name, phase, json.dumps(outcome), flush=True)
            counter.close()
            results[name] = res
        finally:
            a.close()
            b.close()
        json.dump(results, open(path, "w"), indent=1)


if __name__ == "__main__":
    main()
