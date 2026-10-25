"""Packet loss vs payload length for FLRC modes (raw bursts, no ARQ).

  python len_sweep.py [--tx COM33] [--rx COM34] [--n 300]
Writes results_len_sweep.json.
"""
import argparse, json, os, time
import serial
from flrc_host import cmd, cmd_wait, kv, stop_rx

HERE = os.path.dirname(os.path.abspath(__file__))
MODES = [("1300k CR1/2", 0x45, 0x00), ("1300k CR3/4", 0x45, 0x02), ("1300k CR1/1", 0x45, 0x04),
         ("325k CR3/4", 0xC7, 0x02), ("1040k CR3/4", 0x69, 0x02)]
LENS = list(range(6, 33)) + [40, 48, 64, 96, 127]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tx", default="COM33")
    ap.add_argument("--rx", default="COM34")
    ap.add_argument("--n", type=int, default=300)
    a = ap.parse_args()
    tx = serial.Serial(a.tx, 115200, timeout=2)
    rx = serial.Serial(a.rx, 115200, timeout=2)
    time.sleep(0.3)
    for s in (tx, rx):
        s.write(b"\n"); time.sleep(0.05); s.reset_input_buffer()
        cmd(s, "OPT 1 1 0x80 64", "OK")
    res = {}
    for name, br, cr in MODES:
        for s in (tx, rx):
            cmd(s, f"CFG {br:#x} {cr:#x} 0x20 0 0x30 3 127", "OK")
        row = {}
        for ln in LENS:
            rx.reset_input_buffer()
            rx.write(b"RX\n"); time.sleep(0.1)
            cmd(tx, f"BURST {a.n} {ln}", "BURSTDONE", 20)
            cmd_wait(tx, "PROF", 2)
            time.sleep(0.05)
            st = kv(stop_rx(rx))
            row[ln] = {"rx": int(st["raw"]), "crc": int(st["crc"]), "bad": int(st["rawbad"])}
        res[name] = row
        bad = [ln for ln, r in row.items() if r["rx"] < a.n * 0.95]
        print(f"{name:>12}: " + " ".join(f"{ln}:{100 * (a.n - r['rx']) / a.n:.0f}%" for ln, r in row.items()), flush=True)
        print(f"{'':>12}  lengths with >5% loss: {bad}", flush=True)
    with open(os.path.join(HERE, "results_len_sweep.json"), "w") as fh:
        json.dump({"n": a.n, "results": res}, fh, indent=1)


if __name__ == "__main__":
    main()
