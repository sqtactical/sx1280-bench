"""JenCore (MeshCore) group-text test between two Jentastic nodes over LoRa.

Each node sends N texts on the JC: channel (encoded by the firmware as JenCore GRP_TXT floods); the other node must
decrypt them and hand them to its client as text from the JenCore pseudo node 0x4D430001.
  python jc_test.py [--ch 3] [--n 5] [--gap 4]
"""
import argparse, threading, time
from pubsub import pub
from mesh_speed import connect
from jt_speed import Replies, nexus

ap = argparse.ArgumentParser()
ap.add_argument("--ch", type=int, default=3)
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--gap", type=float, default=4.0)
args = ap.parse_args()

got, lock = [], threading.Lock()


def on_text(packet, interface):
    with lock:
        got.append((interface, packet.get("from"), packet.get("channel", 0), packet.get("decoded", {}).get("text", ""),
                    packet.get("rxRssi"), packet.get("rxSnr")))


pub.subscribe(on_text, "meshtastic.receive.text")
replies = Replies()
a, b = connect("COM14"), connect("COM27")
time.sleep(3)
try:
    for name, tx, rx in (("A->B", a, b), ("B->A", b, a)):
        with lock:
            got.clear()
        sent = [f"jc {name} {k} {int(time.time()) % 10000}" for k in range(args.n)]
        for s in sent:
            tx.sendText(s, channelIndex=args.ch)
            time.sleep(args.gap)
        time.sleep(3)
        with lock:
            mine = [g for g in got if g[0] is rx and not g[3].startswith("\U0001f534")]
        hits = [s for s in sent if any(s in g[3] for g in mine)]
        print(f"{name}: delivered {len(hits)}/{len(sent)}")
        for g in mine[:3]:
            print(f"   from=0x{(g[1] or 0):08x} ch={g[2]} rssi={g[4]} snr={g[5]} text={g[3]!r}")
    for name, iface in (("A", a), ("B", b)):
        for t in nexus(iface, replies, "JENCORE STATS", wait=2.5):
            print(name, t.replace("\n", " | "))
finally:
    a.close()
    b.close()
