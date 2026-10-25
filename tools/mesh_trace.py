"""Capture B's raw serial log (ms timestamps) while A, driven through the API, sends acknowledged DMs to B.
  python mesh_trace.py [--a COM14] [--b COM27] [--n 5] [--size 200] [--broadcast]  -> trace_B.log"""
import argparse, os, threading, time
import serial
import meshtastic.serial_interface as si
import mesh_speed  # noqa: F401  (removes the library's per-write sleep)

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--a", default="COM14")
ap.add_argument("--b", default="COM27")
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--size", type=int, default=200)
ap.add_argument("--broadcast", action="store_true", help="send without ACK as fast as the queue allows")
ap.add_argument("--gap", type=float, default=0.5, help="seconds between acknowledged messages")
args = ap.parse_args()

# B's node number, then release the port for the raw log reader
b = si.SerialInterface(args.b)
bnum = b.myInfo.my_node_num
b.close()
time.sleep(1)

lines, stop = [], threading.Event()


def reader():
    with serial.Serial(args.b, 115200, timeout=0.2) as s:
        while not stop.is_set():
            ln = s.readline()
            if ln:
                lines.append(ln.decode(errors="replace").rstrip())


th = threading.Thread(target=reader, daemon=True)
th.start()
a = si.SerialInterface(args.a)
time.sleep(2)
for i in range(args.n):
    if args.broadcast:
        a.sendData(os.urandom(args.size), portNum=256, wantAck=False)
        time.sleep(args.gap)
        continue
    ev = threading.Event()
    t0 = time.time()
    a.sendData(os.urandom(args.size), destinationId=bnum, portNum=256, wantAck=True,
               onResponse=lambda p: ev.set(), onResponseAckPermitted=True)
    print(i, ev.wait(20), round(time.time() - t0, 3))
    time.sleep(args.gap)
time.sleep(5)
a.close()
stop.set()
th.join()
open(os.path.join(HERE, "trace_B.log"), "w", encoding="utf8").write("\n".join(lines))
print(len(lines), "log lines from B")
