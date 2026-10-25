"""RNode (Reticulum) over BLE Nordic UART on Jentastic nodes.

  python rnode_ble.py scan
  python rnode_ble.py probe <addr>                       KISS detect / version / platform / radio params
  python rnode_ble.py link <addrA> <addrB> [--n 5]       A sends KISS DATA frames over LoRa, B's client must receive them
  python rnode_ble.py unpair <addr> [<addr> ...]         remove the Windows pairing again
"""
import asyncio, os, sys
from bleak import BleakClient, BleakScanner

NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write: host -> node
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: node -> host
FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
NAMES = {0x08: "DETECT", 0x50: "FW_VERSION", 0x48: "PLATFORM", 0x49: "MCU", 0x01: "FREQ", 0x02: "BW", 0x04: "SF",
         0x05: "CR", 0x03: "TXP", 0x00: "DATA", 0x23: "STAT_RSSI", 0x24: "STAT_SNR", 0x0F: "READY"}


def kiss(cmd, payload=b""):
    body = bytearray()
    for b in payload:
        body += bytes([FESC, TFEND]) if b == FEND else bytes([FESC, TFESC]) if b == FESC else bytes([b])
    return bytes([FEND, cmd]) + bytes(body) + bytes([FEND])


class Deframer:
    def __init__(self):
        self.buf, self.frames = bytearray(), []

    def feed(self, data):
        for b in data:
            if b == FEND:
                if self.buf:
                    raw, out, esc = bytes(self.buf), bytearray(), False
                    for x in raw[1:]:
                        if esc:
                            out.append(FEND if x == TFEND else FESC if x == TFESC else x)
                            esc = False
                        elif x == FESC:
                            esc = True
                        else:
                            out.append(x)
                    self.frames.append((raw[0], bytes(out)))
                self.buf = bytearray()
            else:
                self.buf.append(b)


async def scan():
    for d, adv in (await BleakScanner.discover(timeout=8, return_adv=True)).values():
        uuids = [u.lower() for u in adv.service_uuids]
        # Meshtastic nodes advertise their service UUID, whatever short name the owner gave them
        if NUS in uuids or "6ba1b218-15a8-461f-9fa8-5dcae273eafd" in uuids or (d.name or "").startswith("Meshtastic"):
            print(d.address, repr(d.name), "NUS" if NUS in uuids else "", adv.rssi)


async def open_client(addr):
    c = BleakClient(addr, timeout=20, winrt=dict(use_cached_services=False))
    await c.connect()
    # The NUS characteristics need an encrypted link; with the node in NO_PIN mode this is a Just Works pairing
    await c.pair()
    d = Deframer()
    await c.start_notify(NUS_TX, lambda _, data: d.feed(data))
    return c, d


async def probe(addr):
    c, d = await open_client(addr)
    try:
        for cmd, payload in ((0x08, b"\x73"), (0x50, b""), (0x48, b""), (0x49, b""), (0x01, b""), (0x02, b""), (0x04, b""),
                             (0x05, b""), (0x03, b"")):
            await c.write_gatt_char(NUS_RX, kiss(cmd, payload), response=False)
            await asyncio.sleep(0.4)
        for cmd, data in d.frames:
            print(f"  {NAMES.get(cmd, hex(cmd))}: {data.hex()}")
    finally:
        await c.disconnect()


async def link(addr_a, addr_b, n):
    ca, da = await open_client(addr_a)
    cb, db = await open_client(addr_b)
    try:
        sent = []
        for k in range(n):
            # Reticulum-shaped payload: flags (broadcast), hops 0, then a 16-byte destination hash and data
            pkt = bytes([0x00, 0x00]) + os.urandom(16) + f"rnode test {k}".encode()
            sent.append(pkt)
            await ca.write_gatt_char(NUS_RX, kiss(0x00, pkt), response=False)
            await asyncio.sleep(2.0)
        await asyncio.sleep(2)
        rx = [data for cmd, data in db.frames if cmd == 0x00]
        hits = sum(1 for p in sent if p in rx)
        print(f"A->B over LoRa: {hits}/{n} DATA frames delivered to B's RNode client")
        for cmd, data in db.frames[:6]:
            print(f"  B got {NAMES.get(cmd, hex(cmd))}: {data[:24].hex()}{'...' if len(data) > 24 else ''}")
    finally:
        await ca.disconnect()
        await cb.disconnect()


async def unpair(addr):
    c = BleakClient(addr, timeout=20)
    await c.connect()
    await c.unpair()
    print("unpaired", addr)


if __name__ == "__main__":
    if sys.argv[1] == "unpair":
        for a in sys.argv[2:]:
            asyncio.run(unpair(a))
    elif sys.argv[1] == "scan":
        asyncio.run(scan())
    elif sys.argv[1] == "probe":
        asyncio.run(probe(sys.argv[2]))
    elif sys.argv[1] == "link":
        n = int(sys.argv[sys.argv.index("--n") + 1]) if "--n" in sys.argv else 5
        asyncio.run(link(sys.argv[2], sys.argv[3], n))
