// SX1280 file-transfer modem (FLRC / LoRa / GFSK) for "Faketec" (ProMicro nRF52840 + Ebyte E28-2G4M27S).
// Build: arduino-cli, board adafruit:nrf52:pca10056 (identity pin map: P0.x = x, P1.x = 32+x).
//
// USB-CDC text protocol (one command per line):
//   PING                         -> PONG ...
//   CFG <p1> <p2> <p3> <pwr> <pre> [modem] [pl]   (hex/dec) -> OK ...
//        modem 3 = FLRC (p1 bitrate/bw, p2 CR, p3 BT), 1 = LoRa (p1 SF, p2 BW, p3 CR, pre = preamble byte),
//        0 = GFSK (p1 bitrate/bw, p2 modulation index, p3 BT); pl = max payload bytes (FLRC <= 127, others <= 255)
//   RX                           -> receiver mode until any byte arrives from host, then "RXSTAT ..."
//                                   a file transfer is output as "START <n>\n" + <n> raw bytes + "END ...\n"
//   TX <n>                       -> followed by <n> raw bytes; sent with block ARQ, then "DONE ..." / "FAIL ..."
//   RELAY <n>                    -> followed by <n> raw bytes. Mesh-style test against a board in RX: per packet TX,
//                                   hop ACK, the other node forwards it, ACK. The forwarded copy is compared with
//                                   what was sent -> "DONE ... mismatch=<packets that differed>"
//   BURST <n>                    -> send <n> raw full-size packets back to back (no ARQ) -> "BURSTDONE ..."
//   OPT <pipe> <sync> <ramp>     -> tuning switches (see opt* below)
//   DFU                          -> reboot into the UF2 bootloader (a NICENANO drive appears)
#include <Arduino.h>
#include <SPI.h>
#include <initializer_list>

static const uint8_t P_NRST = 9, P_DIO1 = 10, P_SCK = 43, P_CS = 45, P_MOSI = 47, P_MISO = 2;
static const uint8_t P_BUSY = 29, P_TXEN = 38, P_RXEN = 17, P_VEXT = 13;

SPIClass rspi(NRF_SPIM3, P_MISO, P_SCK, P_MOSI);

// ---------------------------------------------------------------- timing (DWT cycle counter, 64 MHz)
static uint32_t lastCyc;
static uint64_t cycHi;
static uint64_t nowUs() {
  uint32_t c = DWT->CYCCNT;
  if (c < lastCyc) cycHi += (1ULL << 32);
  lastCyc = c;
  return (cycHi + c) / 64;
}
static inline uint32_t us32() { return (uint32_t)nowUs(); }

// Hardware watchdog: if the firmware ever wedges (USB stuck, fault) the board resets itself after 30 s.
// Every wait loop feeds it; the longest legitimate gap is one LoRa SF12 packet (~6 s).
static inline void wdtFeed() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }
static void wdtStart() {
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
  NRF_WDT->CRV = 30 * 32768;
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

// ---------------------------------------------------------------- SX1280 low level
enum : uint16_t { IRQ_TXDONE = 0x0001, IRQ_RXDONE = 0x0002, IRQ_SYNCVALID = 0x0004, IRQ_SYNCERR = 0x0008,
                  IRQ_HDRVALID = 0x0010, IRQ_HDRERR = 0x0020, IRQ_CRCERR = 0x0040, IRQ_TIMEOUT = 0x4000 };
enum : uint8_t { M_GFSK = 0, M_LORA = 1, M_FLRC = 3 };
static const uint8_t RXBASE = 0x00;
// FLRC needs >= 6 payload bytes: shorter packets are padded, flagged in byte 0, pad count in the last byte
#define MINPL 6
#define F_PAD 0x40

static uint8_t sbuf[272], rbuf[272];
static uint32_t busyErr, radioResets;
static bool inRecovery;
static void radioReset();
static void radioConfig();

// BUSY normally drops within a few hundred us. If it stays high the SX1280 is wedged (seen after long
// TX/RX alternation): hard-reset and reconfigure it, and drop the pending command (the ARQ retries).
static bool waitBusy() {
  uint32_t t0 = us32();
  wdtFeed();
  while (digitalRead(P_BUSY)) {
    if (us32() - t0 > 10000) {
      busyErr++;
      if (!inRecovery) {
        inRecovery = true;
        radioReset();
        radioConfig();
        radioResets++;
        inRecovery = false;
      }
      return false;
    }
  }
  return true;
}
static void xfer(uint16_t n) {
  if (!waitBusy() && !inRecovery) { memset(rbuf, 0, n); return; }
  digitalWrite(P_CS, LOW);
  rspi.transfer(sbuf, rbuf, n);
  digitalWrite(P_CS, HIGH);
}
static void cmdv(std::initializer_list<uint8_t> l) {
  uint16_t n = 0;
  for (uint8_t b : l) sbuf[n++] = b;
  xfer(n);
}
static uint16_t getIrq() { cmdv({0x15, 0, 0, 0}); return (rbuf[2] << 8) | rbuf[3]; }
static void clrIrq(uint16_t m) { cmdv({0x97, (uint8_t)(m >> 8), (uint8_t)m}); }
static void writeReg(uint16_t a, const uint8_t *d, uint8_t n) {
  sbuf[0] = 0x18; sbuf[1] = a >> 8; sbuf[2] = a;
  memcpy(sbuf + 3, d, n);
  xfer(3 + n);
}
static void writeReg1(uint16_t a, uint8_t v) { writeReg(a, &v, 1); }
static uint8_t readReg(uint16_t a) { cmdv({0x19, (uint8_t)(a >> 8), (uint8_t)a, 0, 0}); return rbuf[4]; }

// ---------------------------------------------------------------- radio config / state
struct Cfg { uint8_t modem, p1, p2, p3, pwr, pre, pl; } cfg = {M_FLRC, 0x45, 0x04, 0x20, 0, 0x30, 127};
static uint8_t dpp = 123;       // data bytes per packet = pl - 4 header bytes
static uint8_t curLen = 0, curTxBase = 0;
static bool optPipe = true;     // preload the next packet into the other half of the SX1280 buffer while on air
static bool optSync = true;     // sync word on (off only makes sense for experiments: noise floods the receiver)
static uint8_t optRamp = 0x80;  // PA ramp time code (0x00 = 2us ... 0xE0 = 20us)
static uint8_t optWin = 64;     // packets per ACK (1 = stop-and-wait, every packet acknowledged)
static bool inRx = false;
static int8_t lastRssi = 0;

// Two TX slots in the 256-byte SX1280 buffer when a packet fits in half of it. Slot 1 shares the RX area,
// which is fine because the sender never listens while a packet is preloaded there.
static inline bool canPipe() { return optPipe && cfg.pl <= 127; }
static inline uint8_t slotBase(uint8_t s) { return cfg.pl > 127 ? 0x00 : (s ? 0x00 : 0x80); }

static uint32_t flrcBps(uint8_t br) {
  switch (br) {
    case 0x45: return 1300000; case 0x69: return 1040000; case 0x86: return 650000;
    case 0xAA: return 520000;  case 0xC7: return 325000;  default:   return 260000;
  }
}
static uint32_t gfskBps(uint8_t br) {
  switch (br) {
    case 0x04: return 2000000; case 0x28: return 1600000; case 0x4C: case 0x45: return 1000000;
    case 0x70: case 0x69: return 800000; case 0x8D: case 0x86: return 500000;
    case 0xB1: case 0xAA: return 400000; case 0xCE: case 0xC7: return 250000; default: return 125000;
  }
}
static uint32_t loraBwHz(uint8_t bw) {
  switch (bw) { case 0x0A: return 1625000; case 0x18: return 812500; case 0x26: return 406250; default: return 203125; }
}

// Time on air in us for one variable-length packet with <len> payload bytes (datasheet 7.1, 7.3.4, 7.4.4)
static uint32_t toaUs(uint8_t len) {
  if (cfg.modem == M_FLRC) {
    uint32_t preBits = ((cfg.pre >> 4) + 1) * 4;
    uint32_t unc = preBits + 21 + (optSync ? 32 : 0) + 16;
    uint32_t raw = (len + 2) * 8 + (cfg.p2 == 0x04 ? 0 : 6);
    uint32_t cod = cfg.p2 == 0x00 ? raw * 2 : cfg.p2 == 0x02 ? (raw * 4 + 2) / 3 : raw;
    return (uint32_t)((uint64_t)(unc + cod) * 1000000ULL / flrcBps(cfg.p1));
  }
  if (cfg.modem == M_GFSK) {
    uint32_t bits = ((cfg.pre >> 4) + 1) * 4 + (optSync ? 32 : 0) + 9 + len * 8 + 16;
    return (uint32_t)((uint64_t)bits * 1000000ULL / gfskBps(cfg.p1));
  }
  // LoRa, explicit header, CRC on, legacy coding rate
  int sf = cfg.p1 >> 4, cr = cfg.p3 & 0x07;
  float c1 = sf < 7 ? 6.25f : 4.25f;
  int c2 = sf < 7 ? 4 * sf : 4 * sf + 8, c3 = sf > 10 ? 4 * (sf - 2) : 4 * sf;
  int npre = (cfg.pre & 0x0F) << (cfg.pre >> 4);
  int num = 8 * len + 16 - c2 + 20;
  float nsym = npre + c1 + 8 + (num > 0 ? (num + c3 - 1) / c3 : 0) * (cr + 4);
  return (uint32_t)(nsym * (float)(1UL << sf) * 1e6f / loraBwHz(cfg.p2));
}

static void setLen(uint8_t n) {
  if (n == curLen) return;
  if (cfg.modem == M_FLRC)        // preamble, 32-bit sync, match sync1, variable length, length, CRC 2 B, no whitening
    cmdv({0x8C, cfg.pre, (uint8_t)(optSync ? 0x04 : 0x00), (uint8_t)(optSync ? 0x10 : 0x00), 0x20, n, 0x10, 0x08});
  else if (cfg.modem == M_GFSK)   // preamble, 4-byte sync, match sync1, variable length, length, CRC 2 B, whitening on
    cmdv({0x8C, cfg.pre, (uint8_t)(optSync ? 0x06 : 0x00), (uint8_t)(optSync ? 0x10 : 0x00), 0x20, n, 0x20, 0x00});
  else                            // preamble, explicit header, length, CRC on, standard IQ
    cmdv({0x8C, cfg.pre, 0x00, n, 0x20, 0x40, 0x00, 0x00});
  curLen = n;
}

static void radioReset() {
  digitalWrite(P_NRST, LOW); delay(3);
  digitalWrite(P_NRST, HIGH); delay(20);
  waitBusy();
}

static void radioConfig() {
  digitalWrite(P_TXEN, LOW); digitalWrite(P_RXEN, LOW);
  inRx = false;
  dpp = cfg.pl - 4;
  cmdv({0x80, 0x00});                                  // SetStandby(STDBY_RC)
  cmdv({0x8A, cfg.modem});                             // SetPacketType
  uint32_t f = (uint32_t)((2440000000ULL << 18) / 52000000ULL);  // 2440 MHz
  cmdv({0x86, (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f});
  cmdv({0x8F, slotBase(0), RXBASE});                   // SetBufferBaseAddress
  curTxBase = slotBase(0);
  cmdv({0x8B, cfg.p1, cfg.p2, cfg.p3});                // SetModulationParams
  if (cfg.modem == M_LORA) {
    int sf = cfg.p1 >> 4;
    writeReg1(0x0925, sf < 7 ? 0x1E : sf < 9 ? 0x37 : 0x32);  // required after SetModulationParams (13.4.1)
    writeReg1(0x093C, 0x01);                           // frequency error compensation
  }
  curLen = 0; setLen(cfg.pl);
  static const uint8_t sync[4] = {0x2D, 0x4B, 0xD4, 0x93};
  if (cfg.modem != M_LORA) writeReg(0x09CF, sync, 4);  // SyncWord1 (LoRa keeps its default 0x12 sync word)
  cmdv({0x8E, cfg.pwr, optRamp});                      // SetTxParams(power, ramp)
  // The guard IRQ is latched but not routed to DIO1: pollRx uses it to spot the next packet overwriting the buffer
  uint16_t m = IRQ_TXDONE | IRQ_RXDONE | IRQ_CRCERR | IRQ_TIMEOUT;
  uint16_t mi = m | (cfg.modem == M_LORA ? IRQ_HDRVALID : IRQ_SYNCVALID);
  cmdv({0x8D, (uint8_t)(mi >> 8), (uint8_t)mi, (uint8_t)(m >> 8), (uint8_t)m, 0, 0, 0, 0});
  cmdv({0x9E, 0x01});                                  // SetAutoFs(on): stay in FS after TX/RX
  clrIrq(0xFFFF);
}

static void stopRx() {
  if (!inRx) return;
  cmdv({0xC1});                                        // SetFs
  digitalWrite(P_RXEN, LOW);
  clrIrq(0xFFFF);
  inRx = false;
}

static void startRx() {
  if (inRx) return;
  setLen(cfg.pl);
  digitalWrite(P_TXEN, LOW); digitalWrite(P_RXEN, HIGH);
  clrIrq(0xFFFF);
  cmdv({0x82, 0x02, 0xFF, 0xFF});                      // SetRx continuous
  inRx = true;
}

static void pumpIn();
static uint32_t txCount, txFail;

static uint8_t slotLen[2];
static uint32_t tFire;
// profiling accumulators (us): SPI payload load, SetTx issue, SetTx->TxDone, TxDone handling
static uint32_t pLoad, pFire, pAir, pPost, pN;

static void txLoad(uint8_t slot, const uint8_t *d, uint8_t n) {
  uint32_t t = us32();
  sbuf[0] = 0x1A; sbuf[1] = slotBase(slot);
  memcpy(sbuf + 2, d, n);
  if (n < MINPL) {
    memset(sbuf + 2 + n, 0, MINPL - n);
    sbuf[2] |= F_PAD;
    sbuf[2 + MINPL - 1] = MINPL - n;
    n = MINPL;
  }
  xfer(2 + n);
  slotLen[slot] = n;
  pLoad += us32() - t;
}

static void txFire(uint8_t slot) {
  uint32_t t = us32();
  stopRx();
  if (curTxBase != slotBase(slot)) { cmdv({0x8F, slotBase(slot), RXBASE}); curTxBase = slotBase(slot); }
  setLen(slotLen[slot]);
  digitalWrite(P_RXEN, LOW); digitalWrite(P_TXEN, HIGH);
  cmdv({0x83, 0x02, 0x00, 0x00});                      // SetTx, no timeout
  tFire = us32();
  pFire += tFire - t;
}

// keepPa: another packet follows immediately, leave the PA enabled
static bool txWait(uint8_t slot, bool pump, bool keepPa) {
  uint32_t toa = toaUs(slotLen[slot]), lim = toa + toa / 4 + 5000;   // generous: a timeout aborts the packet
  bool ok = true;
  while (!digitalRead(P_DIO1)) {
    wdtFeed();
    if (pump) pumpIn();
    if (us32() - tFire > lim) { ok = false; break; }
  }
  uint32_t t = us32();
  pAir += t - tFire;
  if (!keepPa || !ok) digitalWrite(P_TXEN, LOW);
  clrIrq(0xFFFF);
  pPost += us32() - t;
  pN++;
  txCount++;
  if (!ok) { txFail++; cmdv({0xC1}); }
  return ok;
}

static bool radioTx(const uint8_t *d, uint8_t n, bool pump) {
  txLoad(0, d, n);
  txFire(0);
  return txWait(0, pump, false);
}

// Sends cnt packets back to back. build(k) fills pk for packet k and returns its length; with pipelining the
// next packet is written into the other buffer slot while the current one is on air.
static uint8_t pk[260];
static void txSeries(uint32_t cnt, bool pump, uint8_t (*build)(uint32_t k)) {
  bool pipe = canPipe();
  uint8_t s = 0;
  txLoad(s, pk, build(0));
  for (uint32_t k = 0; k < cnt; k++) {
    bool more = k + 1 < cnt;
    txFire(s);
    if (pipe && more) txLoad(s ^ 1, pk, build(k + 1));
    txWait(s, pump, pipe && more);
    if (!pipe && more) txLoad(s, pk, build(k + 1));
    if (pipe) s ^= 1;
  }
}

static void profReset() { pLoad = pFire = pAir = pPost = pN = 0; }
static void profPrint(uint32_t toa) {
  if (!pN) return;
  Serial.printf("PROF n=%lu load=%lu fire=%lu air=%lu (toa=%lu startup=%ld) post=%lu us/pkt\n", pN, pLoad / pN,
                pFire / pN, pAir / pN, toa, (int32_t)(pAir / pN) - (int32_t)toa, pPost / pN);
}

static uint32_t rxOk, rxCrc, rxOverrun;
// returns payload length (>0), 0 = nothing, -1 = bad packet
static int pollRx(uint8_t *out, bool wantRssi) {
  if (!digitalRead(P_DIO1)) return 0;
  uint16_t irq = getIrq();
  clrIrq(0xFFFF);                                      // also clears this packet's guard IRQ
  if (!(irq & IRQ_RXDONE)) return -1;
  if (irq & IRQ_CRCERR) { rxCrc++; return -1; }
  cmdv({0x17, 0, 0, 0});                               // GetRxBufferStatus
  uint8_t len = rbuf[2], off = rbuf[3];
  if (len == 0 || len > cfg.pl) return -1;
  sbuf[0] = 0x1B; sbuf[1] = off; sbuf[2] = 0;
  memset(sbuf + 3, 0, len);
  xfer(3 + len);
  memcpy(out, rbuf + 3, len);
  // In continuous RX the next packet is written over the same buffer. If its sync word / header was already
  // detected while we were reading, the bytes may be mixed: drop it and let the ARQ resend.
  if (getIrq() & (IRQ_SYNCVALID | IRQ_HDRVALID)) { rxOverrun++; return -1; }
  if (out[0] & F_PAD) {
    if (out[len - 1] >= len) return -1;
    len -= out[len - 1];
    out[0] &= ~F_PAD;
  }
  if (wantRssi) {
    cmdv({0x1D, 0, 0, 0, 0, 0, 0});
    lastRssi = -(int)rbuf[cfg.modem == M_LORA ? 2 : 3] / 2;
  }
  rxOk++;
  return len;
}

// ---------------------------------------------------------------- link protocol
enum : uint8_t { T_DATA = 0x01, T_POLL = 0x02, T_ACK = 0x03, T_START = 0x04, T_STARTACK = 0x05,
                 T_RDATA = 0x06, T_RFWD = 0x07, T_RAW = 0x10, F_POLL = 0x80 };
#define MAXDPP 251              // 255-byte payload - 4 header bytes
#define MAXPK 64                // max packets per ARQ block (ACK bitmap width)
#define BLKMAX (MAXDPP * MAXPK)
static inline uint32_t blkBytes() { return (uint32_t)dpp * optWin; }
static inline uint64_t fullMask(uint8_t n) { return n >= 64 ? ~0ULL : ((1ULL << n) - 1); }

// --- USB input prefetch (TX side): two block buffers, the inactive one is filled while the radio works
static uint8_t blkbuf[2][BLKMAX];
static uint32_t fillN[2], wantN[2];
static int curB;
static void pumpIn() {
  wdtFeed();
  int nb = curB ^ 1;
  if (fillN[nb] >= wantN[nb]) return;
  int a = Serial.available();
  if (a <= 0) return;
  uint32_t k = min((uint32_t)a, wantN[nb] - fillN[nb]);
  fillN[nb] += Serial.read(blkbuf[nb] + fillN[nb], k);
}

// --- USB output ring (RX side)
#define RING 32768
static uint8_t ring[RING];
static uint32_t rHead, rTail, rCount;
// moves at most one CDC FIFO worth per call so the RX loop gets back to the radio quickly
static void pumpOut() {
  wdtFeed();
  if (!rCount) return;
  int w = Serial.availableForWrite();
  if (w <= 0) return;
  uint32_t c = min(rCount, (uint32_t)(RING - rTail));
  c = min(c, (uint32_t)w);
  size_t done = Serial.write(ring + rTail, c);
  rTail = (rTail + done) % RING; rCount -= done;
  Serial.flush();
}
static void outPush(const uint8_t *d, uint32_t n) {
  while (n) {
    while (rCount == RING) pumpOut();
    uint32_t c = min(n, (uint32_t)(RING - rHead));
    c = min(c, RING - rCount);
    memcpy(ring + rHead, d, c);
    rHead = (rHead + c) % RING; rCount += c; d += c; n -= c;
  }
}
static void outPrintf(const char *fmt, ...) {
  char b[200];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  outPush((const uint8_t *)b, n);
}

// ---------------------------------------------------------------- TX side
static bool waitReply(uint8_t type, uint8_t key, uint32_t tmo, bool pump) {
  startRx();
  uint32_t t0 = us32();
  while (us32() - t0 < tmo) {
    wdtFeed();
    if (pump) pumpIn();
    int n = pollRx(pk, false);
    if (n >= 10 && pk[0] == type && pk[1] == key) { stopRx(); return true; }
  }
  stopRx();
  return false;
}

// after a failed TX/RELAY, swallow the rest of the file so it is not parsed as commands
static void drainInput() {
  uint8_t b[64];
  uint32_t t = millis();
  while (millis() - t < 1000) {
    wdtFeed();
    if (Serial.available()) { Serial.read(b, sizeof b); t = millis(); }
  }
}

static bool fillBlocking(int b, uint32_t tmoMs) {
  uint32_t t0 = millis();
  while (fillN[b] < wantN[b]) {
    wdtFeed();
    int a = Serial.available();
    if (a > 0) { uint32_t k = min((uint32_t)a, wantN[b] - fillN[b]); fillN[b] += Serial.read(blkbuf[b] + fillN[b], k); t0 = millis(); }
    else if (millis() - t0 > tmoMs) return false;
  }
  return true;
}

// state of the round being sent, read by buildData()
static uint8_t sIdx[MAXPK], sCnt, sBlk, sNpk;
static uint32_t sN;
static uint8_t buildData(uint32_t k) {
  uint8_t i = sIdx[k];
  uint32_t off = i * dpp, len = min((uint32_t)dpp, sN - off);
  pk[0] = T_DATA | (k == sCnt - 1u ? F_POLL : 0); pk[1] = sBlk; pk[2] = i; pk[3] = sNpk;
  memcpy(pk + 4, blkbuf[curB] + off, len);
  return 4 + len;
}
static uint8_t burstLen;
static uint8_t buildRaw(uint32_t i) {
  pk[0] = T_RAW; pk[1] = i; pk[2] = i >> 8; pk[3] = i >> 16;
  return burstLen;
}

static void doTx(uint32_t total) {
  uint32_t ackTmo = 1500 + 3 * toaUs(12);
  uint32_t pkts = 0, retx = 0, polls = 0, tmos = 0;
  uint8_t sess = (uint8_t)(us32() ^ (us32() >> 8)) | 1;
  txFail = 0;

  // prefetch first two blocks
  curB = 0;
  uint32_t queued = 0;
  wantN[0] = min(blkBytes(), total); fillN[0] = 0; queued += wantN[0];
  wantN[1] = min(blkBytes(), total - queued); fillN[1] = 0; queued += wantN[1];
  if (!fillBlocking(0, 5000)) { Serial.printf("FAIL usb-timeout\n"); return; }

  uint64_t t0 = nowUs();
  // START handshake
  bool ok = false;
  uint32_t tHs = millis();
  for (int i = 0; i < 2000 && !ok && millis() - tHs < 20000; i++) {
    pk[0] = T_START; pk[1] = sess; memcpy(pk + 2, &total, 4); pk[6] = dpp; pk[7] = optWin; pk[8] = pk[9] = 0;
    radioTx(pk, 10, true);
    ok = waitReply(T_STARTACK, sess, ackTmo, true);
    if (!ok) tmos++;
  }
  if (!ok) { drainInput(); Serial.printf("FAIL no-startack tmos=%lu busyErr=%lu\n", tmos, busyErr); return; }

  uint64_t tStart = nowUs() - t0, tData = 0, tAck = 0, tFill = 0;
  profReset();
  uint32_t sent = 0;
  uint8_t blk = 0;
  while (sent < total) {
    uint64_t tf = nowUs();
    if (!fillBlocking(curB, 5000)) { Serial.printf("FAIL usb-timeout sent=%lu\n", sent); return; }
    tFill += nowUs() - tf;
    uint32_t n = wantN[curB];
    uint8_t npk = (n + dpp - 1) / dpp;
    uint64_t full = fullMask(npk), acked = 0;
    bool first = true;
    uint32_t idle = 0;
    while (acked != full) {
      // (re)send all unacked packets back to back, flag the last one as poll
      sCnt = 0;
      for (int i = 0; i < npk; i++) if (!(acked >> i & 1)) sIdx[sCnt++] = i;
      sBlk = blk; sNpk = npk; sN = n;
      uint64_t td = nowUs();
      txSeries(sCnt, true, buildData);
      pkts += sCnt; if (!first) retx += sCnt;
      tData += nowUs() - td;
      first = false;
      uint64_t ta = nowUs();
      bool got = waitReply(T_ACK, blk, ackTmo, true);
      uint32_t tLost = millis();
      while (!got) {
        tmos++;
        if (++idle > 3000 || millis() - tLost > 20000) {
          drainInput();
          Serial.printf("FAIL link-lost sent=%lu pkts=%lu tmos=%lu\n", sent, pkts, tmos);
          return;
        }
        pk[0] = T_POLL; pk[1] = blk; pk[2] = npk; memset(pk + 3, 0, 7);
        radioTx(pk, 10, true); polls++;
        got = waitReply(T_ACK, blk, ackTmo, true);
      }
      tAck += nowUs() - ta;
      idle = 0;
      uint64_t bm; memcpy(&bm, pk + 2, 8);
      acked |= bm & full;
    }
    sent += n; blk++;
    // rotate buffers: the prefetched one becomes current, queue the next block
    curB ^= 1;
    int nb = curB ^ 1;
    fillN[nb] = 0; wantN[nb] = min(blkBytes(), total - queued); queued += wantN[nb];
  }
  uint64_t dt = nowUs() - t0;
  Serial.printf("DONE bytes=%lu us=%lu pkts=%lu retx=%lu polls=%lu tmos=%lu txfail=%lu kbps=%.1f "
                "t_start=%lu t_data=%lu t_ack=%lu t_usbwait=%lu blocks=%u\n",
                total, (uint32_t)dt, pkts, retx, polls, tmos, txFail, total * 8000.0 / dt,
                (uint32_t)tStart, (uint32_t)tData, (uint32_t)tAck, (uint32_t)tFill, blk);
  profPrint(toaUs(cfg.pl));
}

// ---------------------------------------------------------------- mesh-style relay (RELAY / RX)
// Per packet: A sends T_RDATA, B answers the hop ACK, B forwards the packet (T_RFWD) as if to the next node,
// A plays that next node and ACKs it, then sends the following packet. A's next T_RDATA also counts as an
// implicit ACK for B, like a mesh node hearing its packet being passed on.
static uint8_t fwdBuf[260], fwdLen;

// waits up to tmo for one good packet into pk; RX stays on when a packet is returned
static int waitPkt(uint32_t tmo, bool pump) {
  startRx();
  uint32_t t0 = us32();
  while (us32() - t0 < tmo) {
    wdtFeed();
    if (pump) { pumpIn(); pumpOut(); }
    int n = pollRx(pk, false);
    if (n > 0) return n;
  }
  stopRx();
  return 0;
}

static void sendAck16(uint16_t s) {
  uint8_t a[10] = {T_ACK, (uint8_t)s, (uint8_t)(s >> 8)};
  radioTx(a, 10, false);
}

static void doRelay(uint32_t total) {
  uint32_t ackTmo = 1500 + 3 * toaUs(12);
  uint32_t retries = 0, dupFwd = 0, hopAcks = 0, mismatch = 0;
  uint8_t sess = (uint8_t)(us32() ^ (us32() >> 8)) | 1;
  txFail = 0;
  curB = 0;
  uint32_t queued = 0;
  wantN[0] = min(blkBytes(), total); fillN[0] = 0; queued += wantN[0];
  wantN[1] = min(blkBytes(), total - queued); fillN[1] = 0; queued += wantN[1];
  if (!fillBlocking(0, 5000)) { Serial.printf("FAIL usb-timeout\n"); return; }

  uint64_t t0 = nowUs();
  uint32_t sent = 0;
  uint16_t s = 0;
  while (sent < total) {
    if (!fillBlocking(curB, 5000)) { Serial.printf("FAIL usb-timeout sent=%lu\n", sent); return; }
    uint32_t n = wantN[curB];
    for (uint32_t off = 0; off < n; off += dpp, s++) {
      uint8_t len = min((uint32_t)dpp, n - off);
      bool fwdOk = false;
      uint32_t tLost = millis();
      for (uint32_t att = 0; !fwdOk; att++) {
        if (millis() - tLost > 20000) {
          drainInput();
          Serial.printf("FAIL link-lost seq=%u retries=%lu\n", s, retries);
          return;
        }
        if (att) retries++;
        pk[0] = T_RDATA; pk[1] = s; pk[2] = s >> 8; pk[3] = sess;
        memcpy(pk + 4, blkbuf[curB] + off, len);
        radioTx(pk, 4 + len, true);
        // hop ACK, then the forwarded copy
        uint32_t tw = us32(), lim = ackTmo + toaUs(4 + len) + 2000;
        for (;;) {
          uint32_t el = us32() - tw;
          if (el >= lim) break;
          int m = waitPkt(lim - el, true);
          if (m <= 0) break;
          uint16_t ps = pk[1] | pk[2] << 8;
          if (pk[0] == T_ACK && ps == s) hopAcks++;
          else if (pk[0] == T_RFWD && ps == s && m == 4 + len) { memcpy(fwdBuf, pk + 4, len); fwdOk = true; break; }
          else if (pk[0] == T_RFWD && ps == (uint16_t)(s - 1)) { dupFwd++; sendAck16(ps); }  // our last ACK was lost
        }
        stopRx();
      }
      sendAck16(s);                                     // ACK as the next hop
      pumpIn();
      if (memcmp(fwdBuf, blkbuf[curB] + off, len)) mismatch++;  // end-to-end check of the relayed copy
    }
    sent += n;
    curB ^= 1;
    int nb = curB ^ 1;
    fillN[nb] = 0; wantN[nb] = min(blkBytes(), total - queued); queued += wantN[nb];
  }
  uint64_t dt = nowUs() - t0;
  Serial.printf("DONE bytes=%lu us=%lu kbps=%.2f pkts=%u retries=%lu hopacks=%lu dupfwd=%lu mismatch=%lu txfail=%lu "
                "radioResets=%lu\n", total, (uint32_t)dt, total * 8000.0 / dt, s, retries, hopAcks, dupFwd, mismatch,
                txFail, radioResets);
}

static uint16_t rExp;
static uint8_t rSess;
static bool rOn;
static uint32_t rFwdRetx, rRelayed;
static inline void radioLock();
static inline void radioUnlock();

// B side: pk holds a T_RDATA packet of n bytes
static void relayHandle(int n) {
  uint32_t ackTmo = 1500 + 3 * toaUs(12);
  radioLock();
  for (;;) {
    uint16_t s = pk[1] | pk[2] << 8;
    if (!rOn || pk[3] != rSess) { rOn = true; rSess = pk[3]; rExp = 0; }
    if (s == rExp) { memcpy(fwdBuf, pk, n); fwdBuf[0] = T_RFWD; fwdLen = n; rExp++; rRelayed++; }
    else if (s != (uint16_t)(rExp - 1)) break;          // out of sequence: ignore
    sendAck16(s);                                       // hop ACK
    radioTx(fwdBuf, fwdLen, false);                     // forward to the "next node"
    bool next = false;
    for (int tries = 0; tries < 4; tries++) {
      int m = waitPkt(ackTmo, false);
      if (m >= 3 && pk[0] == T_ACK && (uint16_t)(pk[1] | pk[2] << 8) == s) break;
      if (m > 4 && pk[0] == T_RDATA) { n = m; next = true; break; }  // implicit ACK: A moved on (or resent)
      rFwdRetx++;
      radioTx(fwdBuf, fwdLen, false);
    }
    if (!next) break;
  }
  startRx();
  radioUnlock();
}

static void doBurst(uint32_t n, uint8_t len) {
  burstLen = constrain(len, 6, cfg.pl);
  for (int i = 0; i < cfg.pl; i++) pk[i] = (uint8_t)(i * 37 + 11);
  txFail = 0;
  profReset();
  uint64_t t0 = nowUs();
  txSeries(n, false, buildRaw);
  uint64_t dt = nowUs() - t0;
  Serial.printf("BURSTDONE n=%lu us=%lu txfail=%lu toa=%lu pl=%u kbps_payload=%.2f\n",
                n, (uint32_t)dt, txFail, toaUs(burstLen), burstLen, n * burstLen * 8000.0 / dt);
  profPrint(toaUs(burstLen));
}

// ---------------------------------------------------------------- RX side
// While receiving, DIO1 is serviced from the GPIOTE interrupt so the packet leaves the SX1280 buffer before
// the next one overwrites it, even when the USB task is busy. The main loop only drains this queue.
#define RQN 32
static uint8_t rq[RQN][256], rqLen[RQN], rqScratch[256];
static volatile uint32_t rqHead, rqTail, rqDrop;
static volatile bool isrArmed, rssiWanted;

static void dio1Isr() {
  if (!isrArmed) return;
  uint32_t h = rqHead;
  if (h - rqTail >= RQN) { if (pollRx(rqScratch, false) > 0) rqDrop++; return; }
  int n = pollRx(rq[h % RQN], rssiWanted);
  if (n > 0) { rqLen[h % RQN] = n; rqHead = h + 1; }
}
// the main loop must not share the SPI bus with dio1Isr
static inline void radioLock() { NVIC_DisableIRQ(GPIOTE_IRQn); }
static inline void radioUnlock() { NVIC_EnableIRQ(GPIOTE_IRQn); }

static uint8_t rxblk[BLKMAX];
static void sendCtl(uint8_t type, uint8_t key, uint64_t bm) {
  uint8_t a[10];
  a[0] = type; a[1] = key; memcpy(a + 2, &bm, 8);
  radioLock();
  radioTx(a, 10, false);
  startRx();
  radioUnlock();
}

// next received packet into pk: from the ISR queue, or polled if an edge was missed
static int rxNext() {
  if (rqTail != rqHead) {
    uint32_t t = rqTail % RQN;
    int n = rqLen[t];
    memcpy(pk, rq[t], n);
    rqTail++;
    return n;
  }
  if (!digitalRead(P_DIO1)) return 0;
  radioLock();
  int n = pollRx(pk, rssiWanted);
  radioUnlock();
  return n;
}

static void doRx() {
  bool active = false, done = false;
  uint8_t sess = 0, cur = 0, lastLen = 0, rnpk = 0;
  uint32_t total = 0, recvd = 0, dataPk = 0, dupPk = 0, raw = 0, rawBad = 0;
  uint64_t bm = 0, prevFull = 0, t0 = 0, t1 = 0;
  rxOk = rxCrc = rxOverrun = 0;
  rqHead = rqTail = rqDrop = 0;
  rOn = false; rFwdRetx = rRelayed = 0;
  rssiWanted = true;
  startRx();
  isrArmed = true;
  while (!Serial.available()) {
    pumpOut();
    int n = rxNext();
    if (n <= 0) continue;
    uint8_t t = pk[0] & 0x7F;
    if (t == T_RAW) {
      raw++;
      for (int i = 4; i < n; i++) if (pk[i] != (uint8_t)(i * 37 + 11)) { rawBad++; break; }
      continue;
    }
    if (t == T_RDATA && n > 4) { relayHandle(n); continue; }
    if (t == T_START && n >= 8) {
      if (!active || pk[1] != sess) {
        sess = pk[1]; memcpy(&total, pk + 2, 4);
        active = true; done = false; cur = 0; bm = 0; recvd = 0; dataPk = 0; dupPk = 0; prevFull = 0;
        rssiWanted = false;
        t0 = nowUs();
        outPrintf("START %lu rssi=%d\n", total, lastRssi);
      }
      sendCtl(T_STARTACK, sess, 0);
      continue;
    }
    if (!active) continue;
    if (t == T_DATA && n > 4) {
      uint8_t blk = pk[1], idx = pk[2], npk = pk[3];
      dataPk++;
      if (blk == cur && !done && idx < npk && npk <= MAXPK) {
        if (bm >> idx & 1) dupPk++;
        else {
          memcpy(rxblk + idx * dpp, pk + 4, n - 4);
          bm |= 1ULL << idx;
          if (idx == npk - 1) lastLen = n - 4;
          rnpk = npk;
        }
        if (bm == fullMask(rnpk)) {
          uint32_t blen = (rnpk - 1) * dpp + lastLen;
          // ACK first if requested so the sender is not kept waiting by USB
          if (pk[0] & F_POLL) { sendCtl(T_ACK, cur, bm); pk[0] = 0; }
          outPush(rxblk, blen);
          recvd += blen; prevFull = bm; bm = 0; cur++;
          if (recvd >= total) {
            done = true; t1 = nowUs();
            outPrintf("END bytes=%lu us=%lu kbps=%.2f datapk=%lu dup=%lu crcerr=%lu overrun=%lu rssi=%d\n",
                      recvd, (uint32_t)(t1 - t0), recvd * 8000.0 / (t1 - t0), dataPk, dupPk, rxCrc, rxOverrun, lastRssi);
          }
        }
      } else dupPk++;
      if (!(pk[0] & F_POLL)) continue;
    } else if (t != T_POLL) continue;
    // answer poll (from DATA|F_POLL or POLL)
    uint8_t blk = pk[1];
    if (blk == cur && !done) sendCtl(T_ACK, blk, bm);
    else if (blk == (uint8_t)(cur - 1)) sendCtl(T_ACK, blk, prevFull);
  }
  isrArmed = false;
  stopRx();
  while (rCount) pumpOut();
  Serial.printf("RXSTAT ok=%lu crc=%lu raw=%lu rawbad=%lu overrun=%lu qdrop=%lu relayed=%lu fwdretx=%lu rssi=%d busyErr=%lu\n",
                rxOk, rxCrc, raw, rawBad, rxOverrun, rqDrop, rRelayed, rFwdRetx, lastRssi, busyErr);
}

// ---------------------------------------------------------------- command loop
static void nfcPinsAsGpio() {
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; while (!NRF_NVMC->READY) {}
    NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk; while (!NRF_NVMC->READY) {}
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; while (!NRF_NVMC->READY) {}
    NVIC_SystemReset();
  }
}

void setup() {
  nfcPinsAsGpio();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  pinMode(P_VEXT, OUTPUT); digitalWrite(P_VEXT, HIGH);
  pinMode(P_CS, OUTPUT); digitalWrite(P_CS, HIGH);
  pinMode(P_NRST, OUTPUT); digitalWrite(P_NRST, HIGH);
  pinMode(P_TXEN, OUTPUT); digitalWrite(P_TXEN, LOW);
  pinMode(P_RXEN, OUTPUT); digitalWrite(P_RXEN, LOW);
  pinMode(P_BUSY, INPUT);
  pinMode(P_DIO1, INPUT);
  Serial.begin(115200);
  rspi.begin();
  rspi.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  delay(50);
  radioReset();
  radioConfig();
  attachInterrupt(P_DIO1, dio1Isr, RISING);
  wdtStart();
}

static char line[64];
static uint8_t lineN;

void loop() {
  nowUs();
  wdtFeed();
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (lineN < sizeof line - 1) line[lineN++] = c; continue; }
    line[lineN] = 0; lineN = 0;
    char *cmd = strtok(line, " ");
    if (!cmd) continue;
    if (!strcmp(cmd, "PING")) {
      cmdv({0xC0, 0});
      uint8_t st = rbuf[0];
      Serial.printf("PONG status=0x%02X pkttype=%d fw=0x%02X%02X busyErr=%lu radioResets=%lu resetreas=0x%lX gpregret=0x%02lX\n",
                    st, (cmdv({0x03, 0, 0}), rbuf[2]), readReg(0x0153), readReg(0x0154), busyErr, radioResets,
                    readResetReason(), NRF_POWER->GPREGRET);
    } else if (!strcmp(cmd, "CFG")) {
      char *a[7];
      for (int i = 0; i < 7; i++) a[i] = strtok(nullptr, " ");
      uint8_t *dst[7] = {&cfg.p1, &cfg.p2, &cfg.p3, &cfg.pwr, &cfg.pre, &cfg.modem, &cfg.pl};
      for (int i = 0; i < 7; i++) if (a[i]) *dst[i] = strtoul(a[i], nullptr, 0);
      if (!a[5]) cfg.modem = M_FLRC;
      if (!a[6]) cfg.pl = 127;
      cfg.pl = constrain(cfg.pl, 16, cfg.modem == M_FLRC ? 127 : 255);
      radioConfig();
      Serial.printf("OK modem=%d p=0x%02X,0x%02X,0x%02X pwr=%d(%ddBm) pre=0x%02X pl=%u pipe=%d toa=%luus toa10=%luus\n",
                    cfg.modem, cfg.p1, cfg.p2, cfg.p3, cfg.pwr, cfg.pwr - 18, cfg.pre, cfg.pl, canPipe(),
                    toaUs(cfg.pl), toaUs(10));
    } else if (!strcmp(cmd, "RX")) {
      doRx();
    } else if (!strcmp(cmd, "TX")) {
      char *a = strtok(nullptr, " ");
      uint32_t n = a ? strtoul(a, nullptr, 0) : 0;
      if (n) doTx(n); else Serial.println("FAIL size");
    } else if (!strcmp(cmd, "RELAY")) {           // RELAY <n> + n raw bytes; the relayed copy comes back on USB
      char *a = strtok(nullptr, " ");
      uint32_t n = a ? strtoul(a, nullptr, 0) : 0;
      if (n) doRelay(n); else Serial.println("FAIL size");
    } else if (!strcmp(cmd, "BURST")) {
      char *a = strtok(nullptr, " "), *b = strtok(nullptr, " ");    // BURST <n> [len]
      doBurst(a ? strtoul(a, nullptr, 0) : 100, b ? strtoul(b, nullptr, 0) : cfg.pl);
    } else if (!strcmp(cmd, "OPT")) {             // OPT <pipe 0/1> <sync 0/1> <ramp code> <win 1..64>
      char *a = strtok(nullptr, " "), *b = strtok(nullptr, " "), *c = strtok(nullptr, " "), *d = strtok(nullptr, " ");
      if (a) optPipe = atoi(a);
      if (b) optSync = atoi(b);
      if (c) optRamp = strtoul(c, nullptr, 0);
      if (d) optWin = constrain(atoi(d), 1, MAXPK);
      radioConfig();
      Serial.printf("OK pipe=%d sync=%d ramp=0x%02X win=%u toa=%luus\n", optPipe, optSync, optRamp, optWin,
                    toaUs(cfg.pl));
    } else if (!strcmp(cmd, "DFU")) {             // jump to the UF2 bootloader (USB drive) for reflashing
      Serial.println("OK DFU"); Serial.flush(); delay(100);
      // enterUf2Dfu() came straight back to the app on this bootloader, so ask for UF2 mode both ways:
      // GPREGRET magic and the RAM signature its double-reset detector looks for.
      __disable_irq();
      NRF_POWER->GPREGRET = 0x57;                     // DFU_MAGIC_UF2_RESET
      *(volatile uint32_t *)0x20007F7C = 0x5A1AD5;    // DFU_DBL_RESET_MAGIC @ DFU_DBL_RESET_MEM
      NVIC_SystemReset();
    } else if (!strcmp(cmd, "RESET")) {
      radioReset(); radioConfig(); Serial.println("OK");
    } else {
      Serial.printf("ERR unknown '%s'\n", cmd);
    }
  }
}
