# SX1280 bench: FLRC / GFSK / LoRa on Meshtastic and Jentastic

Bench tools, raw results and reports from testing FLRC and GFSK on two **ProMicro nRF52840 + Ebyte E28-2G4M27S**
(SX1280, "Faketec 2.4G") boards, first with a minimal modem firmware and then inside Meshtastic and Jentastic.

The firmware changes themselves live in two forks:

- Meshtastic 2.7.26 + FLRC/GFSK presets: [sqtactical/firmware `sx1280-fast-modems`](https://github.com/sqtactical/firmware/tree/sx1280-fast-modems)
  ([docs](https://github.com/sqtactical/firmware/blob/sx1280-fast-modems/docs/sx1280-fast-modems.md))
- Jentastic `testing` + SETFSK/SETFLRC fixes: [sqtactical/jentastic `sx12xx-fsk-flrc`](https://github.com/sqtactical/jentastic/tree/sx12xx-fsk-flrc) (now in Jentastic `testing`; original commits on `sx1280-fsk-flrc-26a0451`)
  ([docs](https://github.com/sqtactical/jentastic/blob/sx12xx-fsk-flrc/docs/sx1280-fsk-flrc.md))

## Headline numbers

All measured with the two boards in one room at −18 dBm, 200-byte payloads.

| What | FLRC | GFSK | LoRa |
|---|---|---|---|
| Minimal modem, PC to PC, block ACK (fastest mode) | 934 kb/s | 1344 kb/s | 181 kb/s (SF5) |
| Meshtastic fork, median DM→ACK round trip | 76-96 ms | 66-73 ms | 126 ms (SHORT_TURBO), 546 ms (MEDIUM_SLOW) |
| Jentastic with fixes, median DM→ACK round trip | 79-97 ms | 69-79 ms | 123 ms (SHORT_TURBO) |
| Jentastic `testing` as shipped | 0 delivered | 250 kbps only, 683 ms | 691 ms |

## Layout

| Path | Contents |
|---|---|
| `firmware/flrc_modem/` | Minimal Arduino modem (FLRC/LoRa/GFSK, ARQ, relay, burst) used for the raw PC-to-PC tests |
| `tools/` | Test scripts (below) |
| `results/modem/` | Raw JSON from the minimal-modem tests (rates, packet length sweeps, ACK window sweeps, relay) |
| `results/meshtastic/` | `mesh_results_v*.json` preset sweeps, `prof_results_*.json` profiler runs |
| `results/jentastic/` | `jt_*.json` mode sweeps and diagnostics, `prof_jt_results.json`, `stock281_*.json` (upstream 2.8.1) |
| `report/` | `Informe_SX1280_FLRC_LoRa_GFSK.pdf` (minimal modem), `Meshtastic_FLRC_GFSK_Fast_Modems.pdf` (Meshtastic fork) |
| `patches/` | The same changes as the forks, as single patches against Meshtastic `v2.7.26.54e0d8d` and Jentastic `26a0451` |

## Tools

All Python, using the `meshtastic` library (the scripts patch out its 1 s sleep after every serial write on Windows).
Defaults assume node A on `COM14` and node B on `COM27`.

| Script | Purpose |
|---|---|
| `flash.py` | Build/flash the minimal modem, or flash any UF2: `--meshtastic` puts nodes in DFU with `--enter-dfu` and copies the UF2 |
| `flrc_host.py`, `len_sweep.py`, `make_tables.py` | Minimal-modem file transfer tests, packet-length sweep, report tables |
| `mesh_speed.py` | Meshtastic fork: switch both nodes to a preset, then ACK test (DMs with want_ack) and stream test (broadcasts) |
| `mesh_trace.py` | Capture node B's raw serial log while A sends |
| `prof_run.py`, `prof_show.py` | Collect and print the firmware profiler (`-DPROFILE_PACKETS`) output from both nodes |
| `jt_speed.py` | Jentastic: switch modes with `/nexus-local SETFLRC …` / `SETFSK …` and run the same ACK and stream tests |
| `jt_trace.py`, `jt_gap.py`, `prof_jt.py` | Jentastic log capture, broadcast-spacing test (signed packets), profiler collection |
| `jc_test.py` | JenCore group-text test over LoRa between two nodes |
| `rnode_ble.py` | RNode over BLE NUS: KISS probe and Reticulum frames A→B over LoRa (needs pairing or `NO_PIN`) |
| `split_commits.py` | Splits a working-tree diff into topic commits (used to build the fork branches) |

Paths inside some scripts point at the original bench setup (`W:\…`, `%LOCALAPPDATA%\flrc-tools`); adjust before use.

Node configuration backups are deliberately not included: they contain the nodes' private keys.
