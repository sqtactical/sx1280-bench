"""Collect every results_*.json into one summary (summary.json + markdown tables on stdout)."""
import glob, json, os

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    p = os.path.join(HERE, name)
    return json.load(open(p)) if os.path.exists(p) else None


def rows(doc, key):
    out = {}
    for r in (doc or {}).get("results", []):
        v = r.get(key, {})
        out[r["mode"]] = (r.get("nominal_kbps") or r.get("eff_kbps"), v)
    return out


def merged(pattern, key):
    """rows from every file matching pattern (sorted by name); a successful later run replaces an earlier one"""
    out = {}
    for p in sorted(glob.glob(os.path.join(HERE, pattern))):
        for mode, (nom, v) in rows(json.load(open(p)), key).items():
            if "error" not in v or mode not in out:
                out[mode] = (nom, v)
    return out


def main():
    summary = {}
    for modem, win64 in (("flrc", "results_flrc_v2.json"), ("gfsk", "results_gfsk.json"), ("lora", "results_lora.json")):
        w64 = rows(load(win64), "file")
        w1 = merged(f"results_{modem}_win1*.json", "file")
        rl = merged(f"results_{modem}_relay*.json", "relay")
        bu = rows(load(win64), "burst")
        table = []
        for mode, (nom, f) in w64.items():
            nom = nom or 0
            b = bu.get(mode, (0, {}))[1]
            f1 = w1.get(mode, (0, {}))[1]
            r = rl.get(mode, (0, {}))[1]
            table.append({"mode": mode, "nominal": nom, "burst": b.get("raw_kbps"), "win64": f.get("host_kbps"),
                          "win1": f1.get("host_kbps"), "relay": r.get("e2e_kbps"),
                          "sha": all(x.get("sha_ok", True) for x in (f, f1, r) if x),
                          "rssi": b.get("rssi")})
        summary[modem] = table
        print(f"\n### {modem.upper()}\n")
        print("| Modo | Nominal | Ráfaga sin ACK | ACK/64 paq. | ACK/paquete | Relay (malla) | SHA |")
        print("|---|---|---|---|---|---|---|")
        for t in table:
            def f(x, ref=t["nominal"]):
                return "–" if x is None else f"{x:.2f} ({100 * x / ref:.0f}%)" if ref else f"{x:.2f}"
            print(f"| {t['mode']} | {t['nominal']:.2f} | {f(t['burst'])} | {f(t['win64'])} | {f(t['win1'])} | "
                  f"{f(t['relay'])} | {'OK' if t['sha'] else 'FALLO'} |")
    ws = {}
    for p in sorted(glob.glob(os.path.join(HERE, "winsweep_*.json"))):
        d = json.load(open(p))
        r = d["results"][0]
        ws.setdefault(d["modem"], {})[d["win"]] = {"mode": r["mode"], "kbps": r["file"].get("host_kbps"),
                                                   "nominal": r["nominal_kbps"]}
    summary["winsweep"] = ws
    if ws:
        print("\n### Ventana de ACK (modo más rápido)\n")
        for m, d in ws.items():
            print(m, {w: round(v["kbps"] or 0, 1) for w, v in sorted(d.items())})
    json.dump(summary, open(os.path.join(HERE, "summary.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
