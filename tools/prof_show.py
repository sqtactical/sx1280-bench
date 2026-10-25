"""Print prof_results.json as per-node tables, slowest stages and threads first."""
import json, os, sys

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "prof_results.json")
for preset, phases in json.load(open(path)).items():
    for phase, res in phases.items():
        print(f"\n##### {preset} {phase}: {json.dumps(res['outcome'])}")
        for node in ("A", "B"):
            b = res[node]
            if not b:
                print(f"  {node}: no PROF block")
                continue
            print(f"  -- {node}: window {b['window_ms']} ms, raw log lines {b['log_lines']}")
            for kind in ("stage", "thread"):
                for name, s in sorted(b[kind].items(), key=lambda x: -x[1]["total_ms"])[:12]:
                    print(f"     {kind:6} {name:16} n={s['n']:5}  avg={s['avg_us']:7} us  max={s['max_us']:7} us  total={s['total_ms']:6} ms")
