#!/usr/bin/env python3
"""ARGUS build_dashboard.py

Generate a SELF-CONTAINED HTML dashboard for the ARGUS VIO health monitor (Pillar
3) -- no server, no install, no CDN. All charts are matplotlib figures embedded as
base64 PNGs, so the single .html opens in any browser offline. (Streamlit was the
original plan; pip is SSL-blocked in this environment and a static bundle is more
portable for a demo anyway.)

Surfaces:
  * KPI cards (Scenario-D health + the Scenario-A drift gate),
  * the Scenario-D health timeline (NOMINAL -> blackout LOST -> recover),
  * the lights-off recovery ablation (C1 vs C3),
  * the cross-scenario drift ablation (A/B/C vs the 1.5% Honeywell gate).

Run with the eval interpreter:
  ~/.venvs/argus-eval/bin/python scripts/build_dashboard.py
-> data/eval/argus_dashboard.html
"""
from __future__ import annotations

import base64
import io
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

EVAL = Path.home() / "argus/data/eval"
SCEN = ["scenario_A", "scenario_B", "scenario_C"]


def b64(path: Path) -> str:
    if not path.is_file():
        return ""
    return base64.b64encode(path.read_bytes()).decode()


def fig_b64(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.is_file() else {}


def drift_chart() -> str:
    rows = []
    for s in SCEN:
        m = load_json(EVAL / "C1_klt" / s / "metrics.json")
        if m:
            rows.append((s.replace("scenario_", "Zone "), m.get("drift_pct_ate", 0.0),
                         m.get("path_length_m", 0.0)))
    if not rows:
        return ""
    fig, ax = plt.subplots(figsize=(7, 3.4))
    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    colors = ["#2e7d32" if v < 1.5 else "#f9a825" for v in vals]
    bars = ax.bar(labels, vals, color=colors)
    ax.axhline(1.5, color="#c62828", ls="--", lw=1.5, label="1.5% Honeywell gate")
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.05, f"{v:.2f}%", ha="center", fontsize=9)
    ax.set_ylabel("ATE drift %")
    ax.set_title("Pillar 1 — drift vs distance (C1 KLT baseline)")
    ax.legend(); ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(0, max(2.0, max(vals) * 1.3))
    return fig_b64(fig)


def card(title: str, value: str, sub: str = "", good: bool | None = None) -> str:
    cls = "" if good is None else (" good" if good else " bad")
    return (f'<div class="card{cls}"><div class="cval">{value}</div>'
            f'<div class="ctitle">{title}</div><div class="csub">{sub}</div></div>')


def main() -> int:
    scd = load_json(EVAL / "scenario_D" / "scenario_D_metrics.json")
    c3 = scd.get("offline_c3", {})
    c1 = scd.get("offline_c1", {})
    scenA = load_json(EVAL / "C1_klt" / "scenario_A" / "metrics.json")

    # KPI cards.
    cards = "".join([
        card("Scenario A drift (ATE)", f"{scenA.get('drift_pct_ate','—')}%",
             "gate &lt; 1.5%", good=scenA.get("drift_pct_ate", 9) < 1.5),
        card("Health NOMINAL (lit)", f"{c3.get('pct_nominal','—')}%",
             "healthy tracking in lit zones"),
        card("Blackout detected", f"{c3.get('pct_lost','—')}% LOST",
             f"time-in-LOST {c3.get('time_in_lost_s','—')} s", good=True),
        card("Recovery activations", f"{c3.get('recovery_activations','—')}",
             f"C1 baseline: {c1.get('recovery_activations','—')}", good=True),
        card("Recovery hold flagged", f"{c3.get('time_recovery_active_s','—')} s",
             "drone yields to monitor (C3)"),
        card("VIO drift thru blackout", f"{c3.get('vio_drift',{}).get('max_m','—')} m",
             "dead-reckoning, no loop closure"),
    ])

    # Ablation table.
    def row(label, key, sub=None):
        cv = c3.get(key, "—") if sub is None else c3.get(sub, {}).get(key, "—")
        bv = c1.get(key, "—") if sub is None else c1.get(sub, {}).get(key, "—")
        return f"<tr><td>{label}</td><td>{cv}</td><td>{bv}</td></tr>"
    abl = "".join([
        row("NOMINAL %", "pct_nominal"), row("DEGRADED %", "pct_degraded"),
        row("LOST %", "pct_lost"), row("time-in-LOST (s)", "time_in_lost_s"),
        row("recovery activations", "recovery_activations"),
        row("time recovery held (s)", "time_recovery_active_s"),
        row("VIO max drift (m)", "max_m", "vio_drift"),
    ])

    timeline_img = b64(EVAL / "scenario_D" / "scenario_D_health.png")
    drift_img = drift_chart()

    html = f"""<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ARGUS — VIO Health Dashboard</title>
<style>
 :root{{--bg:#0e1117;--panel:#161b22;--fg:#e6edf3;--mut:#8b949e;--acc:#1565c0;--good:#2e7d32;--bad:#c62828}}
 *{{box-sizing:border-box}} body{{margin:0;background:var(--bg);color:var(--fg);
   font-family:-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif}}
 header{{padding:22px 28px;border-bottom:1px solid #30363d}}
 h1{{margin:0;font-size:22px}} .sub{{color:var(--mut);font-size:13px;margin-top:4px}}
 main{{padding:22px 28px;max-width:1100px;margin:0 auto}}
 h2{{font-size:15px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;
    margin:30px 0 12px;border-bottom:1px solid #30363d;padding-bottom:6px}}
 .grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(165px,1fr));gap:12px}}
 .card{{background:var(--panel);border:1px solid #30363d;border-radius:10px;padding:14px 16px}}
 .card.good{{border-left:3px solid var(--good)}} .card.bad{{border-left:3px solid var(--bad)}}
 .cval{{font-size:25px;font-weight:650}} .ctitle{{font-size:13px;margin-top:4px}}
 .csub{{font-size:11px;color:var(--mut);margin-top:2px}}
 img{{max-width:100%;border-radius:10px;border:1px solid #30363d;background:#fff}}
 table{{border-collapse:collapse;width:100%;font-size:14px;background:var(--panel);
   border-radius:10px;overflow:hidden}}
 th,td{{padding:9px 14px;text-align:left;border-bottom:1px solid #30363d}}
 th{{background:#1c2230;color:var(--mut);font-weight:600}}
 td:nth-child(2){{color:#7ee787;font-variant-numeric:tabular-nums}}
 td:nth-child(3){{color:var(--mut);font-variant-numeric:tabular-nums}}
 .foot{{color:var(--mut);font-size:12px;margin-top:30px}}
</style></head><body>
<header><h1>ARGUS — Robust VIO Health Monitor</h1>
<div class=sub>GPS-denied stereo-inertial VIO with failure detection &amp; recovery</div></header>
<main>
 <h2>Key metrics</h2><div class=grid>{cards}</div>

 <h2>Scenario D — lights-off detection &amp; recovery</h2>
 <p class=sub>Forward traverse; the Zone-B lights cut mid-flight. The monitor holds NOMINAL while
  lit (~100 inliers), flips to LOST the moment the cameras go dark, and (C3) raises the recovery
  hold; it recovers when the lights return.</p>
 {'<img src="data:image/png;base64,'+timeline_img+'">' if timeline_img else '<p>(timeline image missing)</p>'}

 <h2>Lights-off recovery ablation (C1 vs C3)</h2>
 <table><tr><th>metric</th><th>C3 (recovery on)</th><th>C1 (recovery off)</th></tr>{abl}</table>

 <h2>Pillar 1 — drift gate (cross-scenario)</h2>
 {'<img src="data:image/png;base64,'+drift_img+'">' if drift_img else '<p>(drift chart unavailable)</p>'}

 <div class=foot>Generated by scripts/build_dashboard.py from data/eval/*. Self-contained — no
  server or network required. Source: argus_health (Pillar 3), VINS-Fusion (Pillar 1).</div>
</main></body></html>"""

    out = EVAL / "argus_dashboard.html"
    out.write_text(html)
    print(f"[dashboard] -> {out}  ({len(html)//1024} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
