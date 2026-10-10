#!/usr/bin/env python3
"""Depth-restoration metrics and capture files for demos/restore_depth_viewer.py.

Pure numpy/OpenCV -- no ROS, no network -- so the numbers can be tested on their own.

Given the raw sensor depth, the model's restored depth and a ground-truth depth (all uint16
millimetres, 0 = no depth, same crop), `evaluate_frame` scores both the restored AND the raw
depth against the ground truth, in two regions:

  object   ground-truth-valid pixels where the sensor failed: raw depth is 0, or differs from
           the ground truth by more than fail_mm. This is where a restoration model has work
           to do.
  all      every pixel with valid ground truth (dilutes the object region with the easy,
           unchanged background).

`write_capture` saves the frames, a metrics.json and a self-contained report.html.
"""
import base64
import datetime as dt
import html
import json
import warnings
from pathlib import Path

import cv2
import numpy as np

DELTAS = {1.05: "delta_1_05", 1.10: "delta_1_10", 1.25: "delta_1_25"}      # threshold -> metric key
METRIC_KEYS = ("rmse_m", "mae_m", "absrel", "delta_1_05", "delta_1_10", "delta_1_25")
REGIONS = ("object", "all")
ALIGN_WARN_MM = 10.0     # median |raw - ground truth| above this suggests the camera moved

DEFINITIONS = {
    "ground truth": "G, uint16 mm, 0 = invalid. Taken as given: the scene must be the same view as "
                    "the captured frames (camera not moved).",
    "pixels scored": "region AND G > 0 AND prediction > 0. `coverage` = scored / (region AND G > 0): "
                     "how much of the region the prediction actually covers. A prediction of 0 "
                     "(no depth) is left out of the errors, not counted as a perfect or a bad value.",
    "region object": "G > 0 and (raw == 0 or |raw - G| > fail_mm).",
    "region all": "G > 0.",
    "rmse_m": "sqrt(mean((P - G)^2)), metres. Lower is better.",
    "mae_m": "mean(|P - G|), metres. Lower is better.",
    "absrel": "mean(|P - G| / G), dimensionless. Lower is better.",
    "delta_1_05": "fraction of scored pixels with max(P/G, G/P) < 1.05. Higher is better. "
                  "delta_1_10 and delta_1_25 use 1.10 and 1.25.",
    "mean / std": "per-frame metrics are computed first, then averaged over the captured frames "
                  "(std is the sample std, ddof = 1).",
    "alignment check": "median |raw - G| over pixels valid in both; far above a few mm means the "
                       "camera moved between the ground-truth capture and these frames.",
}


# ── Images ──────────────────────────────────────────────────────────────────

def colorize_depth(depth_mm, lo, hi):
    """Turbo colour map of depth between lo and hi metres; black = no depth."""
    depth = depth_mm.astype(np.float32) * 1e-3
    scaled = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    img = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth_mm == 0] = 0
    return img


def error_map(pred_mm, gt_mm, max_mm=50.0):
    """Inferno map of |pred - gt| from 0 to max_mm; black where either has no depth."""
    err = np.abs(pred_mm.astype(np.float32) - gt_mm.astype(np.float32))
    img = cv2.applyColorMap((np.clip(err / max_mm, 0.0, 1.0) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    img[(pred_mm == 0) | (gt_mm == 0)] = 0
    return img


def _caption(img, text):
    strip = np.zeros((26, img.shape[1], 3), np.uint8)
    cv2.putText(strip, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([strip, img])


def _b64_png(img):
    return base64.b64encode(cv2.imencode(".png", img)[1].tobytes()).decode()


# ── Metrics ─────────────────────────────────────────────────────────────────

def reference_depth(depths_mm):
    """Ground-truth depth from several raw frames of a static scene: the per-pixel median of
    the valid (> 0) samples; 0 where no frame had depth."""
    stack = np.stack(depths_mm).astype(np.float32)
    stack[stack == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)      # all-NaN slices -> NaN -> 0 below
        med = np.nanmedian(stack, axis=0)
    return np.round(np.nan_to_num(med, nan=0.0)).astype(np.uint16)


def region_masks(raw_mm, gt_mm, fail_mm):
    valid = gt_mm > 0
    both = valid & (raw_mm > 0)
    off = np.zeros(gt_mm.shape, bool)
    off[both] = np.abs(raw_mm[both].astype(np.int32) - gt_mm[both].astype(np.int32)) > fail_mm
    return {"object": valid & ((raw_mm == 0) | off), "all": valid}


def depth_metrics(pred_mm, gt_mm, region):
    """Metrics of `pred_mm` against `gt_mm` over `region` (see the module docstring)."""
    ref = region & (gt_mm > 0)
    ok = ref & (pred_mm > 0)
    n_ref, n = int(ref.sum()), int(ok.sum())
    out = {"pixels": n_ref, "scored": n, "coverage": (n / n_ref) if n_ref else None}
    if n == 0:
        out.update({k: None for k in METRIC_KEYS})
        return out
    p, g = pred_mm[ok].astype(np.float64), gt_mm[ok].astype(np.float64)
    d = p - g
    ratio = np.maximum(p / g, g / p)
    out.update(rmse_m=float(np.sqrt(np.mean(d ** 2)) / 1000.0),
               mae_m=float(np.mean(np.abs(d)) / 1000.0),
               absrel=float(np.mean(np.abs(d) / g)),
               **{key: float(np.mean(ratio < thr)) for thr, key in DELTAS.items()})
    return out


def evaluate_frame(raw_mm, restored_mm, gt_mm, fail_mm):
    """({region: {"restored": metrics, "raw": metrics}}, alignment median |raw - gt| in mm, masks)."""
    masks = region_masks(raw_mm, gt_mm, fail_mm)
    both = (gt_mm > 0) & (raw_mm > 0)
    align = float(np.median(np.abs(raw_mm[both].astype(np.float64) - gt_mm[both]))) if both.any() else None
    regions = {name: {"restored": depth_metrics(restored_mm, gt_mm, m),
                      "raw": depth_metrics(raw_mm, gt_mm, m)} for name, m in masks.items()}
    return regions, align, masks


def summarize(per_frame_regions):
    """{region: {kind: {key: {"mean", "std", "n"}}}} over the frames (None-valued frames skipped)."""
    out = {}
    for region in REGIONS:
        out[region] = {}
        for kind in ("restored", "raw"):
            stats = {}
            for key in METRIC_KEYS + ("coverage", "pixels"):
                vals = [f[region][kind][key] for f in per_frame_regions if f[region][kind][key] is not None]
                stats[key] = ({"mean": float(np.mean(vals)),
                               "std": float(np.std(vals, ddof=1)) if len(vals) > 1 else None,
                               "n": len(vals)} if vals else None)
            out[region][kind] = stats
    return out


# ── Report ──────────────────────────────────────────────────────────────────

def _rounded(obj, digits=6):
    if isinstance(obj, dict):
        return {k: _rounded(v, digits) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_rounded(v, digits) for v in obj]
    if isinstance(obj, float):
        return round(obj, digits)
    return obj


_COLUMNS = (("rmse_m", "RMSE (m) ↓"), ("mae_m", "MAE (m) ↓"), ("absrel", "AbsRel ↓"),
            ("delta_1_05", "δ&lt;1.05 ↑"), ("delta_1_10", "δ&lt;1.10 ↑"), ("delta_1_25", "δ&lt;1.25 ↑"),
            ("coverage", "coverage"), ("pixels", "pixels"))


def _fmt(key, stat):
    if stat is None or stat.get("mean") is None:
        return "–"
    m, s = stat["mean"], stat.get("std")
    if key in ("rmse_m", "mae_m", "absrel"):
        return f"{m:.4f}" + (f" ± {s:.4f}" if s is not None else "")
    if key == "pixels":
        return f"{m:,.0f}"
    return f"{100 * m:.1f}%" + (f" ± {100 * s:.1f}" if s is not None else "")


def _summary_table(summary):
    rows = []
    for region in REGIONS:
        for kind in ("restored", "raw"):
            stats = summary[region][kind]
            cells = "".join(f"<td>{_fmt(k, stats[k])}</td>" for k, _ in _COLUMNS)
            cls = ' class="restored"' if kind == "restored" else ""
            rows.append(f"<tr{cls}><th>{region}</th><th>{'restored' if kind == 'restored' else 'raw sensor'}</th>{cells}</tr>")
    head = "".join(f"<th>{label}</th>" for _, label in _COLUMNS)
    return f"<table><thead><tr><th>region</th><th>depth</th>{head}</tr></thead><tbody>{''.join(rows)}</tbody></table>"


def _frame_table(frames):
    def cell(f, kind, key):
        stat = (f.get("regions") or {}).get("object", {}).get(kind, {}).get(key)
        return _fmt(key, None if stat is None else {"mean": stat})

    rows = []
    for f in frames:
        align = f.get("alignment_median_abs_mm")
        sam = f.get("sam2_score")
        rows.append(f"<tr><th>{f['index']}</th><td>{f['server_ms']:.0f}</td><td>{f['latency_ms']:.0f}</td>"
                    f"<td>{'–' if sam is None else format(sam, '.2f')}</td>"
                    f"<td>{'–' if align is None else format(align, '.1f')}</td>"
                    f"<td>{cell(f, 'restored', 'rmse_m')}</td><td>{cell(f, 'raw', 'rmse_m')}</td>"
                    f"<td>{cell(f, 'restored', 'absrel')}</td><td>{cell(f, 'restored', 'delta_1_05')}</td></tr>")
    return ("<table><thead><tr><th>frame</th><th>server ms</th><th>→restored ms</th><th>SAM2</th>"
            "<th>align mm</th><th>obj RMSE (m)</th><th>obj RMSE raw</th>"
            "<th>obj AbsRel</th><th>obj δ&lt;1.05</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table>")


_CSS = """
:root{--bg:#fff;--fg:#1d2126;--mut:#5b6570;--line:#d9dee3;--row:#f3f6f9;--acc:#0b6bcb;--warn:#b3261e;color-scheme:light dark}
@media (prefers-color-scheme:dark){:root{--bg:#14171a;--fg:#e6e9ec;--mut:#9aa5b0;--line:#2c333a;--row:#1c2126;--acc:#6cb2ff;--warn:#ff8a80}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,sans-serif}
main{max-width:1180px;margin:0 auto;padding:24px 16px 64px}
h1{font-size:1.4rem;margin:0 0 4px}h2{font-size:1.1rem;margin:32px 0 8px;border-bottom:1px solid var(--line);padding-bottom:4px}
.mut{color:var(--mut)}.warn{color:var(--warn);font-weight:600}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;display:block;overflow-x:auto}
th,td{padding:4px 10px;border-bottom:1px solid var(--line);text-align:right;white-space:nowrap}
th:first-child,td:first-child{text-align:left}thead th{color:var(--mut);font-weight:600}
tbody th{font-weight:600;text-align:left}tr.restored{background:var(--row)}
img{max-width:100%;height:auto;border:1px solid var(--line);display:block}
.frame{margin:14px 0}dl{display:grid;grid-template-columns:max-content 1fr;gap:2px 16px;margin:0}dt{color:var(--mut)}dd{margin:0}
code{background:var(--row);padding:0 4px;border-radius:3px}
"""


def render_report(record, view_images, gt_image):
    """Self-contained HTML (images embedded) for a capture's `record` (the metrics.json dict)."""
    e = html.escape
    m, gt = record["model"], record["ground_truth"]
    name = m.get("name", "FRAME" if m["port"] == 32000 else m["health"].get("model", ""))
    title = f"Depth restoration capture — :{m['port']} {name} — {record['created']}"
    parts = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'>"
             f"<meta name='viewport' content='width=device-width,initial-scale=1'><title>{e(title)}</title>"
             f"<style>{_CSS}</style></head><body><main>",
             f"<h1>{e(title)}</h1><div class='mut'>{record['n_frames']} frames · {e(record['tool'])}</div>"]

    parts.append("<h2>Setup</h2><dl>")
    health = ", ".join(f"{k}={v}" for k, v in m["health"].items())
    rows = [("model", f":{m['port']} — {health}"),
            ("ground truth", gt["source"] if gt else "NONE — metrics were not computed"),
            ("crop", f"{record['camera']['crop'][0]}×{record['camera']['crop'][1]} px"),
            ("SAM2 prompts", e(str(record["prompts"])) if record["prompts"] else "none"),
            ("object region", f"raw depth 0 or |raw − ground truth| > {record['settings']['fail_mm']:g} mm")]
    parts += [f"<dt>{e(k)}</dt><dd>{e(v) if k != 'SAM2 prompts' else v}</dd>" for k, v in rows]
    parts.append("</dl>")

    if not gt:
        parts.append("<p class='warn'>No ground-truth depth: only the frames and timings were saved. "
                     "Press <code>g</code> with the opaque stand-in in place, or pass <code>--gt</code>, "
                     "then capture again.</p>")
    else:
        parts.append("<h2>Summary</h2><p class='mut'>Mean ± std over the captured frames. The raw sensor row is "
                     "scored against the same ground truth: where it has no depth it is left out, see "
                     "coverage.</p>")
        parts.append(_summary_table(record["summary"]))
        warn = [f for f in record["frames"] if (f.get("alignment_median_abs_mm") or 0) > ALIGN_WARN_MM]
        if warn:
            parts.append(f"<p class='warn'>Alignment check: median |raw − ground truth| exceeds {ALIGN_WARN_MM:g} mm "
                         f"in {len(warn)} of {record['n_frames']} frames — the camera may have moved since the "
                         "ground truth was captured, which would make these metrics meaningless.</p>")
        parts.append("<h2>Per frame</h2><p class='mut'>obj = the object region, restored depth unless marked raw. "
                     "server ms = request to reply; →restored ms = camera frame arriving to restored image ready; "
                     "align mm = the camera-moved check.</p>" + _frame_table(record["frames"]))

    parts.append("<h2>Frames</h2><p class='mut'>Left to right: raw depth, restored depth"
                 + (", |restored − ground truth| (black = no depth; 0–50 mm)" if gt else "")
                 + f". Depth colours span {record['settings']['depth_range_m'][0]:g}–"
                   f"{record['settings']['depth_range_m'][1]:g} m.</p>")
    for f, img in zip(record["frames"], view_images):
        parts.append(f"<div class='frame'><div class='mut'>frame {f['index']} — "
                     f"<a href='{e(f['files']['view'])}'>view</a> · <a href='{e(f['files']['raw_depth'])}'>raw</a> · "
                     f"<a href='{e(f['files']['restored_depth'])}'>restored</a> · <a href='{e(f['files']['rgb'])}'>rgb</a>"
                     f"</div><img alt='frame {f['index']}' src='data:image/png;base64,{_b64_png(img)}'></div>")
    if gt_image is not None:
        parts.append(f"<h2>Ground truth</h2><img alt='ground truth depth' src='data:image/png;base64,{_b64_png(gt_image)}'>")

    parts.append("<h2>Definitions</h2><dl>" + "".join(f"<dt>{e(k)}</dt><dd>{e(v)}</dd>"
                                                      for k, v in DEFINITIONS.items()) + "</dl>")
    parts.append("</main></body></html>")
    return "".join(parts)


def write_capture(root, tag, meta, trials, gt_mm, gt_source, fail_mm, depth_range):
    """Save one capture under root/<tag>_<time>/ and return (dir, record).

    trials: dicts with seq, rgb (RGB uint8), raw, restored (uint16 mm), server_ms, latency_ms,
            info (reply fields) and points (SAM2 prompts used).
    gt_mm / gt_source: ground-truth depth and a sentence saying where it came from, or None.
    """
    now = dt.datetime.now().astimezone()
    out = Path(root) / f"{tag}_{now.strftime('%Y%m%d_%H%M%S')}"
    n = 1
    while out.exists():
        n += 1
        out = Path(root) / f"{tag}_{now.strftime('%Y%m%d_%H%M%S')}_{n}"
    out.mkdir(parents=True)
    lo, hi = depth_range

    frames, per_frame, view_images = [], [], []
    if gt_mm is not None:
        cv2.imwrite(str(out / "gt_depth.png"), gt_mm)
    for i, t in enumerate(trials):
        stem = f"frame_{i:02d}"
        files = {"rgb": f"{stem}_rgb.png", "raw_depth": f"{stem}_raw_depth.png",
                 "restored_depth": f"{stem}_restored_depth.png", "view": f"{stem}_view.png"}
        cv2.imwrite(str(out / files["rgb"]), cv2.cvtColor(t["rgb"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / files["raw_depth"]), t["raw"])
        cv2.imwrite(str(out / files["restored_depth"]), t["restored"])
        panels = [_caption(colorize_depth(t["raw"], lo, hi), "raw depth"),
                  _caption(colorize_depth(t["restored"], lo, hi), "restored depth")]
        rec = {"index": i, "sequence": int(t["seq"]), "files": files,
               "server_ms": float(t["server_ms"]), "latency_ms": float(t["latency_ms"]),
               "sam2_score": t["info"].get("sam2_score"), "info": t["info"],
               "points": [list(p) for p in t["points"]]}
        if gt_mm is not None:
            regions, align, masks = evaluate_frame(t["raw"], t["restored"], gt_mm, fail_mm)
            files["object_region"] = f"{stem}_object_region.png"
            cv2.imwrite(str(out / files["object_region"]), masks["object"].astype(np.uint8) * 255)
            panels.append(_caption(error_map(t["restored"], gt_mm), "|restored - ground truth|, 0-50 mm"))
            rec["regions"], rec["alignment_median_abs_mm"] = regions, align
            per_frame.append(regions)
        view = np.hstack(panels)
        cv2.imwrite(str(out / files["view"]), view)
        view_images.append(view)
        frames.append(rec)

    record = {
        "tool": "demos/restore_depth_viewer.py",
        "created": now.isoformat(timespec="seconds"),
        "n_frames": len(trials),
        "model": meta["model"],
        "camera": meta["camera"],
        "prompts": meta["prompts"],
        "settings": {"fail_mm": fail_mm, "depth_range_m": [lo, hi], "align_warn_mm": ALIGN_WARN_MM},
        "ground_truth": ({"source": gt_source, "file": "gt_depth.png",
                          "valid_fraction": float((gt_mm > 0).mean())} if gt_mm is not None else None),
        "units": {"depth_png": "uint16 millimetres, 0 = no depth", "rmse_m / mae_m": "metres",
                  "absrel": "ratio", "delta_*": "fraction of scored pixels, 0..1"},
        "definitions": DEFINITIONS,
        "frames": frames,
        "summary": summarize(per_frame) if per_frame else None,
    }
    record = _rounded(record)
    (out / "metrics.json").write_text(json.dumps(record, indent=2, ensure_ascii=False))
    gt_image = colorize_depth(gt_mm, lo, hi) if gt_mm is not None else None
    (out / "report.html").write_text(render_report(record, view_images, gt_image), encoding="utf-8")
    return out, record
