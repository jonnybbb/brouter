#!/usr/bin/env python3
"""Summarize frozen paired route evidence without altering the experiment's acceptance rules."""
import argparse
import collections
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


def distribution(values):
    values = sorted(values)
    if not values:
        return {"n": 0, "median": None, "mean": None, "p90": None, "max": None}
    return {"n": len(values), "median": statistics.median(values),
            "mean": statistics.mean(values), "p90": values[math.ceil(.9 * len(values)) - 1], "max": values[-1]}


def outside_distance(reference, route):
    """Metres outside a 10 m geometry corridor, sampled at <=25 m spacing; not edge identity."""
    if reference == route:
        return 0.0
    latitude = statistics.mean(p[1] / 1e6 - 90 for p in reference + route)
    xscale = 111195 * math.cos(math.radians(latitude)) / 1e6
    yscale = 111195 / 1e6
    ox, oy = reference[0][:2]
    project = lambda p: ((p[0] - ox) * xscale, (p[1] - oy) * yscale)
    base = [project(p) for p in reference]
    grid = collections.defaultdict(list)
    for a, b in zip(base, base[1:]):
        for x in range(math.floor((min(a[0], b[0]) - 10) / 100), math.floor((max(a[0], b[0]) + 10) / 100) + 1):
            for y in range(math.floor((min(a[1], b[1]) - 10) / 100), math.floor((max(a[1], b[1]) + 10) / 100) + 1):
                grid[x, y].append((a, b))
    outside = 0.0
    path = [project(p) for p in route]
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        count = max(1, math.ceil(length / 25))
        for i in range(count):
            t = (i + .5) / count
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            nearby = False
            for c, d in grid[math.floor(x / 100), math.floor(y / 100)]:
                dx, dy = d[0] - c[0], d[1] - c[1]
                squared = dx * dx + dy * dy
                u = max(0, min(1, ((x - c[0]) * dx + (y - c[1]) * dy) / squared)) if squared else 0
                if math.hypot(x - c[0] - u * dx, y - c[1] - u * dy) <= 10:
                    nearby = True
                    break
            if not nearby:
                outside += length / count
    return outside


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.experiment
    output = args.output or root / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    cells = list(csv.DictReader((root / "cells.csv").open()))
    metrics = {}
    for row in csv.DictReader((root / "ride-analysis.csv").open()):
        metrics.setdefault(row["cell"], {}).setdefault(row["stage"], {})[row["metric"]] = float(row["value"]) if row["value"] else None
    text = (root / "ride-review-data.js").read_text()
    routes = {r["label"]: r for r in json.loads(text.removeprefix("window.rideReview=").rstrip(";\n"))}
    summaries = {}
    rows = []
    for profile in sorted({c["profile"] for c in cells}):
        group = [c for c in cells if c["profile"] == profile]
        wins = [c for c in group if c["applied"] == "true"]
        attempted = [c for c in group if c["baseline_pricing_failure"] != "not_attempted"]
        summary = {"cells": len(group), "applied": len(wins),
                   "geometry_changed": sum(c["baseline_signature"] != c["result_signature"] for c in wins),
                   "failed_requests": sum(c["failed"] == "true" for c in group),
                   "truncated": sum(c["truncated"] == "true" for c in group),
                   "pricing_attempted": len(attempted),
                   "priced": sum(float(c["oracle_before"]) > 0 for c in group),
                   "reasons": dict(collections.Counter(c["reason"].split(":")[0] for c in group)),
                   "gain_all_pct": distribution([float(c["official_gain_pct"]) for c in group]),
                   "gain_applied_pct": distribution([float(c["official_gain_pct"]) for c in wins]),
                   "request_ms": distribution([int(c["request_ms"]) for c in group]),
                   "added_ms": distribution([int(c["added_ms"]) for c in group]),
                   "proposals": sum(int(c["proposals"]) for c in group),
                   "evaluations": sum(int(c["evaluations"]) for c in group),
                   "finalizations": sum(int(c["finalizations"]) for c in group)}
        for selection, selected in [("all", group), ("applied", wins)]:
            for name in sorted({name for c in selected for stage in metrics.get(c["cell"], {}).values() for name in stage}):
                pairs = [(metrics[c["cell"]]["before"].get(name), metrics[c["cell"]]["after"].get(name))
                         for c in selected if "before" in metrics.get(c["cell"], {}) and "after" in metrics[c["cell"]]]
                valid = [(a, b) for a, b in pairs if a is not None and b is not None]
                deltas = [b - a for a, b in valid]
                rows.append({"profile": profile, "selection": selection, "metric": name,
                             "selected_cells": len(selected), "paired_available": len(valid),
                             "before_median": statistics.median(a for a, b in valid) if valid else "",
                             "after_median": statistics.median(b for a, b in valid) if valid else "",
                             "paired_delta_median": statistics.median(deltas) if valid else "",
                             "paired_delta_mean": statistics.mean(deltas) if valid else "",
                             "decreased": sum(d < -1e-6 for d in deltas), "increased": sum(d > 1e-6 for d in deltas),
                             "unchanged": sum(abs(d) <= 1e-6 for d in deltas)})
        summaries[profile] = summary
    with (output / "ride-deltas.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    changes = []
    for cell in cells:
        if cell["applied"] != "true":
            continue
        route = routes[cell["cell"]]
        before, after = (route["a"], route["b"]) if route["aIsOriginal"] else (route["b"], route["a"])
        changes.append({"cell": cell["cell"], "profile": cell["profile"], "gain_pct": float(cell["official_gain_pct"]),
                        "before_distance_m": before["metrics"]["distance_m"], "after_distance_m": after["metrics"]["distance_m"],
                        "after_outside_before_10m_m": outside_distance(before["points"], after["points"]),
                        "before_outside_after_10m_m": outside_distance(after["points"], before["points"])})
    if changes:
        with (output / "accepted-geometry-changes.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(changes[0]))
            writer.writeheader()
            writer.writerows(changes)
    counts = collections.Counter()
    for row in csv.DictReader((root / "rejections.csv").open()):
        counts[row["reason"]] += int(row["count"])
    evidence = {"experiment": str(root.resolve()), "profiles": summaries, "candidate_rejections": dict(counts),
                "geometry_comparison": "Midpoint samples spaced at most 25 m; 10 m undirected corridor; approximation, not physical edge identity.",
                "sha256": {f: hashlib.sha256((root / f).read_bytes()).hexdigest()
                           for f in ["cells.csv", "ride-analysis.csv", "ride-review-data.js", "rejections.csv", "manifest.properties"]},
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (output / "summary.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(output)
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
