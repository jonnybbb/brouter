#!/usr/bin/env python3
"""
Generate static HTML+Leaflet visualizations of failed round-trip routes
from brouter-core/build/reports/loops/.results/*.json.

Each failure case gets one HTML file with:
- The rejected route drawn as a polyline on OSM tiles
- Start point marker
- Failure reason annotation
- A brief diagnosis hint based on error pattern

Skips "could-not-build" cases (no coordinates).

Output: docs/features/visual-analysis/failures-greedy-fastbike/
"""

import json
import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = PROJECT_ROOT / "brouter-core" / "build" / "reports" / "loops" / ".results"
OUT_DIR = PROJECT_ROOT / "docs" / "features" / "visual-analysis" / "failures-greedy-fastbike"

HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
  <title>{label}</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    body {{ font-family: -apple-system, system-ui, sans-serif; margin: 0; padding: 0; }}
    #header {{ padding: 10px 15px; background: #fff8e1; border-bottom: 2px solid #f57c00; }}
    #header h1 {{ margin: 0; font-size: 18px; }}
    #header .meta {{ font-size: 12px; color: #555; margin-top: 4px; }}
    #header .reason {{ color: #c62828; font-weight: 600; margin-top: 6px; }}
    #header .diagnosis {{ font-style: italic; color: #1565c0; margin-top: 4px; }}
    #map {{ height: calc(100vh - 100px); }}
  </style>
</head>
<body>
  <div id="header">
    <h1>{label}</h1>
    <div class="meta">{region} · {distance_km} km · {profile} · dir={direction}° · variant={variant}</div>
    <div class="reason">FAIL: {reason}</div>
    <div class="diagnosis">Diagnosis hint: {diagnosis}</div>
  </div>
  <div id="map"></div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const coords = {coords};
    const start = coords[0];
    const map = L.map('map').setView([start[1], start[0]], 11);
    L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
      attribution: '&copy; OpenStreetMap contributors',
      maxZoom: 18,
    }}).addTo(map);
    const latlngs = coords.map(c => [c[1], c[0]]);
    const polyline = L.polyline(latlngs, {{
      color: '#c62828', weight: 3, opacity: 0.75
    }}).addTo(map);
    L.marker([start[1], start[0]]).addTo(map).bindPopup('Start');
    map.fitBounds(polyline.getBounds(), {{ padding: [20, 20] }});
  </script>
</body>
</html>
"""

INDEX_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
  <title>Failed greedy/fastbike routes — visual analysis</title>
  <style>
    body {{ font-family: -apple-system, system-ui, sans-serif; max-width: 1100px;
            margin: 30px auto; padding: 0 20px; }}
    h1 {{ color: #c62828; }}
    .stats {{ background: #f5f5f5; padding: 10px 15px; border-radius: 4px; margin: 15px 0; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ padding: 8px 12px; text-align: left; border-bottom: 1px solid #ddd; }}
    th {{ background: #fafafa; }}
    tr:hover {{ background: #f9f9f9; }}
    a {{ color: #1565c0; text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    .reason-chaos {{ color: #c62828; }}
    .reason-hostile {{ color: #ef6c00; }}
    .reason-could-not-build {{ color: #757575; }}
    .reason-other {{ color: #555; }}
  </style>
</head>
<body>
  <h1>Failed greedy/fastbike routes — visual analysis</h1>
  <div class="stats">
    <strong>{total} failed cases</strong> with coordinates ({with_coords} visualizable).
    Could-not-build cases ({no_coords}) skipped — no route geometry to draw.
  </div>
  <p>Click any row to open the route visualization.</p>
  <table>
    <thead>
      <tr><th>Region</th><th>Distance</th><th>Direction</th><th>Profile</th>
          <th>Variant</th><th>Failure</th><th>Diagnosis</th></tr>
    </thead>
    <tbody>
      {rows}
    </tbody>
  </table>
</body>
</html>
"""

def categorize_failure(err):
    """Extract a short failure category + diagnosis hint from the error message."""
    if not err:
        return ("none", "no failure", "passed")
    if "could not build" in err:
        return ("could-not-build",
                "candidate generation exhausted",
                "Profile too restrictive for sparse network (sparse rural)")
    m = re.search(r'(\d+) self-intersections', err)
    if m:
        n = int(m.group(1))
        if n >= 21:
            return ("chaos",
                    f"{n}+ self-intersections (clamped)",
                    "Likely terrain-constrained zigzagging or direction-mismatch")
        return ("chaos",
                f"{n} self-intersections",
                "Borderline chaotic — close to the 5 threshold")
    m = re.search(r'contiguous (\d+)m of profile-hostile', err)
    if m:
        n = int(m.group(1))
        return ("hostile",
                f"hostile-stretch {n}m",
                "Off-road stretch (track/path) above 1500m cap" +
                (" — borderline" if n < 2000 else ""))
    if "tolerance" in err or "distance" in err and "best error" in err:
        return ("distance", "distance tolerance", "Route overshoots/undershoots target")
    return ("other", err[:50], "(needs manual inspection)")

def label_to_filename(label):
    return re.sub(r'[^a-z0-9]+', '_', label.lower()).strip('_')

def main():
    if not RESULTS_DIR.exists():
        print(f"ERROR: {RESULTS_DIR} not found", file=sys.stderr)
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    cases = []
    for fp in sorted(RESULTS_DIR.glob("*__fastbike__*__greedy.json")):
        try:
            d = json.loads(fp.read_text())
        except Exception:
            continue
        if not d.get("error"):
            continue
        cat, summary, hint = categorize_failure(d["error"])
        coords = d.get("coordinates") or []
        cases.append({
            "label": d["label"],
            "region": d["region"],
            "distance_km": int(d["distanceMeters"] / 1000),
            "direction": int(d["direction"]),
            "profile": d["profileName"],
            "variant": d["variant"],
            "category": cat,
            "summary": summary,
            "hint": hint,
            "error": d["error"],
            "coords": coords,
        })

    with_coords = [c for c in cases if c["coords"]]
    no_coords = [c for c in cases if not c["coords"]]

    # Per-case HTML
    for c in with_coords:
        filename = label_to_filename(c["label"]) + ".html"
        out = OUT_DIR / filename
        html = HTML_TEMPLATE.format(
            label=c["label"],
            region=c["region"],
            distance_km=c["distance_km"],
            profile=c["profile"],
            direction=c["direction"],
            variant=c["variant"],
            reason=c["error"][:300],
            diagnosis=c["hint"],
            coords=json.dumps(c["coords"]),
        )
        out.write_text(html)

    # Index page
    rows = []
    for c in cases:
        cls = f"reason-{c['category']}"
        link_target = label_to_filename(c["label"]) + ".html" if c["coords"] else None
        cell0 = f'<a href="{link_target}">{c["region"]}</a>' if link_target else c["region"]
        rows.append(
            f'<tr>'
            f'<td>{cell0}</td>'
            f'<td>{c["distance_km"]}km</td>'
            f'<td>{c["direction"]}°</td>'
            f'<td>{c["profile"]}</td>'
            f'<td>{c["variant"]}</td>'
            f'<td class="{cls}">{c["summary"]}</td>'
            f'<td>{c["hint"]}</td>'
            f'</tr>'
        )

    index_html = INDEX_TEMPLATE.format(
        total=len(cases),
        with_coords=len(with_coords),
        no_coords=len(no_coords),
        rows="\n".join(rows),
    )
    (OUT_DIR / "index.html").write_text(index_html)

    print(f"Generated {len(with_coords)} per-case HTML files.")
    print(f"Skipped {len(no_coords)} could-not-build cases (no coordinates).")
    print(f"Open: {OUT_DIR / 'index.html'}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
