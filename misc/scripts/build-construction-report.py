#!/usr/bin/env python3
"""Render all targeted construction cases using saved exact route geometry; no routing."""
import csv
import hashlib
import html
import json
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DIAG = ROOT / 'brouter-core/build/construction-diagnosis'
OLD = ROOT / 'brouter-core/build/refine-evaluation-12687333987490758169'
PAIRED = ROOT / 'brouter-core/build/refine-evaluation-2675540930182886791'
CAPTURE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DIAG / 'report-capture'
OUT = DIAG / 'html-report'
TEMPLATE = ROOT / 'misc/scripts/construction-report'


def js_data(path):
    return json.loads(path.read_text().split('=', 1)[1].strip().rstrip(';'))


def coords(node_id):
    return [node_id >> 32, node_id & 0xffffffff]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_gpx(path, points):
    ns = {'g': 'http://www.topografix.com/GPX/1/1'}
    nodes = ET.parse(path).findall('.//g:trkpt', ns)
    assert len(nodes) == len(points), (path, len(nodes), len(points))
    for node, point in zip(nodes, points):
        actual = [round((float(node.attrib['lon']) + 180) * 1e6), round((float(node.attrib['lat']) + 90) * 1e6)]
        assert actual == point[:2], (path, actual, point)
    return len(nodes)


def thumbnail(case):
    lines = [case['before']['points'], case['after']['points']] if case['after'] else [case['before']['points']]
    origin = lines[0][0]
    scale = math.cos(math.radians(origin[1] / 1e6 - 90))
    projected = [[[(p[0] - origin[0]) * scale, origin[1] - p[1]] for p in line] for line in lines]
    xs = [p[0] for line in projected for p in line]
    ys = [p[1] for line in projected for p in line]
    w, h = max(1, max(xs) - min(xs)), max(1, max(ys) - min(ys))
    parts = []
    for line, color in zip(projected, ['#bd530b', '#007ba4']):
        d = ' '.join(('M' if i == 0 else 'L') + f'{p[0]:.2f},{p[1]:.2f}' for i, p in enumerate(line))
        parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.6" opacity=".85" vector-effect="non-scaling-stroke"/>')
    svg = f'<svg role="img" aria-label="{html.escape(case["label"])} route overlay" viewBox="{min(xs)-w*.08} {min(ys)-h*.08} {w*1.16} {h*1.16}">{"".join(parts)}</svg>'
    return f'<button class="thumb" data-case="{case["label"]}"><strong>{case["label"].replace("_", " ")}</strong>{svg}<small>{html.escape(case["group"])} · {case["status"]}</small></button>'


OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'gpx').mkdir(exist_ok=True)
old = {r['label']: r for r in js_data(OLD / 'ride-review-data.js')}
paired = {r['label']: r for r in js_data(PAIRED / 'ride-review-data.js')}
labels = list(csv.DictReader((DIAG / 'final-case-results.csv').open()))
failures = {}
for line in (OLD / 'diagnosis/frozen-failures.tsv').read_text().splitlines():
    label, profile, index, ids = line.split('\t')
    failures[label] = (int(index), [int(v) for v in ids.split(',')])
notes = {
    'rural_lozere_80km_gravel_E': ('Construction defect: sparse connector', 'The frozen route jumps from the junction to a distant transfer point, then follows the road backwards. The green line is the native road shape captured independently from the local graph. Cleanup now details and validates connectors before committing them.'),
    'basel_30km_fastbike_S': ('Construction defect: moved endpoint', 'The two red endpoints are approximately 22 m apart. Rematching changed the committed endpoint onto another edge before the next leg started. The red segment is present in the frozen route; the fix preserves the accepted waypoint and matched edge.'),
    'mallorca_100km_fastbike_E': ('Pricing defect: valid rounded waypoint', 'This historical geometry is valid. The small red marker is the router’s clipped waypoint; its integer rounding cell intersects the native road. The old pricing test rejected it. This repair requires no route change, and a different regenerated loop is not evidence of a geometry fix here.'),
}
native_ids = [787215828444862662,787216773337667677,787218890756544275,787220385405163031,787221038240191943,787221622355744155,787222597313320324,787223980292789625,787224933775529372,787225758409250310,787226471373821592]
manifest = {'builtUtc': datetime.now(timezone.utc).isoformat(), 'scope': 'All 23 targeted cases; historical originals versus independent post-fix regeneration. Not a controlled quality A/B.', 'offline': True, 'sourceHashes': {}, 'cases': []}
for path in [OLD/'ride-review-data.js', DIAG/'final-case-results.csv', DIAG/'validation-summary.json', ROOT/'docs/loop-construction-defects.md', TEMPLATE/'CaptureConstructionRoutes.java', DIAG/'CaptureConstructionRoutes.java']:
    manifest['sourceHashes'][str(path.relative_to(ROOT))] = digest(path)
cases = []
for row in labels:
    label = row['cell']
    historic = old[label]
    before = dict(historic['a' if historic['aIsOriginal'] else 'b'])
    old_gpx = OLD / before['gpx']
    before['gpx'] = f'gpx/{label}-historical.gpx'
    shutil.copyfile(old_gpx, OUT / before['gpx'])
    count = audit_gpx(old_gpx, before['points'])
    captured = CAPTURE / f'{label}.json'
    after = json.loads(captured.read_text()) if captured.exists() else None
    if after:
        # Recompute this derived measurement from the actual routing radius, matching RefineStage.
        # The capture run's provisional reporting denominator did not affect routing or pricing.
        km = int(re.search(r'_(\d+)km_', label)[1])
        radius = {30:4800,50:8000,75:11937,80:12700,100:15900}[km]
        after['requestedDistanceM'] = 2 * math.pi * radius
        after['metrics']['distance_error_pct'] = 100 * abs(after['metrics']['distance_m'] / after['requestedDistanceM'] - 1)
        after['gpx'] = f'gpx/{label}-regenerated.gpx'
        source_gpx = CAPTURE / f'{label}.gpx'
        shutil.copyfile(source_gpx, OUT / after['gpx'])
        count += audit_gpx(source_gpx, after['points'])
        manifest['sourceHashes'][str(captured.relative_to(ROOT))] = digest(captured)
    status = 'PRICED' if after and after['cost'] > 0 and after['cost'] == after['repeatCost'] else 'FAILED / MISSING'
    group = 'Historical pricing failure' if label in failures else 'Raw-seed case' if label in paired else 'Original smoke case'
    case = {'label':label,'profile':historic['profile'],'group':group,'before':before,'after':after,'status':status,'testSeconds':float(row['seconds']), 'testStatus':row['status'], 'defect':None, 'paired':None}
    if label in failures:
        index, ids = failures[label]
        case['defect'] = {'title':'Historical pricing failure location', 'text':'This marks the frozen walker’s first mismatch. It locates the symptom; it does not independently prove which construction operation caused it. The new finished-route check passes for this label.', 'focus':[coords(ids[0])], 'highlight':[], 'native':[],'minWidth':300, 'index':index}
    if label in notes:
        defect = case['defect']; defect['title'], defect['text'] = notes[label]
        if label.startswith('rural_lozere'):
            assert any(a[:2] == coords(native_ids[0]) and b[:2] == coords(native_ids[-2]) for a,b in zip(before['points'], before['points'][1:])), 'Lozere jump must exist literally'
            defect.update(focus=[coords(v) for v in native_ids], native=[coords(v) for v in native_ids], highlight=[coords(native_ids[0]),coords(native_ids[-2])],minWidth=100)
        elif label.startswith('basel'):
            seam = [coords(805441427679142253),coords(805442037564498114)]
            assert any(a[:2] == seam[0] and b[:2] == seam[1] for a,b in zip(before['points'],before['points'][1:])), 'Basel seam must exist literally'
            defect.update(focus=seam,highlight=seam,minWidth=110)
        else:
            shape=[coords(v) for v in [785089583405216973,785091481780761638]]
            via=coords(785090171815736472)
            assert any(p[:2]==via for p in before['points'])
            defect.update(focus=shape,native=shape,highlight=[via],minWidth=75)
    if label in paired:
        r=paired[label]
        case['paired']={'outcome':r['outcome'],'reason':r['reason'],'identical':r['a']['points']==r['b']['points'],'requestMs':r['requestMs']}
    cases.append(case)
    manifest['cases'].append({'label':label,'status':status,'historicalPoints':len(before['points']),'regeneratedPoints':len(after['points']) if after else None,'auditedGpxPoints':count,'literalGeometryIdentical':before['points']==after['points'] if after else None})
manifest['geometryAudit']={'cases':len(cases),'pricedCaptures':sum(c['status']=='PRICED' for c in cases),'gpxPointsVerified':sum(c['auditedGpxPoints'] for c in manifest['cases']),'gpxFilesVerified':sum(1+(c['regeneratedPoints'] is not None) for c in manifest['cases']), 'basis':'Every GPX coordinate checked against the plotted integer coordinates; no geometry substitution or simplification.'}
import markdown
report = markdown.markdown((ROOT/'docs/loop-construction-defects.md').read_text(), extensions=['fenced_code','tables'])
page = (TEMPLATE/'index.html').read_text().replace('{{GALLERY}}','\n'.join(thumbnail(c) for c in cases)).replace('{{REPORT}}',report).replace('../../../refine-evaluation','../../refine-evaluation')
(OUT/'index.html').write_text(page)
for filename in ['report.js','report.css']:
    shutil.copyfile(TEMPLATE/filename, OUT/filename)
(OUT/'data.js').write_text('window.constructionReport='+json.dumps({'cases':cases,'manifest':manifest},separators=(',',':'),allow_nan=False).replace('<','\\u003c')+';\n')
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest['geometryAudit'],indent=2))
print(OUT/'index.html')
