#!/usr/bin/env python3
"""Build a complete, offline route comparison from retained evidence. Does not run routing."""
import argparse
import collections
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write_js(path, variable, value):
    path.write_text(variable + '=' + json.dumps(value, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c') + ';\n')


def project(points, origin, latitude):
    scale = math.cos(math.radians(latitude)) * .111195
    return [((p[0] - origin[0]) * scale, (p[1] - origin[1]) * .111195) for p in points]


def difference(reference, route, origin, latitude):
    """Highlight literal route portions outside a 10 m corridor using <=25 m samples."""
    if reference == route:
        return [], 0.0
    base = project(reference, origin, latitude)
    path = project(route, origin, latitude)
    grid = collections.defaultdict(list)
    for a, b in zip(base, base[1:]):
        for x in range(math.floor((min(a[0], b[0]) - 10) / 100), math.floor((max(a[0], b[0]) + 10) / 100) + 1):
            for y in range(math.floor((min(a[1], b[1]) - 10) / 100), math.floor((max(a[1], b[1]) + 10) / 100) + 1):
                grid[x, y].append((a, b))
    spans, current, metres = [], [], 0.0
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        count = max(1, math.ceil(length / 25))
        for i in range(count):
            t = (i + .5) / count
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            near = False
            for c, d in grid[math.floor(x / 100), math.floor(y / 100)]:
                dx, dy = d[0] - c[0], d[1] - c[1]
                den = dx * dx + dy * dy
                u = max(0, min(1, ((x - c[0]) * dx + (y - c[1]) * dy) / den)) if den else 0
                if math.hypot(x - c[0] - u * dx, y - c[1] - u * dy) <= 10:
                    near = True
                    break
            if near:
                if current:
                    spans.append(current)
                    current = []
            else:
                lo = [a[j] + i / count * (b[j] - a[j]) for j in (0, 1)]
                hi = [a[j] + (i + 1) / count * (b[j] - a[j]) for j in (0, 1)]
                if not current:
                    current.append(lo)
                current.append(hi)
                metres += length / count
    if current:
        spans.append(current)
    return spans, metres


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment', type=Path)
    parser.add_argument('--archive', type=Path, help='Optional older reports/loops directory')
    args = parser.parse_args()
    root = args.experiment.resolve()
    output = root / 'complete-review'
    (output / 'cases').mkdir(parents=True, exist_ok=True)
    assets = Path(__file__).with_name('refinement-review')
    raw = (root / 'ride-review-data.js').read_text()
    routes = json.loads(raw.removeprefix('window.rideReview=').rstrip(';\n'))
    cells = {c['cell']: c for c in read_csv(root / 'cells.csv')}
    # This report's written interpretation describes this particular frozen experiment.
    # Refuse to carry its conclusions into a different run merely by changing a path.
    if (len(cells), sum(c['applied'] == 'true' for c in cells.values()),
            sum(c['truncated'] == 'true' for c in cells.values())) != (460, 34, 77):
        raise ValueError('The narrative targets the corrected 460/34/77 experiment; update it for a different result.')
    for name in ('index.html', 'review.js', 'review.css'):
        shutil.copy2(assets / name, output / name)
    traces, rejections = collections.defaultdict(list), collections.defaultdict(list)
    for row in read_csv(root / 'evaluations.csv'):
        traces[row['cell']].append(row)
    for row in read_csv(root / 'rejections.csv'):
        rejections[row['cell']].append(row)
    index, audit = [], []
    for number, row in enumerate(sorted(routes, key=lambda r: r['label'])):
        c = cells[row['label']]
        before, after = (row['a'], row['b']) if row['aIsOriginal'] else (row['b'], row['a'])
        identical = before['points'] == after['points']
        origin = before['points'][0][:2]
        latitude = sum(p[1] / 1e6 - 90 for p in before['points'] + after['points']) / (len(before['points']) + len(after['points']))
        removed, removed_m = difference(after['points'], before['points'], origin, latitude)
        added, added_m = difference(before['points'], after['points'], origin, latitude)
        assert identical == (c['baseline_signature'] == c['result_signature'])
        ident = f'pair-{number:03d}'
        summary = dict(id=ident, label=row['label'], region=row['region'], profile=row['profile'],
                       outcome=row['outcome'], identical=identical, gain=row['gain'],
                       requestMs=row['requestMs'], addedMs=row['addedMs'], addedM=added_m, removedM=removed_m,
                       reason=row['reason'], beforeKm=before['metrics']['distance_m'] / 1000,
                       afterKm=after['metrics']['distance_m'] / 1000, source='paired', variants=['Original', 'Result'])
        index.append(summary)
        for route in (before, after):
            route['gpx'] = '../' + route['gpx']
        detail = dict(**summary, before=before, after=after, origin=origin, latitude=latitude,
                      removed=removed, added=added, cell=c, traces=traces[row['label']], rejections=rejections[row['label']])
        write_js(output / 'cases' / (ident + '.js'), 'window.routeCase', detail)
        audit.append(dict(cell=row['label'], identical=identical, added_m=added_m, removed_m=removed_m))
    archives = []
    if args.archive:
        historical = args.archive.resolve()
        combined = historical / 'all-routes.geojson'
        if combined.exists():
            for f in json.loads(combined.read_text())['features']:
                props = f['properties']
                label = props.get('name', 'unnamed').split(' [')[0]
                archives.append((label, props.get('variant', 'unknown'), f, combined))
        for path in sorted((historical / 'tracks').glob('*.geojson')):
            match = re.match(r'(.+?)_(iso_greedy|isochrone|greedy|probe|auto|fast)$', path.stem)
            if not match:
                raise ValueError('Unrecognized historical variant: ' + path.name)
            for feature in json.loads(path.read_text())['features']:
                if feature['geometry']['type'] == 'LineString':
                    archives.append((match[1], match[2], feature, path))
        groups = collections.defaultdict(list)
        for label, variant, feature, path in archives:
            properties = {k: v for k, v in feature['properties'].items() if k != 'messages'}
            groups[label].append(dict(variant=variant, geometry=feature['geometry'], properties=properties,
                                     sourceFile=os.path.relpath(path, output), sourceMtime=path.stat().st_mtime))
        for number, (label, variants) in enumerate(sorted(groups.items())):
            ident = f'archive-{number:03d}'
            region = re.split(r'_\d+km_', label)[0]
            profile = 'fastbike.brf' if '_fastbike_' in label else 'gravel.brf' if '_gravel_' in label else 'other'
            summary = dict(id=ident, label=label, region=region, profile=profile, source='archive',
                           variants=[v['variant'] for v in variants], outcome='historical')
            index.append(summary)
            write_js(output / 'cases' / (ident + '.js'), 'window.routeCase', dict(**summary, recordings=variants))
    write_js(output / 'catalog.js', 'window.routeCatalog', index)
    manifest = dict(experiment=str(root), pairedCases=len(routes), pairedVariants=2 * len(routes),
                    identical=sum(a['identical'] for a in audit), changed=sum(not a['identical'] for a in audit),
                    historicalRecordings=len(archives), historicalCases=sum(x['source'] == 'archive' for x in index),
                    comparison='Original versus published result of one request, after correctness repairs. No before-fix route-quality experiment.',
                    difference='Literal geometry outside a 10 m undirected corridor, sampled at <=25 m spacing. Not edge identity or direction-aware.',
                    sha256={name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in
                            ['cells.csv', 'ride-review-data.js', 'evaluations.csv', 'rejections.csv', 'manifest.properties']})
    manifest['historicalSourcesSha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in sorted({a[3] for a in archives})}
    manifest['generatorSha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    manifest['templateSha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(assets.iterdir()) if p.is_file()}
    (output / 'report-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / 'geometry-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))
    print(output / 'index.html')


if __name__ == '__main__':
    main()
