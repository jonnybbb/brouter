#!/usr/bin/env python3
"""Preserve the rendered 23-case report in docs, independent of ignored build output."""
import hashlib
import json
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[3]
source = root / 'brouter-core/build/construction-diagnosis/html-report'
destination = root / 'docs/reports/loop-construction'
destination.mkdir(parents=True, exist_ok=True)
for name in ['data.js', 'report.js', 'report.css', 'manifest.json', 'render-validation.json']:
    shutil.copyfile(source / name, destination / name)
shutil.copytree(source / 'gpx', destination / 'gpx', dirs_exist_ok=True)
for name in ['final-case-results.csv', 'validation-summary.json', 'report-capture.log']:
    shutil.copyfile(source.parent / name, destination / name)
# Normalize generated XML whitespace and CSV line endings for stable Git storage.
for path in list((destination / 'gpx').glob('*.gpx')) + [destination / 'final-case-results.csv']:
    path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines()) + '\n')
page = (source / 'index.html').read_text()
page = page.replace('<a href="../../refine-evaluation-12687333987490758169/complete-review/index.html">Historical 460-pair report</a>',
                    '<span>Historical 460-pair maps: retained in local build archive</span>')
page = page.replace('<a href="../../refine-evaluation-2675540930182886791/ride-review.html">Latest 3 paired refinement cases</a>',
                    '<span>Latest 3 paired cases: unchanged; summarized in this report</span>')
for name in ['final-case-results.csv', 'validation-summary.json', 'report-capture.log']:
    page = page.replace('href="../' + name + '"', 'href="' + name + '"')
(destination / 'index.html').write_text(page)
files = sorted(p for p in destination.rglob('*') if p.is_file() and p.name != 'bundle-manifest.json')
(destination / 'bundle-manifest.json').write_text(json.dumps({
    'source': str(source.relative_to(root)),
    'scope': 'All 23 targeted cases and 46 exact GPX variants. Larger historical archives remain local.',
    'sha256': {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
}, indent=2) + '\n')
print(destination / 'index.html')
