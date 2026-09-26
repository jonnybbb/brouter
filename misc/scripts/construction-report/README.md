# Construction report

The generated entry point is `brouter-core/build/construction-diagnosis/html-report/index.html`. Open it directly in a local browser. Keep `data.js`, `report.js`, `report.css` and `gpx/` beside it. It works offline and makes no map-tile requests.

It includes every case from `final-case-results.csv`: the frozen historical original and an independently regenerated finished route. These are not same-request refinement pairs. Links lead to the earlier 460-pair report and the latest three-cell paired run.

The report uses exact captured integer coordinates, including quarter-metre elevation. Every plotted point is checked against its GPX before generation. The Basel gap and Lozère jump annotations are also asserted against literal frozen segments. Diagnostic native road shapes come from the retained native-graph investigation.

To regenerate the HTML from existing captures (requires Python `markdown`):

```sh
python3 misc/scripts/build-construction-report.py
```

To capture a fresh set, compile core and integration test sources first, then run from the repository root:

```sh
probe_cp=brouter-core/build/classes/java/main:brouter-core/build/classes/java/integrationTest:brouter-core/build/classes/java/test:brouter-mapaccess/build/classes/java/main:brouter-expressions/build/classes/java/main:brouter-codec/build/classes/java/main:brouter-util/build/classes/java/main
probe_dir=brouter-core/build/construction-diagnosis
javac -cp "$probe_cp" -d "$probe_dir" misc/scripts/construction-report/CaptureConstructionRoutes.java
java -Xmx3g -cp "$probe_dir:$probe_cp" btools.router.CaptureConstructionRoutes "$probe_dir/report-capture-new" > "$probe_dir/report-capture-new.log" 2>&1
python3 misc/scripts/build-construction-report.py "$probe_dir/report-capture-new"
```

The collector matches `FinishedLoopPricingTest` request settings. It writes the exact finished GPX, point arrays, ride measurements, repeated continuous prices and generation time. It preserves failures as error files. The generator retains missing captures in the report. The three raw-seed cases here refer to their valid finished routes; invalid raw-seed geometry is not passed off as a finished route.

The first report capture used a provisional distance-error denominator. The builder recomputes this one descriptive field using `2 * pi * searchRadius`, matching `RefineStage`; routing, coordinates and cost were unaffected. The maintained collector uses that same correct denominator. Both the executed and maintained source hashes are retained.

`manifest.json` records source hashes and the complete selection. `render-validation.json` records the offline DOM and SVG construction check. The browser tool blocked local file navigation, so an automated browser screenshot was not obtained.

## Preserved report

The committed copy is `docs/reports/loop-construction/index.html`. It includes all 23 cases and 46 GPX files and survives a clean build. The larger historical archives stay local. Refresh this copy after generating the report:

```sh
python3 misc/scripts/construction-report/preserve-report.py
```
