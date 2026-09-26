# Descriptive analysis of generated rides

Implemented and validated on 2026-09-26. `RouteRideAnalysis.measure(track, requestedDistanceMeters)` returns an immutable map of 59 numeric measurements plus analysis duration. It consumes the final in-memory `OsmTrack` and does not call the router, change preferences, assign an enjoyment score or influence publication.

The paired evaluation captures it before and after refinement in the same request. The measurements therefore remain useful when the continuous-cost oracle cannot price a route.

## Use

```java
RouteRideAnalysis analysis = RouteRideAnalysis.measure(finishedTrack, 50000);
double knownSurfacePercent = analysis.value("surface_known_pct");
double longestFirmUnpavedMeters = analysis.value("longest_firm_unpaved_m");
Map<String, Double> measurements = analysis.values();
```

Names ending in `_m` use metres, `_pct` use percentages, and `_seconds` use seconds. Encounter and bend counts have explicit count names. `Double.NaN` means unavailable. CSV leaves unavailable values blank; HTML renders them as `unavailable`. A missing or fewer-than-two-point track yields an empty map.

For measurement runs, supply the existing profile override `profile:processUnusedTags=1`, or set `RoutingContext.keyValues["processUnusedTags"]` to `"1"` before routing. BRouter otherwise filters way attributes that the profile does not use, including those in disabled preference branches. This can hide traffic or scenery attributes even when the tile contains them. The paired runner sets the override and records it in its manifest. It increases tag processing work, which is included in request timing. A regression test verifies unchanged greedy geometry/cost and that the setting does not leak into a subsequent request.

Ordinary generated GPX geometry cannot recover road attributes that were never exported. These measurements use the detailed in-memory track. Imported tracks or old messages without segment provenance receive unknown metadata coverage.

## Measurement definitions

| Group | Measurements and meaning |
|---|---|
| Geometry and request fit | `distance_m` sums the final coordinates using `CheapRuler`, excluding duplicate-point segments. `stored_distance_m` exposes the existing track field separately. `distance_error_pct` compares geometric distance with the requested total; absent/nonpositive targets are unavailable. |
| Metadata coverage | `metadata_verified_m` and `_pct` count surviving segments whose endpoints match their original detailed message. `metadata_unknown_m` is the remainder. Verified means attribution to geometry, not verified real-world accuracy. |
| Surface | Distance explicitly tagged as paved, firm unpaved, other unpaved or other recognized surface. Firm unpaved means `fine_gravel` or `compacted`. Paved includes cobblestone and paving stones, so it does not imply smoothness. Missing/unrecognized surface values are unknown. There is no fallback from road class or `cycleway:surface` to the ridden segment's surface. |
| Smoothness | Known smoothness coverage and distance tagged `bad`, `very_bad`, `horrible`, `very_horrible` or `impassable`. This is a tag observation, not a bike-specific suitability decision. |
| Traffic estimates | Coverage for explicit estimated classes 1–7; low exposure uses 1–2 and high exposure uses 4–7. Class 3 is known but neither low nor high. These are descriptive bins, not Level of Traffic Stress or collision-risk estimates. Unclassified roads are unknown. Arterial distance is reported independently using highway tags. |
| Continuous sections | Longest paved, firm-unpaved, low-traffic-estimate and high-traffic-estimate runs. An unknown or nonqualifying segment ends a run. A continuous surface run may contain intersections; this is not a guarantee of uninterrupted pedalling. |
| Scenery estimates | Forest and river attribute coverage for classes 1–6, with exposure to classes 4–6 reported separately. These do not establish actual visibility or scenic quality. |
| Mapped interruptions and access | Encounters with explicitly tagged signals, stop signs and barriers; tagged steps distance; explicit `bicycle=dismount`, `bicycle=no` or `bicycle=private` distance. These are observations only. They do not resolve inherited access rules, exceptions, current closures, all turn restrictions or legal access. Repeated visits count as repeated encounters. Missing node tags cannot establish the absence of an interruption. |
| Model timing | Original per-segment kinematic time is stored during detail routing. `partial_model_moving_seconds` sums positive finite estimates for verified surviving segments. Coverage is reported; `model_moving_seconds` is available only when all positive-length geometry is covered. This does not include stop delays or rerun the physics after cleanup. Surviving estimates can still reflect their original routing context. |
| Elevation | Known-elevation distance, then nonoverlapping 100 m windows with linearly interpolated endpoints. Report ascent, descent, maximum window grades and window distance at grades of at least +8% or at most −8%. Missing elevation breaks the run; incomplete tails are excluded and coverage is reported. These are coarse window measurements, not total unsmoothed ascent or a calibrated effort model. |
| Late climbing | Ascent within the last quarter of geometric distance, apportioned across the overlapping part of each complete 100 m window. Partial tail windows remain excluded. |
| Shape and repetition | Geometry bends of at least 90° and U-turn-like bends of at least 150°. These depend on geometry sampling and are not junction counts. Existing `reuseStemSplit` supplies shared-access and interior reuse using its identity/corridor geometry approximation. Nearby parallel roads can be ambiguous. Existing snapshot self-intersection counts remain in the paired report. |

For surface, traffic, scenery, smoothness and highway-derived exposure, zero data coverage makes dependent measurements unavailable. Partial coverage produces observed distances only; it must not be interpreted as a full-route absence of exposure. No aggregate preference score is calculated.

The bins and elevation window are documented version-1 analysis definitions, not new rider request parameters. Their suitability needs cyclist feedback before they become optimization targets. Rider-specific roughness limits, a combined stress model, actual scenic views, resupply availability and requested duration/climbing targets remain separate work.

## Metadata provenance

`MessageData` retains the original directed segment endpoints and a segment-local kinematic estimate when `OsmPath` records detailed geometry. Cloning retains these values. Cleanup may repair origin pointers and recalculate distance, but cannot rewrite this provenance.

The analyser only uses a message's road/node attributes and timing when the original endpoints match the current incoming segment. New joins, reversed/unmatched geometry, raw tracks and unsupported metadata become unknown. The route-analysis pass never mutates the track. This avoids assigning a removed segment's traffic, surface or time to a replacement connection.

The added provenance is in-memory route metadata; it does not change `.rd5` or GPX formats. Existing routing-cost calculations are unchanged.

## Paired reports

Every evaluation directory now also contains:

- `ride-analysis.csv`: one row per metric, stage and cell, with profile, geometry fingerprint, analysis version and analysis time.
- `ride-analysis.html`: a self-contained before/after table for each cell, including unavailable values and interpretation notes.

Original measurements are captured before speculative refinement, so later changes to the track cannot rewrite them. The existing `cells.csv` and acceptance scorecard remain separate. Reports do not label lower values universally better.

Reproduce the six-cell validation from the repository root:

```sh
./gradlew :brouter-core:integrationTest \
  --tests 'btools.router.RefineEvaluationReportTest' \
  --tests 'btools.router.FullABEvaluationMatrixTest' \
  -Dloop.forks=1 -Dloop.segments.nodownload=true \
  -Dloop.refine.cells=basel_30km_gravel_N,basel_30km_fastbike_N,mallorca_30km_gravel_N,mallorca_30km_fastbike_N,dreieich_30km_gravel_E,dreieich_30km_fastbike_E
```

## Verified results

The final validation passed 632 core tests and 7 integration tests, including the six-cell matrix. Checkstyle and PMD passed for core main, test and integrationTest sources. Tests cover attribution after a cleanup join, missing data, exact tag matching, immutable results, duplicate points, elevation gaps and sampling, report escaping, blank unavailable values, unchanged greedy output and profile-cache isolation.

On the six approximately 30 km routes, analysis of the 12 before/after snapshots took a median of **2.68 ms** and a maximum of **5.42 ms**. This is a small local sample, not a throughput guarantee or a benchmark of running the whole evaluation. Analysis time includes the existing geometry-based reuse calculation, but excludes routing, input hashing, report writing and the other snapshot metrics.

| Cell | Segment attribution % | Known surface % | Traffic-estimate coverage % | Forest-estimate coverage % | River-estimate coverage % |
|---|---:|---:|---:|---:|---:|
| Dreieich gravel east | 99.83 | 71.68 | 1.38 | 66.73 | 11.80 |
| Dreieich fastbike east | 99.77 | 96.47 | 85.78 | 49.16 | 6.12 |
| Mallorca gravel north | 99.98 | 89.20 | 23.54 | 41.99 | 12.98 |
| Mallorca fastbike north | 99.04 | 95.39 | 74.79 | 34.12 | 3.57 |
| Basel gravel north | 99.23 | 75.07 | 16.76 | 35.34 | 61.92 |
| Basel fastbike north | 99.51 | 76.78 | 52.67 | 21.82 | 34.81 |

Different profiles traverse different roads, so these coverage figures are not a controlled comparison of profile settings. An earlier run without full tag collection omitted all forest and river attributes and all gravel traffic attributes. The final runner explicitly preserves those attributes.

All six requests still skipped refinement as `baseline_unpriceable`, with unchanged before/after geometry. The new measurements work independently of that blocker. They do not demonstrate an optimizer win or cyclist preference.

Local artifacts: [HTML report](../brouter-core/build/refine-evaluation-10930795172244170190/ride-analysis.html), [CSV measurements](../brouter-core/build/refine-evaluation-10930795172244170190/ride-analysis.csv), [input manifest](../brouter-core/build/refine-evaluation-10930795172244170190/manifest.properties), [validation log](../brouter-core/build/reports/refinement/ride-analysis-validation.log). A clean build removes these artifacts.
