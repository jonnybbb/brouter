# Loop refinement: morning decision

2026-09-26. **Keep refinement off by default.** The corrected pipeline found 34 accepted route changes out of 460 requests (7.4%), with no measured publication regressions. The changes alter kilometres of route geometry and sometimes improve cyclist-relevant properties. They do not establish that cyclists prefer the routes. Median improvement across all requests is zero, and 77 searches (16.7%) were truncated. The existing product acceptance bar failed.

The original six pricing failures are repaired and covered by passing production regressions. Wider evaluation exposed 14 additional finished-route pricing failures and three raw-search pricing failures. These remain explicit limitations, with frozen geometry, minimal reproductions and graph diagnostics. This is a working comparison pipeline and a decision against enabling the current refinement, not a claim of complete pricing coverage.

## Read or review the evidence

[Complete route report with highlighted differences and all saved variants](../brouter-core/build/refine-evaluation-12687333987490758169/complete-review/index.html). [Plain-language explanation of what improved](loop-refinement-what-improved.md).

- [Blind route review: maps, elevation, measurements and GPX](../brouter-core/build/refine-evaluation-12687333987490758169/ride-review.html)
- [Complete matrix summary](../brouter-core/build/refine-evaluation-12687333987490758169/summary.md), [raw cells](../brouter-core/build/refine-evaluation-12687333987490758169/cells.csv), [analysis](../brouter-core/build/refine-evaluation-12687333987490758169/analysis/summary.json)
- [Remaining failures and reproductions](../brouter-core/build/refine-evaluation-12687333987490758169/diagnosis/README.md)
- [Frozen manifest](../brouter-core/build/refine-evaluation-12687333987490758169/manifest.properties), [input archive](../brouter-core/build/refine-evaluation-12687333987490758169/inputs.zip), [run log](../brouter-core/build/refine-evaluation-12687333987490758169/run.log)

Evidence lives under `build/`; a clean build deletes it. Preserve this directory before cleaning. No changes were pushed or deployed.

## What the full experiment showed

| Measure | Gravel | Fastbike |
|---|---:|---:|
| Requests, including every skip and truncation | 230 | 230 |
| Finished baselines priced / attempted | 182 / 187 | 195 / 204 |
| Accepted changes | 17 (7.4%) | 17 (7.4%) |
| Median cost/m gain, all requests | 0% | 0% |
| Mean cost/m gain, all requests | 0.224% | 0.272% |
| Median cost/m gain, accepted changes only | 3.22% | 2.24% |
| Failed requests / measured publication regressions | 0 / 0 | 0 / 0 |
| Truncated searches | 43 (18.7%) | 34 (14.8%) |
| Median refinement time | 414 ms | 601 ms |
| p90 refinement time | 3,000 ms | 3,000 ms |
| Median full request time | 3,266 ms | 4,133 ms |
| p90 full request time | 18,872 ms | 20,358 ms |
| Maximum full request time | 60,034 ms | 54,222 ms |

All 460 requests returned a route. The 60,034 ms maximum includes overhead beyond the routing deadline; the cap is not an exact end-to-end response guarantee. Measurement overhead averaged about 17 ms per request. Full latency includes it. Component times overlap and must not be summed.

Finished-baseline pricing coverage is 377/391 attempted (96.4%), or 377/460 overall (82.0%). The 69 unattempted cases comprise 51 unsupported producing tiers, 15 forced corridors and three baseline gate rejections. Three successfully priced baselines later failed raw search pricing. Failed pricing is never replaced by summed leg costs.

The 34 accepted changes all have different geometry signatures and continuous before/after prices. The other 426 have identical before/after geometry signatures. The measured regression checks found no worsening of the guarded distance, crossings, reuse, gate or cost conditions. This is evidence from this corpus, not a proof that every future route is valid or preferred.

The full Gradle test **failed deliberately at the unchanged product assertion**, after all cells, report generation and manifest verification. Gravel missed the 2% all-request median-gain bar and the maximum 5% truncation bar. Fastbike was a comparison profile rather than the formal gate; it also misses those thresholds. The p90 added-latency limit was met at its boundary. No thresholds were relaxed.

## Does it plausibly improve the ride?

Yes in some cases, with clear tradeoffs. Across the 17 accepted routes for each profile:

| Rider concern | Gravel changes | Fastbike changes |
|---|---|---|
| Distance fit | All 17 improve; median paired error change −1.93 percentage points | All 17 improve; −1.70 points |
| Actual route change | Median 8.0 km lies outside the original's 10 m corridor | Median 8.5 km lies outside that corridor |
| Traffic exposure | Observed high-traffic distance decreases in 6, increases in 2 | Decreases in 6, increases in 9 |
| Surface suitability | Firm-unpaved distance increases in 8, decreases in 5 | Paved distance increases in 15, decreases in 2 |
| Continuous sections | Longest firm-unpaved run increases in 3, decreases in 1 | Longest paved run increases in 6, decreases in 6 |
| Climbing | Ascent increases in 10, decreases in 7; median +6 m | Increases in 11, decreases in 6; median +6 m |
| Interruptions | Sharp geometry bends decrease in 12, increase in 5; mapped signals decrease in 4, increase in 1 | Bends decrease in 12, increase in 4; signals decrease in 4, increase in 6 |
| Repetition | Interior repeated geometry decreases in 3, increases in none; U-turn count decreases in 8, increases in 2 | Interior repetition decreases in 2, increases in none; U-turn count decreases in 6, increases in none |

Unmentioned cases are unchanged for that metric. These are accepted-only descriptions, not expected benefits for every request. Across all requests the median paired ride-metric change is generally zero. The corridor estimate samples geometry at at most 25 m intervals; it is not physical edge identity and ignores traversal direction.

Lower traffic, less climbing and more gravel are not guaranteed by a lower profile cost. More paved distance can also result from a longer route that better fits the target. The existing objective remains profile cost/m; none of these observations influences optimization.

### Coverage limits

Median original-route coverage across **all** cases:

| Available attribute | Gravel | Fastbike |
|---|---:|---:|
| Surface | 82.5% | 91.6% |
| Traffic estimate | 5.6% | 69.7% |
| Smoothness | 17.3% | 13.9% |
| Elevation | 99.9% | 99.7% |

Among accepted cases, median traffic coverage is 10.8% for gravel and 79.1% for fastbike; smoothness coverage is only 1.7% and 10.3%. Some routes have no surface or smoothness observations. Zero tagged signals, barriers or poor surface does not prove their absence. Traffic is an estimate, not measured vehicle exposure or a safety rating. Continuous tagged sections can still contain intersections. Climb uses complete 100 m windows, while the review chart displays raw samples. Unknown values stay explicit; no enjoyment score is fabricated.

### Systematic examples, including drawbacks

The review contains all 460 pairs. Its default 74-case sample takes the first alphabetical case in each region/profile/outcome group: 16 accepted, 24 unchanged, 20 truncated and 14 unpriceable examples across 12 represented regions. There were no failed whole requests to sample. The configured matrix lists two further regions but yields no cells for them under its existing filters.

Examples drawn from that fixed sample illustrate why numerical gains need human review:

- **Basel 30 km gravel east:** 3.73% lower cost/m, about 88 m less ascent and 323 m more firm unpaved surface; distance error improves only slightly.
- **Annecy 100 km fastbike north:** 2.04% lower cost/m and better distance fit, but about 5.6 km more observed high-traffic distance and 20 m more ascent.
- **Garmisch 30 km fastbike west:** 2.24% lower cost/m, yet geometry distance error remains 33.63%. A better baseline comparison is not automatically a satisfactory ride.
- **Lozère 80 km gravel east:** unchanged because the literal finished geometry is unpriceable; the frozen graph probe exposes the failing junction and omitted transfer sequence.

A/B labels stay hidden until Reveal, GPX names are neutral, and both routes share map/elevation scales. This is practical blinding, not secrecy against inspecting the page source. Feedback downloads locally and is never sent automatically. No human preference responses were collected.

## What failed

All six original smoke requests returned `baseline_unpriceable`. A new production regression reproduced all six failures in nine seconds. The walker failed at internal joins, not simply because a route was long.

| Original case | First invalid join | Operation that created it |
|---|---|---|
| Dreieich gravel east | 810948546417652156 → 810948224295104750 | proximity-based micro-detour removal |
| Dreieich fastbike east | 810864871864813465 → 810863858252531816 | proximity-based back-and-forth removal |
| Mallorca gravel north | 784467594831327099 → 784467384377929576 | proximity-based micro-detour removal |
| Mallorca fastbike north | 784209372807545502 → 784207530266574876 | proximity-based micro-detour removal |
| Basel gravel north | 805438043244965076 → 805436948028304627 | proximity-based back-and-forth removal |
| Basel fastbike north | 805648908959331608 → 805647070713328569 | proximity-based back-and-forth removal |

The numbers are graph position IDs. The diagnostic logs retain the rejected index, supplied geometry and decoded link geometry. Nearby roads are not necessarily connected. Cleanup had been deleting the span between them and leaving a connection that did not exist in the graph.

Fixing those joins exposed further causes:

- Generated waypoint copies replaced the original clipped crosspoint with a junction while the finished geometry retained the clipped samples. Original match provenance is now preserved in memory. The walker verifies the existing graph link and its mandatory transfer geometry, then prices the literal extra via samples through the same path state. Elevation comes from the graph and interpolation, not the displayed endpoint elevations. Turn restrictions are checked against native graph geometry so an inserted sample cannot hide a restricted turn. It restores the link geometry after that evaluation. It does not route a substitute or reset at a via.
- Mallorca's finished routes end at a transfer point before the junction recorded in their waypoint metadata. Detailed pricing now uses the displayed endpoints and recognizes an endpoint that is itself a decoded transfer point.
- Basel fastbike's cleanup removed a loop needed to avoid a forbidden turn. A first guard against that turn exposed another unsafe trim inside an edge. Cleanup now retains detailed restriction snapshots and only makes these joins at real graph junctions with a permitted resulting turn. This avoids reversing partway along a one-way edge.

Cleanup corrections apply to route construction, including routes with refinement disabled. This is a deliberate correctness change, not a search-tuning result. The paired experiment compares the repaired baseline with refinement in the same request; it does not attribute baseline cleanup changes to the optimizer.

## What remains broken, and why no fallback was added

The full matrix's 14 additional finished-route failures occur at recorded internal graph positions. Frozen-link probes show omitted or out-of-order native transfer geometry, including a jump to a distant point followed by the return traversal in Lozère, Grenoble and Girona. Other cases involve clipped-point sequences that do not match a complete incident link. The upstream construction stage responsible for every additional case has not been isolated. See the complete case list and exact graph output in the diagnosis artifact.

Twelve of the 14 failures reproduced at the same position in isolated production tests. Two regenerated routes priced successfully. AUTO's bounded choices can vary with runtime, and the direct regression differs in observational tag settings. This is precisely why the quality experiment uses paired observations from one request. Do not equate a fresh request with replay of the frozen original. The archive includes full geometry but not every internal waypoint field needed for a standalone oracle replay.

Inserting graph points, deleting the problematic excursion or pricing different legs would conceal these defects by changing the subject of comparison. No such repair was applied. The three raw-baseline failures are also listed; their deeper failure reason is not retained by the present CSV.

## Where search candidates are lost

The run generated 6,212 proposals, performed 5,353 evaluations and attempted 372 finalizations. The largest recorded filter is `raw_length_error_worse` (2,943, about 55% of evaluations). Pre/post-snap displacement rejects another 849 proposals. Candidate pricing fails 20 times; final pricing fails 12 times. Final rejection counts include length (113), cost (96), quality gate (57), reuse (19), direction (4), crossings (3) and RCS (3). Counts are events, not mutually exclusive request totals.

The raw-length filter compares a proposal with the raw search baseline before detailed finalization; the published result must separately match or improve the original finished route's length error. That restrictive policy explains substantial attrition but is not by itself a demonstrated implementation bug. It was not weakened. No search tuning rounds were performed.

This run tests the existing local search with 16 evaluations, one chain and three finalists. It does not test a large GA/annealing population or Sherpa's claimed throughput. The next bottleneck is reliable construction/replay of exact detailed geometry, followed by better candidate yield within the existing constraints—not a demonstrated need for a Rust rewrite.

## Validation and reproducibility

- All six original production pricing regressions pass, including repeated pricing without state leakage and unchanged geometry signatures.
- 642 selected core tests passed: continuous-state, endpoint/via geometry, graph elevation interpolation, native turn restrictions, cancellation/deadlines, opening heading, route measurements, routing and greedy parity. Three map-access tests passed.
- Thirteen smoke/report integration tests passed. Core main/test/integration Checkstyle and PMD, plus map-access Checkstyle and PMD, passed. Core/map-access `git diff --check` is clean.
- The completed 460-cell run verified its input hashes before reaching the expected product-bar assertion. Its raw records contain 460 unique labels, exactly 230 per profile.
- An independent export audit compared **2,035,329 points in 920 GPX files** with the immutable captured coordinates and elevations; all matched. All 34 accepted pairs use continuous pricing; all 426 unchanged pairs preserve geometry.
- The same review template rendered correctly in Safari during smoke validation, including hidden/revealed labels. Final full-corpus UI recheck was unavailable because the computer-use service returned `cgWindowNotFound`. Full-corpus data and GPX audits passed. Automatic approval review rejected the optional external OSM tile request because it would disclose route bounds; online background remains untested. Offline maps require no external tiles.

Corrected six-case smoke: `brouter-core/build/refine-evaluation-853567401709769965`. Final validation XML and logs: `brouter-core/build/reports/refinement/overnight/final-validation/`.

The earlier 347-cell run (`refine-evaluation-7571017624342160869`) was stopped after discovering that replayed via elevations came from display samples rather than native graph interpolation. It is marked invalid and contributes no quality evidence. The earlier six-case preview (`refine-evaluation-18132754801696185106`) is likewise superseded for cost comparisons. A failing elevation regression was added before the correction; the full matrix above uses the corrected implementation.

Frozen settings: local tiles, no downloads; AUTO; seed zero; `processUnusedTags=1`; strict quality false as in the existing matrix; local refinement, 16 evaluations, one chain, top three finalists; three-second refinement cap; 60-second request cap; one sequential worker. Actual distance targets are 2π times the configured radii, slightly different from nominal case labels. Code, build files, profiles and resources are archived; tile hashes and runtime details are in the manifest. Routing inputs were unchanged throughout the 56m 42s run.

Reproduce the full experiment (the product assertion is expected to fail with these results):

```sh
./gradlew :brouter-core:integrationTest \
  --tests 'btools.router.FullABEvaluationMatrixTest' \
  -Dloop.forks=1 -Dloop.segments.nodownload=true --console=plain
python3 misc/scripts/analyze-refinement.py <new-experiment-directory>
```

Refinement remains off by default. Existing unrelated uncommitted work and historical reports were preserved. The next useful work is to trace the remaining frozen geometry defects through construction stages, retain complete waypoint provenance for exact replay, and collect cyclist feedback on the blind pairs before changing the product decision.
