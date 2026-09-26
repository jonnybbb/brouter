# PR #46 review fixes

This follow-up to merged PR #46 repairs seven confirmed review findings. Refinement remains off by default.

## What changed

| Review finding | Repair and evidence |
|---|---|
| P1: missing native bend accepted | Finished-track pricing always validates the full native sequence. It no longer infers raw geometry from absent messages or uses the junction shortcut. The real-fixture regression rejects the four-point omission with and without messages. |
| P1: displaced endpoint accepted | Opening and closing samples must belong to the traversed native section under integer projection rules, and the cost kernel must actually reach the clip. The 72 m endpoint displacement is rejected. Clipped sections cannot omit intervening native points. |
| P2: invented raw via backtracking | Raw legs have a separate preparation API. Their guided detail passes establish the clipped geometry; only then does one continuous walk price the joined route. The reproduced mid-edge case now agrees with its detailed reference at cost 2,084, instead of 3,236. No independently priced leg costs are summed. |
| P2: wrong raw distance denominator | The raw price returns cost, distance and signature from that same joined geometry. Search uses its distance on both sides of the length filter. The regression now uses 3,689 m rather than 3,981 m, and 3,607 m rather than 4,108 m. |
| P2: generated vias classified as user vias | Snapping marks optimizer-created vias as generated and aligns their waypoint position with the snapped crosspoint before caching/copying. MOVE, REPLACE and INSERT checks exercise the cleanup classification. SHAPING was already correct and remains so. |
| P2: Android API-23 Map calls | Replaced `getOrDefault` and `computeIfAbsent` with compatible Map operations. No new dependency or desugaring requirement. Android runtime execution was not available in this checkout. |
| P3: stale result metadata | Acceptance refreshes named and matched waypoints, distance and tolerance together. Original planner leg tracks are explicitly unavailable after whole-route refinement, rather than describing the wrong route. Planner telemetry remains historical. |

The finished-track oracle never reconstructs a replacement route. Guided detail preparation is limited to the explicitly raw leg API. It rejects failed retracks and mismatched seams. This costs additional preparation work; it is a correctness repair, not a throughput claim. Existing heading scopes, continuous turn/elevation state, deadlines, cancellation, and publication checks remain in force.

The speculative review observations were not implemented as blanket changes. In particular, publication still checks the deadline; no earlier finalist is published after expiration. No objective, operator mixture, acceptance margin or tuning setting changed.

## Follow-up validation on merged master

Reapplied the repairs to `db1ca3ea`, which contains merged PRs #44 through #47, on branch `fix/refinement-review-correctness`. The patch applied without conflicts. The unrelated local document edits, route-generation utility, reports and generated comparison artifacts were not included.

- `./gradlew build :brouter-map-creator:integrationTest :brouter-server:integrationTest`: passed. Across those suites, 1,162 tests were recorded: 1,154 passed, 8 skipped, no failures or errors. The core suite recorded 909 tests: 902 passed and 7 skipped. Checkstyle, PMD and server distribution checks passed.
- Selected core integration checks: all 16 passed, comprising nine `FinishedLoopPricingTest` cases, six `RefineEvaluationReportTest` cases and one nine-cell `FullABEvaluationMatrixTest` run. Map tiles were reused locally with downloads disabled; tests ran in one fork.
- The paired run again recorded zero accepted changes, zero failed requests and zero measured regressions across four gravel and five fastbike requests. One fastbike request was truncated. Added latency p90 was 493 ms for gravel and 3,000 ms for fastbike; full request p90 was 1,632 ms and 4,228 ms respectively. These subset results do not establish full-matrix quality or latency.
- Android runtime testing and the full 460-cell matrix were not run for this follow-up. Refinement remains off by default.

Reproduction after generating the bundled fixture with `./gradlew :brouter-map-creator:test --rerun-tasks --no-build-cache`:

```sh
./gradlew build :brouter-map-creator:integrationTest :brouter-server:integrationTest
./gradlew :brouter-core:integrationTest \
  --tests btools.router.FinishedLoopPricingTest \
  --tests btools.router.RefineEvaluationReportTest \
  --tests btools.router.FullABEvaluationMatrixTest \
  -Dloop.forks=1 -Dloop.segments.nodownload=true \
  -Dloop.refine.cells=dreieich_30km_gravel_E,dreieich_30km_fastbike_E,mallorca_30km_gravel_N,mallorca_30km_fastbike_N,basel_30km_gravel_N,basel_30km_fastbike_N,rural_lozere_80km_gravel_E,basel_30km_fastbike_S,mallorca_100km_fastbike_E
```

The real-route checks require the corresponding existing `segments4` tiles. Follow-up evidence is local to `/private/tmp/brouter-refinement-followup/brouter-core/build/refine-evaluation-7475259912851196120/`, including frozen inputs, tile hashes, raw records, paired maps and GPX files. Build logs are `/tmp/brouter-followup-build.log` and `/tmp/brouter-followup-integration.log`. None of these local evidence paths is expected to exist in a fresh checkout.

## Original repair validation

The results in this section were recorded before PR #46 merged. Follow-up validation on current master is recorded separately below.

The four new pricing regressions failed before the repair. The generated-via and publication-metadata checks also failed before their respective fixes.

Final checks:

- Full JVM `build`, including Checkstyle, PMD, module tests and server distribution checks: passed.
- Core suite: 902 tests, 884 passed, 18 skipped, zero failures.
- Selected integration suite: 16 tests passed, including all nine real-geography finished-pricing cases, the paired evaluation, and report-accounting checks.
- The nine pricing cases include the original six Dreieich/Mallorca/Basel cases plus Lozère 80 km gravel east, Basel 30 km fastbike south, and Mallorca 100 km fastbike east.
- Existing clipping, native turn restriction, elevation, heading, cancellation, timeout, ownership and publication tests remain passing.
- Re-ran the original sparse-geometry and endpoint mutation sweeps: neither finds a wrongly accepted mutation now. The original denominator probe matches cost and distance for all four junction pairs. Logs are retained with the validation evidence.

`ContinuousPricingRegressionTest` contains the actual-engine counterexamples. `ExactLinkGeometryTest` additionally checks omitted transfers between clipped endpoints. `RefinePricingContractTest` retains an original-versus-reconstructed comparison using valid geometry and adds an explicit malformed-baseline rejection.

Validation used `/tmp/brouter-pr46-tests.gradle` to exclude the pre-existing untracked `GenerateFreshComparisonRoutesTest` utility. It writes local comparison files and is not part of the committed test suite. An initial broad run picked it up and regenerated GeoJSON/data files under `loop-comparison`; that run was stopped. Source and pre-existing document edits were not changed. Final runs were serial to avoid the initial overlapping Gradle result-writer conflict. Logs are retained in `brouter-core/build/pr46-review-fixes/`.

## Paired route observations

The original repair was evaluated in a serial nine-cell run: `brouter-core/build/refine-evaluation-4602991082478880479/`. It contains the frozen input archive, hashes, settings, source patch, raw CSVs, route maps and GPX pairs. Settings remain local search, 16 evaluations, 3,000 ms stage cap, one chain, variety seed zero.

| Measurement | Gravel | Fastbike |
|---|---:|---:|
| Requests | 4 | 5 |
| Entered refinement | 4 | 4 |
| Accepted replacements | 0 | 0 |
| Failed requests / measured regressions | 0 / 0 | 0 / 0 |
| Truncated requests | 0 | 1 |
| Added latency p90 | 477 ms | 3,002 ms |
| Full request latency p90 | 1,569 ms | 4,190 ms |

All nine result geometries match their originals. One request skipped refinement because its producing tier is unsupported. Four reached finalization, where cost or distance guards rejected the candidates. The remaining eligible cases returned no finalist, including one truncated search. The small sample does not establish full-matrix quality or latency distributions. The 460-cell experiment has not been rerun as part of this repair.

The local `ride-review.html` in that evidence directory shows the identical results explicitly. These generated artifacts are not included in this code-only follow-up. This run provides no evidence of cyclist benefits, and no human preference testing was performed.

## Remaining limits and next bottleneck

Strict validation also exposed an existing malformed gravel fixture loop at direction 90 and radius 1,000 m. At track index 139, the route goes from graph node `810511353106683341` directly to `810510524177994657`, omitting native transfer `810511052458972257`. The previous raw shortcut priced that chord as the road. The new negative regression requires refusal and preservation of the original baseline; it does not substitute newly routed geometry to make the price succeed. The originating construction path still needs separate diagnosis.

Candidate pricing failures, length rejection and the final publication guards continue to limit search yield. The current nine-cell run produced no accepted improvement. Repairing correctness therefore does not justify enabling refinement or repeating the old claims about improved routes. A full frozen matrix and candidate-failure diagnosis are the next evaluation work.
