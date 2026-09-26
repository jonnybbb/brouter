# Refinement correctness and measurement repair

Validated on 2026-09-25, on the working tree based on `8d6cbb13` (`roundtrip-refine`). This implements the authorized correctness and evaluation repairs. Refinement remains off by default. The optimizer strategy and Rust/CCH implementation were not expanded.

## Correctness changes

- The baseline is priced from the exact original tier track. A finalized candidate is priced from its merged, cleaned track. Reconstructed raw legs supply search energy only; they cannot supply a finished route's publication price.
- `LoopPrice` records continuous cost, distance, a geometry fingerprint and success/failure/timeout/cancellation. The continuous walker no longer silently sums independently priced legs. Publication requires continuous, finite, positive prices on both sides.
- A scoped refinement deadline reaches leg search, matching, pricing, retracking and cleanup. Checks also run after expensive operations and before publication. Timeout preserves the original route and marks truncation; cancellation is distinguished and propagated. Scoped deadline and heading state are restored.
- Forced heading applies to the opening skeleton leg. Whole-track pricing derives heading validity from the request, independently of the previous routed leg.
- Measurement snapshots copy scalar metrics and a geometry fingerprint before and after the actual refinement hook in one request. They use `RouteChoiceScore.Verdict.score()`, the ranking score. Measurement is explicitly enabled by the harness; its overhead is reported separately.

The timeout is cooperative. A synchronous tile read or an individual helper operation can finish after the deadline before the next check observes it. This is not a hard real-time bound; an expired result cannot pass the final publication check.

## Repaired evaluation

`FullABEvaluationMatrixTest` now runs one request per cell. `PilotExperimentBenchmarkTest` uses that same paired runner and report writer. The regional manifest contains 230 gravel and 230 fastbike cells; Innsbruck is excluded because it is absent from the regional quality suites that define this corpus.

Each run writes a unique `brouter-core/build/refine-evaluation-*` directory containing:

- `manifest.properties`: actual commit, JVM, OS, processor/heap information, search parameters, and SHA-256 hashes of code, build files, profiles and available segment tiles. Hashes are checked again after the run, including the tile file inventory.
- `manifest-cells.csv`: exact region coordinates, radii, distance labels, directions, profiles and algorithms. Per-cell output also records the resolved direction, actual target distance and producing tier.
- `working-tree.patch` and `inputs.zip`: tracked changes plus a code/build/profile snapshot that includes untracked Java sources. Tiles are identified by hash and must be retained separately. The harness never downloads tiles.
- `cells.csv`, `rejections.csv`, `evaluations.csv`: paired measurements, skip/failure reasons, proposal and routing counts, traces, and phase timings.
- `summary.md`: computed correctness, gain, RCS, timing and truncation statistics. Skips, failed requests and truncated searches contribute zero gain; negative gains remain negative. All cells remain in timing statistics. Failed requests are counted separately.

Initialization and finalization timings contain routing/pricing/cleanup timings, so those columns must not be summed. Request timing includes measurement overhead. Subsets are explicitly marked as validation rather than product acceptance. A full run checks the existing gravel acceptance bars by default.

Test report writers that formerly overwrote tracked M0/geometry documents now write under `brouter-core/build/reports/refinement/`. The older M0/M1/M3 documents and ADR-0005 are historical evidence; their quality claims are not revalidated by this change. This note records the corrective validation without replacing the user's existing edits to those documents.

## Validation results

The final Gradle run completed successfully:

| Validation | Result |
|---|---|
| Core roundtrip package, `RoutingEngineTest`, greedy parity and oracle tests | 623 passed, zero failures/skips |
| Map-access tests | 3 passed, zero failures/skips |
| Report contracts plus paired smoke matrix | 6 JUnit tests passed; matrix test exercised 6 routing cells |
| Checkstyle and PMD | Passed for core main/test/integrationTest and mapaccess main |

Regressions cover the reproduced original/finished price mismatch, rejection of a real more expensive reconstruction, prohibition of independent-leg fallback, heading state, cancellation during pricing, active deadline interruption, initialization timeout, finalization failure isolation and unchanged greedy parity. Report tests cover even-sized medians, nearest-rank p90, skip/truncation denominators, missing observations, immutable baseline snapshots and computed length/gate/cost regressions. The synthetic publication tests remain explicitly synthetic; they are not evidence of an optimizer win.

The paired smoke used AUTO, local refinement, 16 evaluations, a 3000 ms stage cap, seed 0 and one matrix worker. It covered 30 km gravel/fastbike cells at Dreieich east, Mallorca north and Basel north.

| Profile | Cells | Failed requests | Refined wins | Correctness regressions | Truncations | Added p90 | Request p90 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Gravel | 3 | 0 | 0 | 0 | 0 | 33 ms | 1168 ms |
| Fastbike | 3 | 0 | 0 | 0 | 0 | 35 ms | 1639 ms |

**All six cells reported `baseline_unpriceable`.** Their before/after geometry fingerprints and quality metrics were unchanged. This validates safe rejection and the paired measurement path, not successful optimization or useful search throughput. The next substantive blocker is continuous-walker coverage on finished production routes. These measurements do not justify enabling refinement or choosing a faster leg engine.

Local evidence: [computed summary](../brouter-core/build/refine-evaluation-8708600103989553934/summary.md), [raw cells](../brouter-core/build/refine-evaluation-8708600103989553934/cells.csv), [manifest](../brouter-core/build/refine-evaluation-8708600103989553934/manifest.properties), [validation log](../brouter-core/build/reports/refinement/correctness-validation.log). Build artifacts are removed by a clean build.

## Reproduce

From the repository root:

```sh
./gradlew :brouter-core:test \
  --tests 'btools.router.roundtrip.*' \
  --tests 'btools.router.GreedyPlannerParityTest' \
  --tests 'btools.router.LoopCostOracleTest' \
  --tests 'btools.router.RoutingEngineTest' \
  :brouter-mapaccess:test

./gradlew :brouter-core:integrationTest \
  --tests 'btools.router.RefineEvaluationReportTest' \
  --tests 'btools.router.FullABEvaluationMatrixTest' \
  -Dloop.forks=1 -Dloop.segments.nodownload=true \
  -Dloop.refine.cells=basel_30km_gravel_N,basel_30km_fastbike_N,mallorca_30km_gravel_N,mallorca_30km_fastbike_N,dreieich_30km_gravel_E,dreieich_30km_fastbike_E
```

The full 460-cell acceptance matrix and the eight-variant pilot were not run. Removing the cell filter selects the full matrix; the product acceptance assertions then apply. AUTO's tier selection may still depend on request timing, which is why the comparison must remain inside a single request.
