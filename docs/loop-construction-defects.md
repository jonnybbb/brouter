# Loop construction defects

[Open the HTML report with all 23 route comparisons and GPX downloads](reports/loop-construction/index.html).

Fixed two route-construction defects and one pricing-walker defect. The final checks passed all 23 targeted finished-route cases, 878 core tests and 30 selected integration tests. Eighteen core tests were skipped; Checkstyle and PMD passed.

Two reconstructed raw seeds still correctly fail at private gates. The historical Freiburg raw-seed failure did not reproduce. Refinement remains off by default. These are correctness improvements; cyclist preference and full-matrix quality have not been re-established for the new code.

## Confirmed cause: sparse bulge connectors

`rural_lozere_80km_gravel_E` reproduces the original continuous-pricing failure at point 919, junction 787215828444862662. Stage probes show valid detailed geometry through leg detailing, concatenation, back-and-forth removal and micro-detour removal. `WaypointSnapper.repairViaPinnedBulges` is the first operation to introduce the malformed sequence.

It used `findTrackUnguided`, a search primitive returning graph junctions without the detailed road shape. The connector also ended at a graph junction beyond the requested clipped splice endpoint. Cleanup inserted those search nodes into the detailed route. In this case the output jumped to a remote transfer point and traversed the road shape backwards to the same junction.

The repair details the connector, requires exact mouth endpoints and rejects a raw-detail fallback. It constructs a separate candidate, retains the existing crossing check, and walks the complete candidate under continuous turn/elevation state before committing. Interrupted or failed validation leaves the original route and waypoint indices intact. Interior connector routing temporarily suppresses the request's opening heading; whole-route validation restores it. The operation observes both the request deadline and any earlier refinement deadline.

The original Lozère production reproduction passed after this repair. Six focused regression tests failed before the fix and passed afterward. They exercise the actual cleanup call, including raw fallback, a displaced endpoint, whole-route rejection, cancellation and timeout.

## Confirmed cause: rematching a committed endpoint

`basel_30km_fastbike_S` remained unpriceable after the connector fix. Its bad join already exists before cleanup. The accepted detailed leg ends at position 805441427679142253 on its original matched edge. The planner calls its ordinary road matcher again to anchor the next leg; that returns position 805442037564498114 on a different edge. The next detailed leg starts at this new point. Concatenating the two legs inserts a roughly 22-metre gap.

The fix preserves the accepted waypoint's exact crosspoint and matched edge for the continuation. It retains the routing engine's existing clipping behavior. If an earlier return estimate used the raw leg's overshooting endpoint, the return is rerouted from the exact committed endpoint.

The original Basel production reproduction passes after the fix. An initial extra guard compared requested coordinates with clipped output coordinates. A 23-case regression check exposed one failed smoke request and severe Basel latency increases. Targeted logs showed normal projection differences of one coordinate unit. That guard was removed; the continuation still preserves the original matched edge and request, and the exact geometry walker remains unchanged. This was a rejected implementation, not a quality-tuning round. Its XML and logs are retained explicitly as intermediate evidence.

The Lozère, Basel and Mallorca regressions are included in the default `FinishedLoopPricingTest` selection, alongside the original six smoke cases.

## Confirmed cause: a rounding check rejected valid geometry

The remaining `mallorca_100km_fastbike_E` mismatch was a walker limitation, not an invalid route. Its native road section runs from `(182792913, 129643725)` to `(182793355, 129643558)`. The router's own `RoutingContext.calcDistance` projection produces the clipped point `(182793050, 129643672)`. The finished route retains every native transfer point in order.

`ExactLinkGeometry.onSection` checked the perpendicular projection's residual separately on each coordinate axis. The latitude residual here is about 1.083 integer units, so its 1.01-unit threshold rejected the point. That test is not equivalent to asking whether the native segment passes through the integer cell that rounds to the displayed point.

The replacement checks intersection with the coordinate cell implied by native integer truncation. It keeps the requirement that the sample belongs to an explicitly matched via on this same edge, retains all native transfer points and their order, and keeps native turn-restriction checks and elevation interpolation. It does not modify the route or add missing geometry. A new test calls the actual projection routine, fails under the old check, and passes under the corrected check. Moving the sample two coordinate units off the road still fails.

This distinction matters: the 14 historical finished-route pricing failures were evidence of a problem in the pipeline, not proof that all 14 route geometries were invalid.

## Evidence and limits

Local traces, red/green test XML and pre-probe source snapshots are in `brouter-core/build/construction-diagnosis/`. The original frozen 460-cell evidence remains unchanged in `brouter-core/build/refine-evaluation-12687333987490758169/`.

After the connector fix alone, the 20-case production check passed 14 and failed six. That includes all six smoke cases; eight of the 14 historically failing labels passed, although two of those labels had also passed earlier regenerations. Thus the evidence is six previously reproducible failures cleared, not eight newly proven fixes. The six remaining failures retained the original frozen mismatch positions.

These repairs do not establish that cyclists prefer the resulting routes. They prevent demonstrated invalid construction operations. New route quality and full latency measurements are separate from the historical refinement gains.

## Raw-seed access failures

The three-cell paired rerun is retained in `brouter-core/build/refine-evaluation-2675540930182886791/`. It used the same 16 evaluations and 3,000 ms refinement budget as the earlier experiment and verified its input manifest. It recorded no accepted changes, no failed requests, no publication regressions and no truncations.

- Berlin 50 km fastbike east: original finished route prices; the reconstructed raw seed fails at source 831329377420482372 while entering node 831329699543029566.
- Grenoble 80 km fastbike east: original finished route prices; the reconstructed raw seed fails at source 798013565206247302 while entering node 798014007587878854.
- Freiburg 30 km fastbike west: this regeneration selected WAYPOINT, which does not refine. A separate explicit raw-seed reconstruction priced successfully at 40,130. The historical failure was not reproduced, and is not claimed fixed.

The first two target nodes are tagged `barrier=gate access=private`. The active fastbike profile evaluates their node initial cost to 1,000,000, which `StdPath` rejects. `RawSeedProbe.java` reproduces the distinction: every individually priced leg succeeds, but their continuous traversal fails at the private-gate via. These individual costs are diagnostic only; they are never summed or substituted for the continuous price. The paired outputs preserve the original geometry signatures.

`RawNodeCostProbe.java`, `RawSeedProbe.java`, their logs and the retained three-cell manifest provide a local reproduction. This is a feasible-seed problem, not a reason to weaken node-access checks. A future initialization change could avoid using an inaccessible node as a through-via. It must produce a genuinely feasible continuous seed before search can proceed.

## Validation status

The final targeted integration run passed all 23 finished-route checks: all 14 historical failures, all six original smoke cases and the finished routes from the three historical raw-seed failures. It also passed the paired subset and six report tests, making 30 selected integration tests with zero failures or skips. Full core validation passed after a minimum-budget compatibility correction: 878 tests passed and 18 were skipped, with zero failures. Checkstyle and PMD passed for production and test code.

The existing minimum-budget test caught an intermediate mistake: enforcing an optional cleanup deadline aborted the first already-completed route that the competition intentionally retains under its minimum child budget. Ordinary generation now skips an expired optional repair and retains its valid route. Active refinement scopes still propagate timeout, cancellation always propagates, and candidate mutation happens only after successful full-route validation.

No new 460-cell quality result is claimed here. The historical 34 accepted changes out of 460 remain evidence for the earlier source snapshot, not a benchmark of these repairs.

## Reproducing the diagnostics

From the repository root, after compiling the core and integration-test sources:

```sh
probe_cp=brouter-core/build/classes/java/main:brouter-core/build/classes/java/integrationTest:brouter-core/build/classes/java/test:brouter-mapaccess/build/classes/java/main:brouter-expressions/build/classes/java/main:brouter-codec/build/classes/java/main:brouter-util/build/classes/java/main
probe_dir=brouter-core/build/construction-diagnosis
javac -cp "$probe_cp" -d "$probe_dir" "$probe_dir/RawSeedProbe.java" "$probe_dir/RawNodeCostProbe.java"
java -cp "$probe_dir:$probe_cp" btools.router.RawSeedProbe
java -cp "$probe_dir:$probe_cp" btools.router.RawNodeCostProbe
```

These use the existing local `segments4` data and profiles. They make no network requests. The production regression selection and request times are recorded in `brouter-core/build/construction-diagnosis/final-case-results.csv`; the default nine-case smoke selection is:

```sh
./gradlew :brouter-core:integrationTest --tests btools.router.FinishedLoopPricingTest -Dloop.forks=1 -Dloop.segments.nodownload=true
```

## Decision

Keep refinement off by default and review these correctness changes. The next search bottleneck is obtaining a feasible continuous seed when a generated via lies on an inaccessible node. Address that separately, then rerun the 460-cell experiment and use blind cyclist review to assess enjoyment. The two private-gate failures must remain rejected until the actual route changes legally.

Final local evidence: `construction-diagnosis/final-core/`, `construction-diagnosis/final-integration/` and `construction-diagnosis/validation-summary.json`, all under `brouter-core/build/`. Intermediate failed implementations and their diagnostics are retained separately. No deployment or push was performed, and pre-existing uncommitted work was preserved.
