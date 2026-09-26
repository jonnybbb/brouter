# Bidirectional Dijkstra in BRouter's routing loop — investigation

Question: would `_findTrack` (RoutingEngine.java:1936) benefit from a bidirectional
Dijkstra/A* implementation (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm#Bidirectional_Dijkstra)?

Method: real measurements, not estimates. All numbers below were produced by
`brouter-core/src/test/java/btools/router/BidirectionalSearchInvestigation.java`
(gated on env `BENCH_SEGMENTS=/path/to/segments4`, `BENCH_HEAP=8g`) using the real
trekking.brf profile and the real ~1.9GB rd5 segment data.

## What the loop actually is (matters for the answer)

`_findTrack` is not plain Dijkstra. Each request runs three passes (`searchRoutedTrack`,
RoutingEngine.java:1754):

- **pass 0**: weighted A*, `airDistanceCostFactor = 1.5` (non-admissible) — finds a first
  track fast, `costCuttingTrack.cost` from it bounds pass 1.
- **pass 1**: `airDistanceCostFactor = 0.0`, i.e. **plain Dijkstra**, bounded by
  `costCuttingTrack.cost + 5000` — the optimality-refining pass.
- **retrack**: guideTrack-guided pass (factor 0) that computes the 3 alternatives.

So the Wikipedia premise ("bidirectional Dijkstra beats Dijkstra") applies only to pass 1,
and whatever replaces it must reproduce the multi-pass, cost-cutting, 3-alternative logic.

## Part 1 — where the engine's search volume actually goes

Frankfurt (8.6821, 50.1109) → 4 destinations, trekking.brf, real segments.
`pass0pops`/`pass1pops` = priority-queue pops parsed from the engine's own `found track ... nodesVisited = N` lines:

| route | beeline | pass0 pops | pass1 pops | retrack pops | wall |
|---|---|---|---|---|---|
| Darmstadt | 27 km | 8,141 | 332,960 | 636 | 457 ms |
| Wuerzburg | 93 km | 719,596 | 1,959,677 | 1,467 | 2,175 ms |
| Coburg | 165 km | 650,983 | 3,490,551 | 2,358 | 3,037 ms |
| Munich | 305 km | 281,218 | 9,780,000 | 4,913 | 7,420 ms |

The bounded plain-Dijkstra pass 1 dominates everything.

## Part 2 — structural bidirectional vs unidirectional benchmark

To isolate the pure algorithmic gain from BRouter's machinery, a graph was direct-loaded
from rd5 (`E5_N45.rd5`, region lon 8–9.9E, lat 49–49.9N): **1,645,073 nodes /
8,459,990 directed edges**, symmetric distance weights. 20 random (s,t) pairs in beeline
buckets {5, 20, 50, 150, 1000} km, seed 4711. Five algorithms were run per pair and all
agreed on the optimal cost (mutual validation): Dijkstra, A* f=1.0, A* f=1.5,
bounded Dijkstra (bound = A*1.5 cost + 5000, the engine's pass-1 analogue), and a true
bidirectional Dijkstra (alternating fronts, stop when `topF + topB >= mu`).

Measured pops, bidirectional vs unidirectional Dijkstra:

| beeline | bidir / Dijkstra pops |
|---|---|
| ~20–50 km | 1.5–2.2× less |
| ~50–150 km | 1.05–1.45× less |
| >150 km | ~1.3× less |

Far below the textbook intuition — a hierarchical road network already concentrates the
unidirectional ball onto arterials, so two balls save little.

Against the *engine's* scheme (A*1.5 pops + bounded-Dijkstra pops), bidirectional pops were
only **~1.2–1.4×** lower at long range (per-row range 0.85–4.38, i.e. sometimes worse).
A bidirectional rewrite could therefore save at most ~20–40% of pops — before accounting
for pops-vs-wall overhead differences (dominance checks, costCutting, memory management).

## Part 3 — the cheaper lever: a heuristic in pass 1

Same routes as Part 1, run once with the default `pass2coefficient=0.0` and once with
`pass2coefficient=1.0` (admissible A* in pass 1), via `rc.keyValues`:

| route | factor | pass1 pops | wall | cost |
|---|---|---|---|---|
| Wuerzburg | 0.0 | 1,959,677 | 2,175 ms | 157119 |
| Wuerzburg | **1.0** | **1,106,217** | **1,416 ms** | 157119 |
| Coburg | 0.0 | 3,490,551 | 3,037 ms | 254322 |
| Coburg | **1.0** | **1,783,800** | **1,903 ms** | 254322 |
| Munich | 0.0 | 9,780,000 | 7,420 ms | 422966 |
| Munich | **1.0** | **3,976,816** | **3,452 ms** | 422966 |

Pass-1 pops drop 1.8–2.5× and wall time ~1.5–2.1×, with **identical resulting cost** for
this profile — a bigger measured win than bidirectional's structural ceiling, with zero
changes to the search loop. Caveat: admissibility holds only for profiles where
cost-per-meter ≥ air-distance-per-meter (true for trekking.brf here; a profile that can
undercut air distance would lose pass 1's optimality guarantee). A conservative variant
would auto-enable it per request only when `costCuttingTrack.cost / beelineMeters ≥ 1`.

## Correctness obstacles to a bidirectional rewrite

1. **Direction- and state-dependent link costs.** `StdPath.processWaySection` evaluates
   elevation with hysteresis buffers (uphillcost/downhillcost/elevationpenaltybuffer):
   the cost of a link depends on path history and travel direction. A reverse frontier from
   the target evaluates the *wrong* cost for the same link. Bidirectional Dijkstra's
   meeting-and-stopping argument assumes symmetric, history-free edge costs — violated here.
2. **Turn restrictions / KinematicPath**: multiple link-holders per link encode turn
   restrictions; reverse traversal would need an inverted turn-restriction model.
3. **Per-link dominance pruning** (`definitlyWorseThan`), the memory-panic path
   (`collectOutreachers`, `openBorderList`), island detection (`RoutingIslandException`),
   fastPartialRecalc and timeouts are all built around one forward `SortedHeap<OsmPath>`;
   a second frontier invalidates most of these invariants.
4. Only pass 1 is plain Dijkstra — bidirectional would have to replace a bounded,
   cost-cutting refinement pass, plus still reproduce the retrack/alternatives pass.

## Part 4 — the greedy *loop* planner (`fork-master`)

The point-to-point numbers above answer the question for plain routing. The greedy
round-trip planner (`btools.router.roundtrip.GreedyRoundTripPlanner`, branch
`fork-master`) has a different workload mix, measured with
`brouter-core/src/test/java/btools/router/LoopPlannerInstrumentation.java`
(worktree at `.worktrees/fork-master`, real segments4, gravel.brf, Dreieich start,
same BENCH_SEGMENTS gate). Per timestamped engine logs, work splits into:

- **Isochrone expansions** (`runIsochroneExpansion`): single-source, budget-bounded
  outward wavefronts that build the candidate pools (and feed ReturnDistanceOracle).
- **Leg searches** (`findTrack*`): short point-to-point hops between placed vias —
  **already goal-directed** (legs run at gravel's `pass1coefficient=4`, not plain
  Dijkstra; the `f0.0` calls are tiny cost-cutting/repair searches of ~100 pops).

Measured (pops = priority-queue pops):

| plan | total wall | leg searches | leg pops | isochrone expansions | isochrone pops |
|---|---|---|---|---|---|
| greedy 50km dir0 | 853 ms | 23 | 15,932 (~700 avg) | 4 | 549,498 |
| greedy 50km dir90 | 947 ms | 52 | 41,588 (~800 avg) | 5 | 681,158 |
| greedy 75km dir180 | 1,589 ms | 49 | 64,904 (~1,300 avg) | 5 | 1,434,703 |
| isogreedy 50km dir0 | 1,514 ms | 43 | 27,274 (~860 avg) | 8 | 1,321,222 |

**≥95% of all search pops in a loop plan come from isochrone expansions** — searches
with *no destination*. A bidirectional frontier is structurally impossible there: there
is no target to start the backward frontier from. The remaining leg searches are already
short A* runs at factor 4 with cached segments (~1k pops each); even eliminating them
entirely would save single-digit percent of plan latency. Per-leg bidirectional gains
measured in Part 2 (~1.5–2.2× vs plain Dijkstra at 5–20 km) do not transfer, because the
legs are not plain Dijkstra to begin with.

## Part 5 — expansion-side knobs under the microscope

Since the isochrone expansion *is* the loop planner's cost center (70–79% of plan wall),
the follow-up question is whether its two sizing knobs — the 4×radius cost budget
(`ISO_BUDGET_FLOOR_FACTOR`) and the 1.5×radius geographic cutoff — are profligate.
Measured with per-pop radial/cost histograms added to `runIsochroneExpansionOnce`
(under `-Diso.dump`), parsed by `LoopPlannerInstrumentation.expansionSectorConcentration`.
All pops binned by air distance and cost from the anchor, in units of searchRadius:

| plan | pops | <0.5r | 0.5–0.75r | 0.75–1r | 1–1.25r | 1.25–1.5r | >1.5r (geo fringe) |
|---|---|---|---|---|---|---|---|
| greedy 50km | 549k | 12.5% | 15.7% | 25.7% | 26.1% | 19.9% | 0.11% |
| greedy 75km | 1,434k | 15.3% | 11.8% | 20.6% | 30.0% | 22.2% | 0.09% |

Cost histogram of the same pops: the [3,4)×r band is the plurality (~40–45% of step
expansion pops); <2×r carries only ~20%. The ISO_GREEDY start-pool expansion (the only
calibrated one) ran to a 10.8×r budget; the sliver beyond 4×r is 31k pops, 2.4% of the
whole plan, for ~5% extra frontier air reach.

Where do *committed* candidates sit? Matching each expansion's anchor to the committed
step it served (track crosspoints; proxy — late retry phases muddy attribution slightly)
gives used-candidate air distance / radius = **0.80–1.15** across all plans, inside the
[0.5, 1.65] window as designed. In cost units that is ≈2.9–3.3×r for the average leg
(step air ≈ 0.73 × leg route length; gravel floor cost ≈ 1.0/m), i.e. **only 6–25% under
the 4×r floor budget** — and terrain indirectness up to 1.43+ was observed mid-plan, so
legs at the window edge in ≥2-cost/m terrain already press against the budget.

Conclusions per knob:

1. **Geographic cutoff (1.5×r): not the limiter.** Pops beyond it = 0.1% (bare boundary
   fringe; it only blocks *neighbor expansion*, preventing runaway). Shrinking it to
   1.2×r would drop the [1.25,1.5] band (~20–24% of pops), but that band is exactly where
   far-window pool candidates, per-bucket frontier picks, the ReturnDistanceOracle cell
   cloud and the Phase-2.1 indirectness signal live. A quality trade (needs A/B on the
   real-geography quality suite), not waste elimination.
2. **Cost budget floor (4×r): no safe headroom.** Committed candidates land ~6–25% below
   it on average and at/above it in indirect terrain; a 3×r floor would save ~40% of pops
   but silently clip exactly the hard-terrain legs. Defensible only as an effort-tier
   (BOUNDED) knob, never as the default.
3. **Budget calibration: working as designed.** Per-step expansions correctly run
   uncalibrated at the floor (their radius is a step window, not the loop radius); the
   calibrated pool expansion's beyond-4×r overshoot is 2.4% of plan pops.
4. **The real measured "repeated work" is structural, not a constant**: ISO_GREEDY's
   conditional internal graph-native comparison re-planned the whole ladder in the
   sampled run (3 extra expansions, ~360 ms) after the blended ladder's ~1.0 s of iso
   work — and the blended result was discarded. It fires only when the blended verdict is
   below the accept bar and exists for a documented quality reason (the
   mallorca_30km_gravel_W undershoot-contraction class), so it is a product trade-off,
   not an optimization. Angularly, pops are near-uniform (±90° of the committed bearing
   ≈ 50%, ±150° ≈ 84–87%), so sector pruning would save ≈ proportionally to its angular
   width while changing candidate semantics — also a quality A/B, deprioritized.

Net: the expansion side has **no free lunch** — measured headroom before touching
candidate placement is ~10–15% of pops, and every larger win sits on a quality-relevant
boundary guarded by deliberate design.

## Part 6 — cost-space candidate windowing (roundTripCostWindow A/B)

Motivation (user insight): *air distance is often wrong, in particular in mountain
areas* — the step window [0.5, 1.65]×airRadius filters candidates by beeline, but
beeline misestimates profile path cost exactly where the planner needs honesty
(valley vs. wall).

### Why not contraction hierarchies

The CH idea (precompute shortcuts once, find better waypoints per profile, then
reroute with the regular point-to-point router) was evaluated and rejected on
structural grounds:

1. **CH metrics are query-time.** BRouter's cost function is a `.brf` profile
   (elevation penalty with hysteresis, way-tag cost factors, turn costs). One CH
   per profile = unbounded preprocessing; one shared CH answers the wrong question.
2. **Elevation hysteresis breaks shortcut composition.** The cost of A→C is not
   cost(A→B)+cost(B→C) when downhill-cost recovery depends on accumulated climb;
   shortcuts would need state that CH does not carry.
3. **Zero-preprocessing is the product model.** BRouter reads tiles and routes;
   there is no index build step to hang a CH off.
4. **The data CH would buy is already computed.** Every per-step isochrone
   expansion measures the true Dijkstra `costFromStart` of each pool candidate —
   the exact "profile-aware distance" CH exists to approximate. Using it is free.

### Design

`GraphNativeCandidateProvider.buildTemplates` gains a cost-window mode
(`RoutingContext.roundTripCostWindow`, default **off**):

- Estimate the step's target path cost from the pool: for each candidate inside
  the air window, scale measured cost to the target radius
  (`cost × targetAirRadius/airDist`; cost scales ~linearly along a radial leg),
  then take a **per-10°-bucket median** (36 buckets; ≥2 samples, else whole-pool
  median; ≥4 samples globally, else fall back to the plain air window).
- Accept candidates with relative cost error in [0.5, 1.65] against their
  bucket's estimate, plus air backstops [0.25, 2.5]×targetAirRadius (a candidate
  whose beeline is implausible for the step geometry is still rejected).
- Rank by |costRel − 1|. In flat terrain cost/air is constant and the mode
  degenerates exactly to the air window (unit-tested).

Harness: `CostWindowABTest` (integrationTest, BENCH_SEGMENTS-gated) runs
3 regions × {GREEDY, ISO_GREEDY} × radii {8km, 11.9km} × dirs {0,90,180,270} ×
{cost-window off, on}, gravel.brf, strict quality gate; reports the planner's own
quality metrics (composite/reuse/distRatio/dirDelta/self-intersections) + wall.

### v1 measurement (global median target, 96 plans, zero harness errors)

| region | terrain | Δcomposite | ΔdistErr | ΔstructCross | ΔwallMs | errors off→on |
|---|---|---|---|---|---|---|
| garmisch | alpine | **+0.015** | −0.016 | −0.125 | −1383 | 2→0 |
| grenoble | alpine valley | **−0.057** | +0.029 | +0.313 | — | 0→0 |
| dreieich | flat control | −0.003 | ~0 | ~0 | — | 0→0 |

- Garmisch: the 2 off-mode failures were hard quality-gate rejections
  (`INVALID_RETRACE, distance ratio 0.50`); cost mode produced valid loops there.
  One regression: GREEDY 8km dir0 composite 0.740→0.660.
- Grenoble failure signature: reuse spikes (10–20% vs 1.4–7% off) + loop
  undershoot (distRatio 1.03→0.74 etc.). Diagnosis: the *global* median target is
  biased by the dominant cheap direction (valley floor) → expensive-but-correct
  valley-wall candidates get rejected → planner locked into the valley corridor.

### v2 (per-bucket median = per-direction indirectness)

Identical design, but the target cost is the median *within the candidate's own
10° bucket* — the valley wall is judged against the wall, not the valley floor.

v2 A/B (same harness, same regions, `--no-build-cache` forced re-execution after
cache-replay was found to skip tile changes; 96 plans, build green):

| region | Δcomposite (v1→v2) | ΔdistErr | ΔstructCross | ΔwallMs | errors off→on |
|---|---|---|---|---|---|
| garmisch | **+0.013** | −0.007 | −0.063 | +1262 | 2→0 |
| grenoble | **−0.030** (was −0.057) | +0.053 | +0.125 (was +0.313) | −706 | 0→**2** |
| dreieich | 0.000 | −0.004 | 0.000 | −463 | 0→0 |

Reading:

- **Garmisch keeps the win.** Both formerly-rejected dir-270 plans
  (`INVALID_RETRACE 0.50`) now return valid loops (composite 0.750, reuse 0.2%);
  dir-180 composite 0.68→0.78, dir-90 0.71→0.79. Runtime outliers on exactly
  those fixed cells (41s/64s vs ≤27s off) — the ladder working harder where the
  alternative was *no loop at all*.
- **Grenoble halves but does not lose its regression.** distErr still +0.053 and,
  worse, ON mode now *fails hard* on dir-270 in both algorithms
  (`INVALID_RETRACE 0.46`) where OFF succeeded (0.95/1.01). Per-bucket medians
  fix within-direction honesty; they cannot fix the deeper asymmetry the window
  creates: in a corridor valley the cheap direction is also the one where a
  retrace-shaped plan assembles, so cost-windowing keeps favoring collapse
  topologies there.
- **Dreieich (flat): exact degeneracy confirmed on real terrain** (Δcomposite
  0.000) — off by default, the mode is inert where there is nothing to fix.

### Verdict

**Keep `roundTripCostWindow` default OFF; do not ship as default.** The mechanism
is real and well-tested (fixes genuinely broken alpine starts where beeline
windowing yields hard quality-gate rejections; byte-identical behavior when off;
exact degeneracy in flat terrain), but across 3 regions × 48 plans the mixed
outcome (alpine win, valley-corridor hard failures) does not justify defaulting
it on. Candidate follow-ups before revisiting: hybrid air+cost scoring instead of
pure cost replacement, or a topology guard that rejects candidate sets whose
median-cost directions all collapse onto one corridor (the retrace-attractor
detector).

### Unit tests

`GraphNativeCandidateProviderTest` (+4): ridge-adjacency accept/reject flip vs.
air mode; per-bucket honesty (wall-sector candidate at 1.6× its bucket's target
admitted though 4.6× the global median); exact degeneracy when cost = 1.3×air;
sparse-pool fallback to the air window. All green; golden loop signatures
byte-identical with the flag off.

## Conclusion

Bidirectional Dijkstra is a poor fit for *both* BRouter search loops:

- **Point-to-point** (`_findTrack` on master): measured gain ceiling ~1.2–1.6× over the
  current two-pass scheme, blocked by direction-/state-dependent costs, turn
  restrictions, and the dominance/memory machinery. The cheaper measured lever is an
  admissible heuristic in pass 1 (`pass2coefficient=1.0`): pass-1 pops −1.8–2.5×, wall
  ~−2×, identical cost for trekking.brf.
- **Greedy loop planner** (fork-master): the dominant primitive is the destination-less
  isochrone expansion (95%+ of pops); leg searches are small and already heuristic.
  Nothing for a bidirectional rewrite to win back.

## Reproduce

```
# point-to-point (master)
BENCH_SEGMENTS=$PWD/segments4 BENCH_HEAP=8g \
  gain :brouter-core:test --tests btools.router.BidirectionalSearchInvestigation -Pgain.diag

# loop planner (worktree at .worktrees/fork-master)
BENCH_SEGMENTS=$PWD/segments4 BENCH_HEAP=8g \
  gain :brouter-core:test --tests btools.router.LoopPlannerInstrumentation -Pgain.diag

# loop planner radial/cost pop histograms (Part 5; same test class, second method —
# sets -Diso.dump itself; timelines land in brouter-core/build/tmp/loopInstr/)
BENCH_SEGMENTS=$PWD/segments4 BENCH_HEAP=8g \
  gain :brouter-core:test \
  --tests btools.router.LoopPlannerInstrumentation.expansionSectorConcentration -Pgain.diag

# Part 6 cost-window A/B (worktree; integrationTest task — delete stale
# test-results/reports first, and pass --no-build-cache: the task cache does NOT
# track the rd5 tiles, so a cached replay would silently skip them)
rm -rf brouter-core/build/test-results/integrationTest brouter-core/build/reports/tests/integrationTest
BENCH_SEGMENTS=/path/to/segments4 BENCH_HEAP=8g \
  gain :brouter-core:integrationTest --tests btools.router.CostWindowABTest -Pgain.diag --no-build-cache
```

(`BENCH_HEAP` knob added to brouter-core/build.gradle's test task; without BENCH_SEGMENTS
the tests self-skip via JUnit Assume.)
