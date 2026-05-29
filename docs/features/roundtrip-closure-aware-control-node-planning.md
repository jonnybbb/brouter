---
parent: Features
---

# Round-Trip Closure-Aware Control-Node Planning Spec

Status: proposed implementation spec, May 2026 (revised after fresh literature-comparison review — see Literature Comparison Addendum below).

This spec is a handoff for an implementation agent. It translates the
round-trip routing literature review into concrete BRouter changes, with the
first implementation focused on improving the existing `GREEDY` and
`ISO_GREEDY` planners rather than replacing them.

## Summary

The current generated-loop implementation already follows the strongest lesson
from the literature: a generated loop should be built from graph-valid control
nodes, not from arbitrary geometric waypoints.

The main gap is that the greedy planner still commits a locally best next leg
before it knows whether that leg can close into a non-chaotic loop. This causes
failures where the distance is good but the final return leg crosses earlier
geometry many times.

The first implementation should make greedy planning closure-aware:

1. route several candidate next legs;
2. keep them ranked, not only the single best;
3. on late steps or when closure is near, probe the return leg for the top
   candidates;
4. score the candidate plus return as a complete loop;
5. commit the first candidate whose closed route passes the production quality
   gate;
6. only then shrink radius or fall back.

This keeps the existing architecture but changes the decision point from
"best next waypoint" to "best next control node that can still form a valid
closed walk".

## Research Mapping

The implementation should treat the papers as design evidence, not as a demand
to copy any one algorithm exactly.

### Greedy Dijkstra Control Vertices

The Jaszcz-style greedy Dijkstra algorithm repeatedly selects a next graph
vertex using distance-to-current and distance-to-previous terms, checks whether
returning to the start would complete the loop, and reduces sub-route distance
when needed.

BRouter already has this shape:

- `GraphNativeCandidateProvider` returns real graph nodes reached by bounded
  Dijkstra expansion.
- `CandidateScorer` includes sub-route distance fit, projected loop fit,
  direction, reuse, spread, previous-node separation, ISO metadata, and
  profile-hostility signals.
- `GreedyRoundTripPlanner` checks return feasibility after committing a leg.

Required improvement: when the chosen leg produces a rejected closure, try other
routed candidates from the same candidate set before changing radius.

### Isochrone-Contained Polygon Placement

The Lewis and Corcoran fixed-length round-trip work shows that reachable-region
information is crucial in coastal, island, mountain, and sparse networks.
Geometric circles alone place waypoints in regions that are close by air but
not actually reachable by the graph.

BRouter already uses this idea in two places:

- `runIsochroneExpansion()` builds profile-specific reachable frontier data.
- `ISO_GREEDY` blends a start-centered isochrone candidate pool with per-step
  graph-native candidates.

Required improvement: keep ISO as a candidate source and terrain signal, but do
not make pure geometric polygon waypoint placement the primary production path.

### Double-Path and Rod-Detour Families

The double-path and rod-detour papers suggest good future fallback algorithms:

- select a target `t`;
- route `s -> t`;
- penalize or poison the outbound path;
- route `t -> s`;
- accept the union only if the closed loop passes distance, overlap, profile,
  and shape checks.

These methods are useful in corridor-like networks where multi-waypoint greedy
can become chaotic. They should be added after closure-aware greedy has been
benchmarked, not mixed into the first change.

## Literature Comparison Addendum (May 2026 review)

This spec was reviewed against a fresh literature scan covering Jaszcz greedy
Dijkstra control vertices, Lewis and Corcoran fixed-length round-trip work
(2022 and 2024), Runamic rod-detour, and the Smart Running thesis. The review
confirmed the existing scope (closure-aware reranking) is the highest-leverage
change and identified two **orthogonal** levers that do not conflict with it.

### Confirmed: BRouter already exceeds the papers in three of four families

- Jaszcz greedy control-vertex scoring is already extended:
  `CandidateScorer.score()` has seven weighted terms (distance, loop
  feasibility, direction-fade, reuse, spread, prev-distance squared, iso
  bonus + hostility, contour-depth, contiguous-hostile-stretch) versus the
  paper's two. The Silesian `(d_prev - target)^2` term is at
  `CandidateScorer.previousDistancePenalty`.
- The Lewis and Corcoran isochrone-polygon method is **subsumed** by
  `IsochroneCandidateProvider`'s road-native frontier sampling: candidates
  are real graph nodes from the isochrone Dijkstra, not snapped polygon
  vertices. Polygon snapping is a regression and should not be ported.
- The eight-criterion "suitable loop" definition (closed, distance-valid,
  low repetition, low overlap, profile-valid, return-feasible,
  non-chaotic, terrain-aware, legal) is fully implemented across
  `RoundTripQualityGate`, `ReuseClassifier`, and `LoopQualityMetrics`.
- Profile-aware hostility activation (`CandidateScorer.setHostilityActive`)
  correctly disables the iso-hostility term for MTB/gravel profiles whose
  cost/airDist baseline is ~9 — the paper does not consider this and would
  collapse the candidate pool for off-road profiles.

### Identified: two orthogonal levers added below

1. **Isochrone-asymmetry initial bearing** (added below as Phase 2).
   `runIsochroneExpansion()` already computes a 36-bucket frontier table
   `[direction_deg, airDist_m, cost, hits, ilon, ilat]` per
   `IsochroneExpansionResult`. The Lewis and Corcoran 2024 paper recommends
   using the most-distant frontier point to find an accessible initial
   bearing — exactly the asymmetry signal needed for coastal, island,
   river-valley, and sparse-rural networks. The data exists; only the
   consumer that biases initial direction is missing. This is **orthogonal
   to closure-aware reranking** — it changes which sector is explored, not
   which candidate is committed.

2. **Direction-change density / zigzag shape metric** (added under Future
   Work as `SHAPE_DENSITY_METRIC`). Hairpin count and self-intersections are
   pass/fail gate signals; moderate-turn density (e.g. 30 turns of >60
   degrees per 50 km) is not measured. Likely unnecessary once closure-aware
   reranking is in place — the existing shape signals plus the rerank
   pressure should catch most chaos — but flagged for evaluation after
   Phase 1 corpus evidence.

### Reframed: `DOUBLE_PATH` / `ROD_DETOUR` are corridor-terrain algorithms

Earlier framing suggested these might primarily help short loops where
multi-step greedy creates combinatorial pressure. The literature is clearer:
they shine where **topology forces close-in shapes** regardless of length
(coast, valley, river, sparse rural). Loop length is a secondary effect of
the same mechanism. Updated under Future Work below.

### Confirmed NOT to port from the papers

- Polygon-on-isochrone vertex placement — regression vs. road-native
  frontier sampling.
- Random initial bearing with 8 x 45 degree variants (TrailRouter-style).
  AUTO competition covers this implicitly; multiplying trials explodes
  runtime budget for no quality win.
- Degree-one leaf removal as preprocessing. Would break
  `SCENIC_OUT_AND_BACK` semantics already encoded in `ReuseClassifier`
  (terminal scenic spurs to viewpoints, peninsulas, dead-end harbour
  roads).
- Held-Karp / 2-opt waypoint reordering on the generated control-node set.
  With 5-15 control nodes already produced by a planner that optimizes
  locally per step, classical TSP refinement rarely changes the order in a
  way that improves the loop. Deferred indefinitely without explicit
  evidence.

## Current Architecture

Relevant production classes:

- `RoutingEngine.doRoundTrip()`
  resolves explicit-via mode, algorithm selection, and AUTO candidate
  competition.
- `RoutingEngine.doGreedyRoundTrip()`
  builds the provider, selects sub-route count, runs `GreedyRoundTripPlanner`,
  and adopts the planner track.
- `GraphNativeCandidateProvider`
  generates per-step graph-native candidates from bounded Dijkstra expansion.
- `IsochroneCandidateProvider`
  provides start-centered ISO candidates for `ISO_GREEDY`.
- `BlendedCandidateProvider`
  mixes ISO and graph-native candidates.
- `CandidateScorer`
  scores candidate endpoints before and after the candidate leg has been
  routed.
- `GreedyRoundTripPlanner`
  incrementally builds legs, checks return feasibility, and creates the final
  route.
- `RoundTripQualityGate`
  applies hard acceptance checks: closure, distance sanity, beelines, ferries,
  self-intersections, hairpins, paved-profile hostility, and reuse semantics.

Important current behavior:

- `GREEDY` and `ISO_GREEDY` do not place a full unordered waypoint set.
- Candidate order is the construction history.
- Fixed waypoint/probe/iso placement already sorts generated directions with
  `sortDirectionsForLoop()`.
- Explicit user vias must keep user order and must not be reordered by this
  work.

## Problem

The planner may find candidates that look good locally:

- next leg distance is close to the sub-route target;
- route cost is reasonable;
- endpoint is not too close to start or previous waypoint;
- partial route has acceptable reuse;
- projected total length looks plausible.

But a local candidate is not actually good unless the resulting closed graph
walk is good. The current flow can commit a leg, route the return leg, discover
that the closed loop is rejected for chaotic shape, undo the leg, shrink the
radius, and retry. It does not first try the next-best routed candidate from the
same candidate pool.

This is especially visible on longer real GPX-like loops where the final return
leg can cross previous legs many times while still hitting the target distance.

## Goals

- Improve `GREEDY` and `ISO_GREEDY` without replacing their core model.
- Use graph-native control nodes as the primary abstraction.
- Make late-step candidate selection closure-aware.
- Try alternate routed candidates before shrinking radius.
- Keep `RoundTripQualityGate` as the final hard acceptance authority.
- Preserve deterministic output.
- Keep runtime bounded and suitable for production.
- Add telemetry that makes candidate and closure behavior auditable.
- Validate with focused unit tests and the real GPX corpus harness.

## Non-Goals

- Do not implement a global optimal cycle solver.
- Do not relax `RoundTripQualityGate` to hide bad loops.
- Do not make geometric polygon waypoints the default production strategy.
- Do not change explicit user-via semantics.
- Do not remove forced algorithms such as `WAYPOINT`, `ISOCHRONE`, `GREEDY`, or
  `ISO_GREEDY`.
- Do not introduce external libraries.
- Do not implement `DOUBLE_PATH` or `ROD_DETOUR` in the first change.

## Design Decision

Keep the existing algorithm hierarchy:

```text
AUTO
  -> ISO_GREEDY first
  -> GREEDY if ISO_GREEDY fails or is weak
  -> WAYPOINT/probe fallback only if greedy variants fail
```

Improve the internals of `GREEDY` and `ISO_GREEDY` by changing candidate
commitment:

```text
old:
  score candidates
  route top candidates
  commit single best candidate
  check return
  if closed loop rejected, undo and shrink radius

new:
  score candidates
  route top candidates
  keep ranked routed candidates
  try candidates in order
  when closure is relevant, probe return for each candidate
  commit the candidate that can form a valid closed loop
  shrink radius only after the same candidate set cannot produce progress
```

## Required Behavior

### Candidate Endpoint Contract

A candidate endpoint is suitable only if all applicable checks hold:

- the endpoint is graph-reachable from the current point;
- the routed candidate leg is profile-compatible;
- the candidate leg does not introduce excessive reuse or self-intersection;
- the endpoint is not just a collapse back toward the previous point;
- a return path to the start exists when closure is relevant;
- the closed route can satisfy distance tolerance;
- the closed route passes the production quality gate.

Do not treat geometric position alone as proof of suitability.

### Early Steps

For early steps where closure is clearly out of reach, the planner may keep the
current lower-cost behavior:

- route a bounded top-k candidate set;
- sort by routed score;
- detail and commit the best candidate that passes per-leg checks;
- if that candidate cannot be detailed or fails per-leg hostility checks, try
  the next routed candidate before shrinking radius.

This avoids paying return-Dijkstra cost too early.

### Late Steps

On late steps, closure quality should drive selection.

A step is late when either:

- `step >= subRouteCount - 1`, or
- `totalDistance + candidateDistance + estimatedReturn` is within the distance
  tolerance window, or
- the existing return-skip rule would no longer skip return evaluation.

For late steps, probe return for the top closure candidates before committing.
The probe must include the candidate leg in the reference track so the return
router is discouraged from retracing the just-selected leg.

### Rejected Closure Retry

If a candidate's closed loop is rejected by `RoundTripQualityGate`, the planner
must try the next routed candidate from the same step/attempt before changing
radius.

This applies especially to rejections such as:

- too many self-intersections;
- too many hairpins;
- profile-hostile final route;
- excessive reuse or invalid retrace classification.

After all probed candidates fail, the planner may shrink radius or continue
with existing fallback behavior.

### Distance Overshoot

If a candidate plus return is too long, do not immediately shrink radius if
untried candidates remain. Another candidate may have a shorter return path.

After all closure-probed candidates overshoot, shrink radius using the existing
backoff rule.

### Fallback Track

Keep best-fallback behavior, but distinguish fallback from accepted output:

- best fallback may be updated with the closest closed route by distance;
- fallback must still be marked degraded if it fails the quality gate;
- production success must require a non-degraded quality-gate pass.

## Implementation Phases

Work is sequenced into three phases with explicit gating between them.
Subsequent phases must not begin until the prior phase's corpus evidence is
captured and committed. This discipline exists because past incremental
patches to the greedy planner each fixed one symptom but introduced worse
regressions (destroying loop shape, creating gaps, collapsing distance) —
see `[[feedback_challenge_suggestions]]`.

### Phase 0 — Baseline Capture (prerequisite)

Before any Phase 1 code merges, the current `master` (or pre-change branch)
behaviour must be captured as the reference point for all subsequent
comparisons.

Required artifacts:

- A JSON file `docs/features/roundtrip-closure-aware-baseline.json`
  containing, per algorithm variant (`greedy` and `iso_greedy` measured
  separately):
  - accepted-route count;
  - average GPX-similarity score;
  - average distance ratio;
  - radial-profile mean absolute error;
  - max-radius delta;
  - farthest-bearing delta;
  - route-reuse fraction;
  - rejection-reason distribution (count per gate-reject reason);
  - median and p95 planner wall-clock per loop.
- The same artifacts for the known 100 km Basel long-loop case (single
  loop, both variants).

The artifact is generated by re-running the existing GPX similarity harness
against the small Basel corpus (see commands below in Acceptance Criteria
section) with telemetry capture enabled, on the commit that is the parent
of the first Phase 1 change.

Why this phase exists: `[[project_phase2v3_pipeline]]` notes that overall
pass-rate alone misleads — `greedy` and `iso_greedy` move independently and
must be evaluated separately. Without a per-variant baseline, Phase 1
acceptance cannot be judged.

Gating: no Phase 1 implementation code merges until the baseline JSON is
committed.

### Phase 1 — Closure-Aware Reranking

This is the original scope of the spec. The remaining Steps 1-7 below
define this phase in detail. The goal is to change the planner's decision
point from "best next waypoint" to "best next control node that can still
form a valid closed walk", without changing the algorithm hierarchy,
quality gate, or provider selection.

#### Step 1: Preserve Existing Gate and Providers

Do not change `RoundTripQualityGate` thresholds in this implementation unless a
test proves a threshold is objectively wrong. This work should make the planner
avoid bad loops, not accept them.

Do not change provider selection in `RoutingEngine.buildCandidateProvider()`.
`GREEDY` should continue to use `GraphNativeCandidateProvider`; `ISO_GREEDY`
should continue to use `BlendedCandidateProvider`.

#### Step 2: Keep Ranked Routed Candidates

In `GreedyRoundTripPlanner`, replace the single `ScoredRoute accepted` decision
with a list:

```java
List<ScoredRoute> routedCandidates = new ArrayList<>();
```

For every routable candidate:

- compute the existing routed scorer score;
- compute cost-per-meter;
- compute tentative partial self-intersections;
- store all values on `ScoredRoute`;
- add it to `routedCandidates`.

Sort by `routedScore` ascending.

Recommended `ScoredRoute` fields:

```java
OsmTrack track;
MatchedWaypoint toMwp;
double routeDistance;
double visitedRatio;
boolean fromIsoCandidate;
double routedScore;
int candidateIndex;
int tentativeSelfIntersections;
int routedLegWorstHostileMeters;
```

If `PARTIAL_SELF_INTERSECTION_WEIGHT` and
`countTentativeSelfIntersections()` already exist, keep them. They are a useful
early steering signal but are not sufficient for closure-leg chaos.

#### Step 3: Add Candidate Trial Loop

After routing and sorting candidates, try them in order:

```text
for candidate in routedCandidates:
  detail candidate leg
  reject candidate if detailed leg is invalid
  reject candidate if paved-profile hostile stretch exceeds gate cap

  if closure is not relevant:
    commit candidate and continue to next step

  probe return from candidate endpoint to start
  if no return route:
    try next candidate

  build finalTrack = previousSegments + candidateLeg + returnLeg
  evaluate finalTrack with RoundTripQualityGate
  if accepted:
    commit candidate and return leg
    populate result
    return success

  update degraded fallback if distance error is best so far
  undo candidate state
  try next candidate

after all candidates fail:
  shrink/backoff according to observed failure mode
```

The current manual undo code can be extracted into a helper rather than replaced
with deep copies. The implementation must correctly restore:

- `segments`;
- `totalDistance`;
- `visitedEdgeCounts`;
- `visitedEdgeFirstPos`;
- `waypointStack`;
- `currentMwp`;
- `prevIlon`;
- `prevIlat`;
- accepted source counters.

#### Step 4: Probe Closure Without Premature Mutation

When probing closure for a candidate:

1. detail the candidate leg first;
2. find the actual end node from the detailed track's last node;
3. match that endpoint to a `MatchedWaypoint`;
4. build a reference track that includes previous segments plus the candidate
   leg;
5. route return from candidate endpoint to start using that reference track;
6. detail the return leg before quality evaluation;
7. merge previous segments, candidate leg, and return leg into a final track;
8. call `qualityGateReason(finalTrack, desiredDistance)`.

Do not add the return leg to `segments` or visited-edge counters until the final
track has passed the gate.

Recommended helper shape:

```java
private ClosureProbe probeClosure(
    List<OsmTrack> acceptedSegments,
    ScoredRoute candidate,
    MatchedWaypoint candidateEnd,
    MatchedWaypoint startMwp,
    double totalBeforeCandidate,
    double desiredDistance,
    long deadline)
```

Recommended `ClosureProbe` fields:

```java
OsmTrack returnTrack;
OsmTrack finalTrack;
double closedDistance;
double distanceError;
String rejectReason;
int selfIntersections;
double finalReuseRatio;
int worstHostileMeters;
double closureScore;
```

#### Step 5: Closure Score

Use hard gate acceptance first. The closure score is only a ranking and
telemetry value among candidate closures.

Initial scoring formula:

```text
closureScore =
    candidate.routedScore
  + 2.0 * (distanceError / tolerance)
  + 0.5 * selfIntersections
  + 3.0 * finalReuseRatio
  + 0.3 * returnDistance / subTarget
  + profileHostilityPenalty
```

Where:

- lower is better;
- `distanceError` is absolute closed-distance error divided by desired distance;
- `selfIntersections` comes from `RoundTripQualityGate.countSelfIntersections`;
- `finalReuseRatio` comes from `GreedyRoundTripPlanner.finalTrackReuseRatio`;
- `profileHostilityPenalty` is zero for non-paved profiles and normalized from
  `RoundTripQualityGate.worstContiguousHostileMetersPaved` for paved profiles.

Do not accept a candidate just because its closure score is lowest. Acceptance
still requires `RoundTripQualityGate.evaluate(...).isAccepted()`.

#### Step 6: Runtime Budget

Keep return probing bounded.

Recommended constants:

```java
private static final int CLOSURE_RERANK_TOP_K = 4;
private static final int CLOSURE_RERANK_TOP_K_EARLY = 1;
private static final int MAX_RETURN_PROBES_PER_STEP = 4;
```

Rules:

- early steps should normally probe zero or one return path, matching current
  behavior;
- late steps may probe up to four candidate closures;
- stop probing when the planner deadline is reached;
- if the candidate list has fewer than the cap, probe only available candidates.

If runtime regressions show up, reduce `CLOSURE_RERANK_TOP_K` before weakening
quality gates.

#### Step 7: Diagnostics and Telemetry

Add structured telemetry to `RoundTripResult`:

- `closureCandidatesProbed`;
- `closureCandidatesAccepted`;
- `closureRejectedByGate`;
- `closureRejectedSelfIntersections`;
- `closureRejectedDistance`;
- `closureRerankWins`;

Definitions:

- `closureCandidatesProbed`: number of candidate return legs routed.
- `closureCandidatesAccepted`: number of probed closures that passed the gate.
  Usually 0 or 1 for a planner run.
- `closureRejectedByGate`: number of probed closures rejected by
  `RoundTripQualityGate`.
- `closureRejectedSelfIntersections`: number rejected with a reason containing
  self-intersections.
- `closureRejectedDistance`: number outside tolerance or outside distance-ratio
  limits.
- `closureRerankWins`: increment when the accepted closure candidate was not
  the locally best routed candidate.

Add diagnostic strings only as supporting detail. Tests should prefer structured
fields where possible.

Recommended diagnostic examples:

```text
step 5: closure probe cand=0 rejected: route has 21 self-intersections
step 5: closure probe cand=1 accepted: total=100243m error=0.2% selfX=2
step 5: closure rerank selected cand=1 over local cand=0
```

### Phase 2 — Isochrone-Asymmetry Initial Bearing

This phase is **orthogonal** to Phase 1: it changes which side of the start
is explored first, not which candidate is committed within a side. The two
stack cleanly.

The Lewis and Corcoran 2024 paper observes that on coastal, island,
mountain, or sparse-rural graphs, geometric "circle around the start" placement
puts waypoints in regions that are close by air but not actually reachable
by the graph. The fix is to inspect the isochrone's reachable frontier
**asymmetry** and bias the initial bearing toward the most-reaching sector.

BRouter already computes the necessary data:

- `RoutingEngine.runIsochroneExpansion()` produces a 36-bucket
  `IsochroneExpansionResult.frontier` table with entries
  `[direction_deg, airDist_m, cost, hits, ilon, ilat]`.
- `IsochroneCandidateProvider` already uses the per-bucket `hits` count as
  a confidence signal (sparse buckets with `hits < 3` are treated as
  one-shot dead-ends).

The missing consumer is a "best-reaching bearing" computation that biases
the initial direction preference when no explicit user direction is given.

#### Step 8: Best-Reaching Bearing Computation

Define indirectness per bucket:

```text
indirectness(bucket) = bucket.cost / bucket.airDist
```

A "best-reaching" bucket is one minimizing indirectness subject to two
quality thresholds (matching `IsochroneCandidateProvider`'s existing
sparse-bucket and reach rules):

```text
bucket.airDist >= 0.6 * searchRadius
bucket.hits    >= 3
```

If two or more buckets satisfy the thresholds and tie on indirectness,
break ties by lowest bucket index (deterministic output).

If no bucket satisfies the thresholds, return "no bias available" — the
caller must fall back to Phase-1 behaviour (existing direction selection)
unchanged.

#### Step 9: Direction Bias Application

In `RoutingEngine.doRoundTrip()` (or the closest downstream caller of
`runIsochroneExpansion` for the `ISO_GREEDY` path), after the expansion
returns:

1. If the request specifies an explicit user direction, **skip** this step
   entirely — user direction is preserved verbatim.
2. If the algorithm is `GREEDY` (no isochrone available), **skip** this
   step.
3. Compute the best-reaching bearing from the frontier table.
4. If "no bias available", proceed with Phase-1 direction selection
   unchanged.
5. Otherwise, set the planner's `DirectionPreference` to the best-reaching
   bearing with the existing fade behaviour (step-1 full weight, step-2
   half, zero thereafter — see `CandidateScorer.directionFade`).

The bias affects only step-1 and step-2 candidate scoring. It does **not**
constrain the rest of the loop — closure-aware reranking (Phase 1) handles
all subsequent candidate decisions.

#### Step 10: Phase 2 Telemetry

Add to `RoundTripResult`:

```java
boolean isoAsymmetryBearingApplied;
double  isoAsymmetryBearingDegrees;          // NaN when not applied
double  isoAsymmetryBestBucketIndirectness;  // NaN when not applied
int     isoAsymmetryBestBucketHits;          // -1 when not applied
int     isoAsymmetryBestBucketAirDistMeters; // -1 when not applied
```

These fields make it auditable whether the bias fired and whether it picked
a quality bucket. They are required because aggregate corpus metrics alone
do not tell us **why** a coastal sentinel improved or did not.

## Acceptance Criteria

This section is the single authoritative acceptance specification for the
work. It supersedes earlier separate "Acceptance Tests", "GPX Corpus
Evaluation", and "Done Criteria" sections. Acceptance is organized per
phase with hard pass/fail criteria, telemetry/observability requirements,
required tests, runtime bounds, and explicit gating conditions for the
next phase.

A phase is considered complete only when **all** of its hard criteria
hold, **all** of its required telemetry fields are populated, **all** of
its tests pass on CI, and the gating artifacts for the next phase are
committed.

### Phase 0 — Baseline Capture

#### Phase 0 Hard Criteria

- Baseline JSON artifact `docs/features/roundtrip-closure-aware-baseline.json`
  exists in the repo, generated from the commit immediately preceding the
  first Phase 1 change.
- Per algorithm variant (`greedy` and `iso_greedy` measured independently,
  per `[[project_phase2v3_pipeline]]`), the artifact records:
  - `acceptedRoutes` (int) and `eligibleLoops` (int);
  - `averageSimilarityScore` (double, 0..1);
  - `averageDistanceRatio` (double, ideal 1.0);
  - `radialProfileMeanAbsoluteError` (double, meters);
  - `maxRadiusDelta` (double, meters);
  - `farthestBearingDelta` (double, degrees);
  - `averageRouteReuse` (double, 0..1);
  - `rejectionReasonDistribution` (map<string, int>);
  - `medianPlannerWallClockMs` and `p95PlannerWallClockMs` (long).
- A separate `known100kmBasel` entry records the same fields for the
  100 km Basel long-loop case (single loop, both variants).
- The artifact is reproducible: re-running the harness on the same commit
  with the same corpus produces identical numerical values.

#### Phase 0 Required Commands

```bash
./gradlew :brouter-core:test \
  -Dloop.tests=true \
  -Dgpx.dir=tmp/gpx/Basel \
  -Dgpx.loop.limit=6 \
  -Dgpx.loop.maxKm=90 \
  -Dloop.algorithms=greedy,iso_greedy \
  -Dloop.baseline.capture=docs/features/roundtrip-closure-aware-baseline.json \
  --tests "btools.router.GpxLoopSimilarityTest"
```

If `-Dloop.baseline.capture` is not yet implemented, capture numbers from
harness output and commit them as a hand-written JSON of the same shape.
The harness flag may be added as part of Phase 1.

#### Phase 0 Gating for Phase 1

No Phase 1 implementation code may merge until the baseline JSON is
committed.

### Phase 1 — Closure-Aware Reranking

#### Phase 1 Hard Criteria

- `GREEDY` and `ISO_GREEDY` keep a ranked list of routed candidates per
  step (not a single accepted candidate). `ScoredRoute` fields populated
  per Step 2.
- On late steps (definition in Required Behavior, "Late Steps"), the
  planner probes return for up to `CLOSURE_RERANK_TOP_K = 4` candidates
  per step with global cap `MAX_RETURN_PROBES_PER_STEP = 4`.
- A candidate's closed loop is evaluated by
  `RoundTripQualityGate.evaluate(...)` before commitment. Acceptance
  requires `verdict.isAccepted() == true`.
- After a closure rejection, the planner tries the next routed candidate
  before any radius shrink.
- After all closure-probed candidates fail, radius backoff proceeds via
  the existing rule. No new backoff logic is introduced.
- Explicit-via mode is unaffected. No closure-aware reranking applies to
  user-supplied vias.
- Deterministic output: two runs on the same inputs produce byte-identical
  accepted tracks. Tie-breaks use stable indices.
- Undo state restoration is complete after every failed candidate trial:
  `segments`, `totalDistance`, `visitedEdgeCounts`,
  `visitedEdgeFirstPos`, `waypointStack`, `currentMwp`, `prevIlon`,
  `prevIlat`, and accepted source counters all return to pre-trial
  values.
- Fallback degraded marking preserved: the planner does not output a
  gate-rejected track as a non-degraded success.
- `RoundTripQualityGate` thresholds are not changed.
- Provider selection in `RoutingEngine.buildCandidateProvider()` is not
  changed.

#### Phase 1 Required Telemetry on `RoundTripResult`

- `closureCandidatesProbed` (int)
- `closureCandidatesAccepted` (int)
- `closureRejectedByGate` (int)
- `closureRejectedSelfIntersections` (int)
- `closureRejectedDistance` (int)
- `closureRerankWins` (int)
- Per-step diagnostic strings describing rerank decisions (Step 7).

#### Phase 1 Required Unit Tests

Under `brouter-core/src/test/java/btools/router/`:

1. `GreedyRoundTripPlannerTest`:
   - Candidate list ordering — routed candidates sorted by `routedScore`;
     partial-self-intersection penalty affects ordering.
   - Alternate retry — first candidate fails post-detail or per-leg
     hostility; second candidate is tried before any radius shrink.
   - Closure rejection retry — first candidate's closed route is
     rejected for self-intersections; second is probed and accepted;
     `closureRerankWins == 1`.
   - Distance overshoot retry — first candidate plus return overshoots
     tolerance; second is within tolerance; no premature radius shrink.
   - Deadline handling — closure probing stops at planner deadline;
     `RoundTripResult` is marked degraded if no candidate accepted.
2. `RoundTripUserViaTest`: user-via flow does not enter closure-aware
   reranking; user-supplied waypoint order preserved.
3. `RoundTripContractTest`: deterministic output across repeated runs.
4. `RoundTripQualityGateTest`: existing gate behaviour unchanged.

Where no clean test seam exists, extract package-private helpers from
`GreedyRoundTripPlanner` rather than building integration-only tests.

#### Phase 1 Required Integration Commands

```bash
./gradlew :brouter-core:test -Dloop.tests=true \
  --tests "btools.router.GreedyRoundTripPlannerTest"
```

Basel small-corpus GPX similarity (production variants only):

```bash
./gradlew :brouter-core:test \
  -Dloop.tests=true \
  -Dgpx.dir=tmp/gpx/Basel \
  -Dgpx.loop.limit=6 \
  -Dgpx.loop.maxKm=90 \
  -Dloop.algorithms=greedy,iso_greedy \
  --tests "btools.router.GpxLoopSimilarityTest"
```

Known 100 km Basel long-loop:

```bash
./gradlew :brouter-core:test \
  -Dloop.tests=true \
  -Dgpx.dir=tmp/gpx/Basel \
  -Dgpx.loop.limit=1 \
  -Dgpx.loop.minKm=100 \
  -Dgpx.loop.maxKm=101 \
  -Dloop.algorithms=greedy \
  --tests "btools.router.GpxLoopSimilarityTest"
```

#### Phase 1 Required Corpus Outcomes vs. Phase 0 Baseline

Per algorithm variant, the Phase 1 measurement must satisfy:

- `acceptedRoutes` not less than baseline.
- `averageSimilarityScore` not less than baseline minus 0.02 (no
  material regression).
- `averageDistanceRatio` within `[baseline - 0.05, baseline + 0.05]`.
- `rejectionReasonDistribution`: combined count of chaotic-shape
  rejections (self-intersections + hairpins) strictly less than
  baseline.
- Known 100 km Basel long-loop: either accepted **or** with strictly
  fewer self-intersections than baseline. If neither, the residual
  failure mode must be documented in the PR with subTrack indices,
  candidate scores, and gate rejection reason.

If `acceptedRoutes` increases while `averageSimilarityScore` decreases by
more than 0.02, the regression must be investigated by manual track
inspection before merge. Per `[[feedback_challenge_suggestions]]`, do
not optimize a single score at the expense of route plausibility.

#### Phase 1 Runtime Budget

- Median planner wall-clock per loop: not more than 1.5x baseline.
- p95 planner wall-clock per loop: not more than 2.0x baseline.

If exceeded, reduce `CLOSURE_RERANK_TOP_K` before relaxing any quality
gate or skipping closure probing.

#### Phase 1 Gating for Phase 2

No Phase 2 implementation code may merge until Phase 1 corpus metrics
are captured and committed as
`docs/features/roundtrip-closure-aware-phase1.json` (same shape as the
baseline artifact).

### Phase 1 Investigation Results (May 2026)

Two attempts at Phase 1 Step 3 were measured against the Phase 0 Basel
baseline (commit `5ba4c4a6`, n=6 loops × 2 algorithms). **Both regressed
beyond the spec's -0.02 similarity threshold.** Reverted without
commit. The architectural assumption "try alternate candidate at the
same radius before shrinking radius" is contradicted by this corpus.

#### Attempt 1 — Step 3a only (alternate retry on pre-closure validation)

Iterate ranked candidates; if validation (detail/metadata/hostility)
fails for the locally best, try next at the same radius instead of
shrinking.

Results on Basel small corpus:

| | Baseline | Step 3a | Δ |
|---|---|---|---|
| GREEDY avg similarity | 0.852 | 0.837 | **-0.015** |
| ISO_GREEDY avg similarity | 0.850 | 0.811 | **-0.039** ✗ |

ISO_GREEDY exceeded the -0.02 threshold. Reverted.

#### Attempt 2 — Step 3a + scorer divergence fix + Step 3b (closure probe)

After committing the scorer divergence fix (commit `c873e12e`, no
behavior change on baseline), implemented the full Step 3 trial loop
with closure-aware probe per spec Steps 3-4. Atomic commit-on-accept;
no rollback paths.

Results on Basel small corpus:

| | Baseline | Step 3 (full) | Δ |
|---|---|---|---|
| GREEDY avg similarity | 0.852 | 0.816 | **-0.036** ✗ |
| ISO_GREEDY avg similarity | 0.850 | 0.764 | **-0.086** ✗ |

10 of 12 per-loop scenarios regressed. Two improved slightly. Runtime
~70% slower (Basel 45 s → 76 s). Reverted.

#### Root cause: shrink-and-retry is a candidate-pool diversifier

The legacy `cand[0] fails → shrink radius → regenerate candidates from
scratch at smaller radius` was a hidden architectural win, not just a
last-resort fallback. At a smaller radius the candidate provider samples
different points in the graph; the new candidate pool has structurally
different geometric options, often producing a better overall loop
shape.

`Try alternate at the same radius` keeps the planner stuck in the same
candidate cluster. On Basel, the locally-best candidate is shape-optimal
by routedScore; alternates ranked lower precisely because their shape is
worse (worse spread, worse direction, more reuse). When the planner
picks them, the resulting routed loop has lower similarity to the GPX
reference — even when the closure passes the production gate.

Concretely on the Basel corpus, ~20 % of step decisions trigger
alternate selection (driven by detail-time hostility failures the
scorer cannot predict from single-pass tracks — see the scorer fix
above for partial mitigation). The alternates that survive the gate
produce visibly worse-shaped loops; aggregating across 6 loops the
similarity drops 0.04–0.09 per variant.

#### Implication for the spec

The spec's Step 3 design assumption — *alternate retry before shrink
is net-positive for loop quality* — is empirically false on this
corpus. Two distinct mechanisms produce regression:

1. **Same-radius alternates are systematically worse-shape** than
   smaller-radius regenerated candidates (the scorer ranks alternates
   lower for shape reasons).
2. **Closure probing rejects candidates that the OLD post-commit gate
   would have rejected too**, but in OLD the planner then shrank and
   tried a smaller-radius candidate; in NEW it tries a worse-shape
   same-radius alternate.

Both mechanisms compound. The scorer-divergence fix (already committed)
addresses neither — it improves ranking accuracy but doesn't change the
fundamental "alternates at same radius are shape-suboptimal" reality.

#### Revised Step 3 design (proposed; not yet implemented)

Three options worth exploring before re-attempting Step 3:

**A. Closure probe without alternate retry** — atomically commit
   cand[0] only after a successful closure probe (preserving the
   "no rollback" win). On any failure (validation, probe, gate),
   shrink radius as the legacy code did. Smaller, safer change; only
   gain is removing the rollback code path.

**B. Closure-quality-aware shrink/alternate decision** — when cand[0]
   fails closure for a *distance* reason (too long/too short), the
   gate's verdict suggests the radius itself is wrong → shrink. When
   cand[0] fails closure for a *shape* reason (hostility, self-
   intersections, reuse), the alternates at the same radius have
   different shapes → try alternates first. Distinguishes the failure
   mode driving the next step.

**C. Closure probe + alternate-bounded-by-score-delta** — try
   alternates only when their `routedScore - bestRoutedScore` is below
   some threshold (e.g. 0.2). When all alternates score much worse
   than the locally-best, shrink instead. Heuristic but pragmatic;
   tunes the trade-off between alternate-retry and shrink-and-retry.

Recommendation: implement **A** first as the smallest reviewable change
and re-measure. If it preserves the baseline (likely), it adds the
closure-probe correctness guarantee without behavior regression. **B**
or **C** can layer on top once **A** has corpus evidence.

#### What stays from the failed attempts

- The **scorer divergence fix** (commit `c873e12e`) is correct and
  committed. It improves the scorer's hostility ranking without
  changing baseline behavior. Foundation for any future Step 3
  design that does rely on alternate-retry.
- The **Step 2 ranked-candidate list** (commit `755b0d3a`) is the
  structural foundation for any of the revised designs (A, B, C
  above). It stays committed.
- The **closure-probe helper (`probeClosure`) and ClosureProbe data
  class** designs from the reverted Step 3 attempt are reusable for
  any revised design; the spec's Step 4 description is unchanged.

### Phase 2 — Isochrone-Asymmetry Initial Bearing

#### Phase 2 Hard Criteria

- The best-reaching bearing computation (Step 8) runs after
  `runIsochroneExpansion` for `ISO_GREEDY` only.
- The bias applies only when `directionPreference == ANY` (no explicit
  user direction).
- The bias applies only at planner steps 1 and 2 via the existing
  `CandidateScorer.directionFade` mechanism. Step 3 and later are
  unaffected.
- Threshold rule: bucket included only if `airDist >= 0.6 *
  searchRadius` and `hits >= 3`. Lower-bound thresholds match
  `IsochroneCandidateProvider`'s existing sparse-bucket rule.
- "No bias available" branch is taken when no bucket meets the
  thresholds. Behaviour in that branch is identical to Phase 1.
- Deterministic tie-break: lowest bucket index wins on equal
  indirectness.

#### Phase 2 Required Telemetry on `RoundTripResult`

- `isoAsymmetryBearingApplied` (boolean)
- `isoAsymmetryBearingDegrees` (double; `NaN` when not applied)
- `isoAsymmetryBestBucketIndirectness` (double; `NaN` when not applied)
- `isoAsymmetryBestBucketHits` (int; `-1` when not applied)
- `isoAsymmetryBestBucketAirDistMeters` (int; `-1` when not applied)

#### Phase 2 Required Unit Tests

Under `brouter-core/src/test/java/btools/router/`. Tests may live in a
new `IsoAsymmetryBearingTest` or be added to `RoutingEngineTest`:

1. Symmetric synthetic frontier — no asymmetry; bias is absent or
   matches existing default direction.
2. Asymmetric synthetic frontier — one quadrant has lower indirectness
   and meets thresholds; applied bearing matches angular center of that
   quadrant.
3. Sparse-bucket fallback — no bucket meets `hits >= 3`; bias not
   applied; behaviour matches Phase 1.
4. Reach-floor fallback — no bucket meets `airDist >= 0.6 *
   searchRadius`; bias not applied; behaviour matches Phase 1.
5. Explicit-direction precedence — when a user direction is given, the
   bias is not applied regardless of frontier asymmetry.
6. `GREEDY`-mode isolation — the bias never fires for `GREEDY`, only
   for `ISO_GREEDY`.

#### Phase 2 Required Integration Commands

Same Basel small-corpus command as Phase 1. Add a coastal sentinel slice
if one exists in the local corpus:

```bash
./gradlew :brouter-core:test \
  -Dloop.tests=true \
  -Dgpx.dir=tmp/gpx/coastal-sentinel \
  -Dloop.algorithms=iso_greedy \
  --tests "btools.router.GpxLoopSimilarityTest"
```

#### Phase 2 Required Corpus Outcomes vs. Phase 1

- `iso_greedy` `acceptedRoutes` not less than Phase 1 measurement.
- `iso_greedy` `averageSimilarityScore` not less than Phase 1 minus 0.02.
- For coastal/island/sparse-rural sentinel cases (if a tagged sentinel
  subset exists), `iso_greedy` similarity strictly greater than Phase 1.
- `greedy` variant metrics unchanged within measurement noise (this
  phase does not touch `GREEDY`).

#### Phase 2 Gating for Future Work

No Future Work items (`DOUBLE_PATH`, `ROD_DETOUR`, `SHAPE_DENSITY_METRIC`)
begin until Phase 2 corpus metrics are captured and committed as
`docs/features/roundtrip-closure-aware-phase2.json`.

## Future Work: Targeted Alternative Algorithms

Do not implement these until Phase 1 (and ideally Phase 2) corpus evidence
shows which failure modes remain. Each entry below is gated explicitly.

### DOUBLE_PATH

Primary use case: **corridor-terrain topology** — coast, valley, river,
sparse rural networks — where two roughly edge-disjoint paths between `s`
and a target `t` form a cleaner loop than any multi-waypoint greedy
construction. Loop length is a secondary effect of the same mechanism
(short loops are more likely to be in tight corridors), not the primary
trigger.

Algorithm sketch:

```text
build candidate target pool from ISO and graph-native nodes
for target t:
  route outbound s -> t
  route return t -> s with outbound path as refTrack/poison
  merge outbound + return
  evaluate quality gate
  score accepted loops
pick best accepted loop
```

Use when:

- ISO pool is narrow but not empty;
- greedy rejected closures for self-intersections;
- graph looks like coast, valley, river corridor, or sparse rural network.

Gating: implement only if Phase 1 + Phase 2 corpus evidence shows residual
chaotic-shape failures concentrated in corridor-terrain sentinel regions.
If closure-aware reranking plus iso-asymmetry bearing already resolves
those cases, do not implement.

### ROD_DETOUR

Best for sparse rural or valley terrain where an attractive outbound path exists
and the main challenge is finding a sufficiently different return.

Algorithm sketch:

```text
choose outbound rod by profile-valid path search
poison rod corridor
route detour return
score closed route
```

This is close to `DOUBLE_PATH` but treats the outbound path itself as the main
object, not only the target endpoint.

Gating: same as `DOUBLE_PATH`. Implement only if corpus evidence shows
residual failures specific to sparse-rural / valley topology that
closure-aware reranking does not resolve.

### SHAPE_DENSITY_METRIC

A continuous direction-change density signal added to `RouteChoiceScore`
(not to the hard gate). Mean absolute bearing change per kilometre, or
count of turns greater than 60 degrees per kilometre, weighted as a soft
penalty in the AUTO-competition winner selection.

Rationale: the existing gate uses hairpin count (greater than 120 degrees,
capped at 20) and self-intersection count (capped at 5). Both are pass/fail
binary signals. Moderate zigzag density — for example, 30 turns of 60-90
degrees over 50 km — does not trigger either gate but produces cyclist
complaints about route plausibility.

Gating: implement only if Phase 1 corpus evidence shows that closure-aware
reranking accepts loops which then receive low subjective similarity scores
in the GPX corpus evaluation for shape-related reasons. The hypothesis is
that the rerank pressure plus existing shape signals will catch most
chaos, making this metric unnecessary. Verify before building.

### ISOCHRONE Polygon

Keep as fallback/debug and small-loop option. It is useful when:

- route length is short;
- reachable region is compact;
- angular coverage is good;
- graph density is high enough that snapped vertices route cleanly.

Do not use pure geometric polygon placement as the production default for
coastal, island, mountain, or sparse networks.

## Cross-Phase Invariants

These rules hold throughout all phases and are regression-tested at every
PR:

- `RoundTripQualityGate` thresholds remain unchanged unless a test proves
  a threshold is objectively wrong with corpus evidence.
- No provider-selection changes in
  `RoutingEngine.buildCandidateProvider()`. `GREEDY` continues to use
  `GraphNativeCandidateProvider`; `ISO_GREEDY` continues to use
  `BlendedCandidateProvider`.
- No external libraries added.
- No relaxation of beeline, ferry, hostile-stretch, hairpin, or
  self-intersection gates.
- Explicit-via semantics preserved: user-supplied waypoint order is
  never reordered by any phase of this work.
- `[[feedback_challenge_suggestions]]` discipline: any phase that
  increases acceptance count while reducing `averageSimilarityScore` by
  more than 0.02 requires manual track inspection and an explicit
  decision before merge.
- All telemetry fields added by any phase remain populated (with sentinel
  `NaN` / `-1` when inactive) in subsequent phases. No field is removed
  without a deprecation comment.
- Determinism: byte-identical accepted tracks on repeated runs with
  identical inputs.

## Overall Done Criteria

The full closure-aware control-node planning work (Phases 0, 1, 2) is
complete when:

- All three per-phase Acceptance Criteria blocks above are satisfied.
- All three corpus artifacts are committed:
  `roundtrip-closure-aware-baseline.json`,
  `roundtrip-closure-aware-phase1.json`,
  `roundtrip-closure-aware-phase2.json`.
- The Future Work section's gating notes are accurate (corpus evidence
  recorded for or against each deferred item).
- No Cross-Phase Invariant is violated by any merged change.

