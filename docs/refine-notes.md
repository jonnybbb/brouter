# Refinement Stage Working Notes

## M0 Integration Verification Findings (2026-09-25)

Before building the refinement stage components, the four integration facts noted in §2.1, §4.5, and M0 were verified against the codebase at `eec40289`:

1. **WAYPOINT writes output before the gate (`RE:393`)**
   - **Verified:** In `RoutingEngine.doRouting` (lines 393–438), when `outfileBase != null`, the output file is formatted and written to disk synchronously inside `doRouting`.
   - `FastStrategy.attempt` invokes `orchestrator.doRoutingIntoRequest`, which calls `RoutingEngine.doRouting`.
   - `RoundTripOrchestrator.doRoundTrip` runs the quality gate (line 480) *after* `attempt()` returns.
   - Therefore, WAYPOINT produces and writes output to disk before `doRoundTrip` ever reaches the gate or any post-gate hook.
   - In contrast, Greedy and Auto either defer output writing (`request.deferredOutputWrite = true`) or suppress it in child engines (`roundTripSuppressDecoration = true`), and the parent writes output at line 614.
   - Per spec §3 (D5) and §4.6, WAYPOINT and ISOCHRONE baselines are never eligible for refinement (`tier_not_supported`).

2. **How `request.lastResult` and the local `quality` are used after the hook**
   - **`request.lastResult`:** Set by Greedy planning (`setPlannerResult` at `RoundTripOrchestrator.java:203`). After the hook, it is published to the engine in the `finally` block: `ops.setLastRoundTripResult(request.lastResult)`.
     - When refinement accepts a replacement, `lastResult` must be updated atomically or have refinement telemetry attached to it, while engine getters expose the result.
     - With refine off, `request.lastResult` is untouched.
   - **Local `quality` (`RoundTripQualityResult`):** In `RoundTripOrchestrator.doRoundTrip`, `quality` is assigned from the gate (line 480) and stored in `request.qualityVerdict = quality` (line 485).
     - In lines 491–525, `quality` is inspected for acceptance, hard-rejections, and advisory/disclosures.
     - Placing the hook between gate evaluation and decoration means:
       - The hook receives the baseline gate verdict. If `!quality.isAccepted()`, refinement is skipped (`gate_rejected`).
       - If a replacement candidate passes the ship predicate, the replacement candidate brings its own `verdict` from finalization.
       - The orchestrator updates local `quality = replacement.verdict`, `request.qualityVerdict = replacement.verdict`, and `request.track = replacement.track`.
       - Downstream decoration then decorates the replacement track using the updated `quality` verdict.

3. **`finalizeAdoptedRoundTripTrack` writes matched waypoints into engine state (`RT/RoundTripTrackCleanup.java:102`)**
   - **Verified:** In `RoundTripTrackCleanup.finalizeAdoptedRoundTripTrack(track, mwps)` (line 102), `state.setMatchedWaypoints(mwps)` is called directly.
   - `state` is the `RoundTripRequestState` implemented by `RoutingEngine`, so this call mutates the engine's `matchedWaypoints` field!
   - **Consequence for `RefineFinalizer`:** The finalizer evaluates candidates before the decision to ship them is made. It MUST NOT mutate the engine's active `matchedWaypoints` or other engine state during speculative finalizations.
   - `RefineFinalizer` must isolate or decouple the cleanup step from engine state so that evaluating candidate A does not corrupt baseline or candidate B. Only upon atomic `publish()` should the engine state be updated with the winning candidate's matched waypoints.

4. **The outer catch clears the route (`RT/RoundTripOrchestrator.java:620`)**
   - **Verified:** In `RoundTripOrchestrator.doRoundTrip` (lines 620–640), any unhandled `Exception` triggers:
     - `setError(ops.errorMessage())`
     - preservation of geometry on `lastRejectedTrack`
     - `setTrack(null)`
   - Therefore, any unhandled exception thrown in the refinement stage would wipe out the baseline route and fail the entire request.
   - **Consequence:** `RefineStage` must catch all non-fatal exceptions internally, log diagnostics, record the failure reason, and gracefully allow the baseline track to ship completely unaffected.

---

## M0 Deliverables & Measurement Summary

### M0.1: Linear Path Walker & `LoopCostOracle`
- **Linear Path Walker:** Implemented package-private `walkPathCost` in `RoutingEngine` and `LoopCostOracle`. Follows link transitions using `routingContext.createPath` with continuous elevation hysteresis and turn-cost context across vias. Handles endpoint edge clipping and transfer-node reduction.
- **Acceptance Criteria Met:**
  - Validated against spike method on $\ge 40$ fixture legs: exact 0.00% reproduction of routed cost.
  - Segmentation invariance: splitting the same continuous ride into 2 vs 5 legs yields identical cost (within float precision).
  - Closed loops, turn penalties, and elevation hysteresis correctly handled.
  - Detailed report available in `docs/m0_1_oracle_report.md`.

### M0.2: Geometry Fidelity
- Implemented `GeometryFidelityTest` validating edge cases: mid-edge vias, same-edge endpoints, opposite arrival/departure, curved edges, and removed-via spurs.
- Skeleton recovery after `RoundTripTrackCleanup` confirmed on GREEDY and AUTO loops.
- Detailed report available in `docs/m0_2_geometry_fidelity_report.md`.

### M0.3: `RefineFinalizer`, `publish()`, and Deadline Contract
- Implemented `RefineFinalizer`, `FinishedCandidate`, `FinalizationOutcome`, and `ShipPredicate`.
- Failure-injection tests confirm that aborts during detail retrack, cleanup, gate evaluation, or pricing leave baseline state 100% byte-identical while publishing diagnostics.
- Ownership test verifies speculative candidates do not leak or mutate engine `matchedWaypoints`.
- Timeouts in retrack or pricing return `FinalizationOutcome.TIMEOUT` (not fallback raw tracks) and flag `refineTruncated`.

### M0.4: Initialization
- Implemented `LegEvaluator`, `DefaultLegEvaluator`, `LegCache`, and `RefineInitializer`.
- Re-routes the $n$ raw legs without `refTrack` anti-reuse penalty.
- Seam validation uses directed edge plus crosspoint identity.
- Detailed report in `docs/m0_4_initialization_report.md`.

### M0.5: One Replacement End-to-End
- Implemented `SplitmixRandom` (Android API 23 compliant pseudo-random number generator using Box-Muller normal transform).
- Implemented `MoveProposalOperator` enforcing:
  - Gaussian displacement $\le 0.25 \times R$ from original via position.
  - Adjacent via spacing $\ge 300\text{ m}$.
  - Radius bound $\le 0.60 \times L$.
  - Detection and rejection of no-op snaps.
- `RefineSingleMoveTest` verified end-to-end execution:
  - Accept path: candidate passing all ship predicate criteria is published atomically.
  - Reject path: inferior candidate rejected, leaving baseline route, waypoints, and verdict bit-identical.

### M0.6: Full-Stage Cost Model Benchmark
- Measured on 32 stratified gravel cells (`FullStageCostModelBenchmark`, `-Dloop.forks=1`):
  - Terrains: Open (Crete Senesi), Town (Dreieich), Coastal (Nice), Hilly (Girona)
  - Distances: 30 km and 100 km; Algorithms: GREEDY and AUTO; Headings: 2 per region.
- Component Latency Breakdown:
  - Baseline pricing: mean 3.3 ms, p90 6 ms
  - Initialization ($n$ legs): mean 26.7 ms, p50 24 ms, p90 59 ms, max 75 ms
  - Evaluation (per proposal): mean 9.8 ms, p50 7 ms, p90 20 ms, max 45 ms
  - Finalization (detail + cleanup + gate + oracle): mean 28.5 ms, p50 27 ms, p90 59 ms, max 81 ms
  - Total Refinement ($k=4$ evaluations): mean 97.7 ms, p50 85 ms, p90 174 ms, max 308 ms
- Farthest point / radius ratio analysis:
  - Theoretical circle: $0.318 L$
  - Observed p50: $0.253 L$, p95: $0.303 L$, max: $0.311 L$
  - Derived empirical radius bound: **`0.60 L`** (comfortably bounds all valid loops without allowing runaway dilation).
- **Decision D5 Assessment:** Added latency p90 is **174 ms**, well within the $\le 3,000\text{ ms}$ budget bar.

### M0.7: Architectural Decisions & Governance
- Authored `docs/adr/0003-refinement-stage-placement.md` (Option A: In-core engine stage).
- Authored `docs/adr/0004-continuous-path-energy-and-ship-predicate.md` (Continuous path walker cost oracle, composite search energy, and ship predicate).
