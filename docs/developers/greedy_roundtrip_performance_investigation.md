---
title: "Greedy round-trip planner: performance investigation"
parent: Developers
---

# Greedy round-trip planner — performance investigation

Scope: `brouter-core/src/main/java/btools/router/GreedyRoundTripPlanner.java` and every
collaborator on its hot path. Symptom under investigation: **loop generation takes up to
minutes and causes timeouts**. All file:line references are to the state of branch
`claude/greedy-round-trip-perf-epbs3d` at the time of writing.

Method: full read of the planner and its collaborators, plus a 19-agent audit
(4 subsystem maps → 5 independent analysis lenses producing 47 raw findings →
adversarial verification of the top 10 by impact). **All 10 verified findings were
CONFIRMED against the code (0 refuted)**; the corrections the verifiers produced are
incorporated below.

## TL;DR — where the minutes go

One AUTO round-trip request is not "one plan with a 30s deadline". It is a
multiplication tree of sequential retries, each with a **fresh** hardcoded deadline,
running Dijkstras that are themselves unnecessarily slow:

```
AUTO competition (sequential, deadline only gates SPAWNING a child)
└─ ISO_GREEDY child                                 ┐
   └─ runGreedyAttempt: up to 4 subRouteCounts      │ each plan() gets a
      └─ plan(): fresh 30s deadline (+10s force-close)│ fresh 30s deadline
   └─ Phase 2.1 axis retry: the whole ladder again  │
   └─ on failure: recursion into GREEDY = ladder ×2 ┘
└─ GREEDY child (same ladder)
└─ WAYPOINT child, ISOCHRONE child (fallbacks)
└─ finalize/adoption passes
```

Inside each `plan()`, per attempt (up to 8 per step, up to 6 steps):

| Cost | What | Where |
|---|---|---|
| 1 bounded Dijkstra (up to 1.5M nodes) | candidate expansion, re-run EVERY attempt for steps ≥ 2 (cache bypassed when refTrack present); no wall-clock or kill check inside its loop | `GraphNativeCandidateProvider.java:77-99`, `RoutingEngine.java:3846-3961` |
| up to 5 point-to-point Dijkstras (10s cap each) | routing top-K candidates — run **undirected** (`airDistanceCostFactor = 0`, single pass, no cost-cutting seed) | `GreedyRoundTripPlanner.java:1473-1505`, `RoutingEngine.java:184,6144-6241` |
| 17-point road match per routed candidate | profile-aware via snap probe rings | `RoutingEngine.java:1910-1926` |
| 1-2 guided Dijkstras | detail retrack of accepted leg (+ fallback reroute), flips NodesCache detail mode and discards decoded tiles both ways | `RoutingEngine.java:6275-6317`, `NodesCache.java:85-108` |
| 1-3 Dijkstras | return leg + up to 2 relaxed variants when it self-crosses + detail retrack | `GreedyRoundTripPlanner.java:1366-1426` |
| 2-3 × O(N²) geometry scans | tentative self-intersections per candidate, gate verdict (twice on the chaos path), metrics | `RoundTripQualityGate.java:570-621,446-451` |

## 1. Findings — architecture & budget (highest impact)

### F1. `plan()` ignores the request budget; retry ladders multiply a hardcoded 30s deadline — **the primary cause of "minutes"**
- `DEFAULT_PLAN_DEADLINE_MS = 30_000` is applied per planner instance (`GreedyRoundTripPlanner.java:192,497`); the engine's `maxRunningTime` (request budget) is never consulted.
- `runGreedyAttempt` (`RoutingEngine.java:4530-4561`) creates up to 4 fresh planners (`greedySubRouteCountPlan`, 2500-2512) → up to ~160s sequential.
- Phase 2.1 axis retry re-runs the ladder (`RoutingEngine.java:2312`); ISO_GREEDY failure recurses into a GREEDY ladder (`RoutingEngine.java:2438`).
- The AUTO competition budget (`RoutingEngine.java:1132`) only stops **spawning** children; a running child cannot be stopped and internally ignores its `doRun(budget)` slice, because `timedFindTrack` overwrites `engine.startTime/maxRunningTime` per call (`GreedyRoundTripPlanner.java:1481-1486`) — the engine's own timeout check is defeated during every leg search.
- The final `doRouting(roundTripRoutingBudgetMs)` fallback reuses the FULL original budget, not remaining time (`RoutingEngine.java:2424,403`).
- Verifier refinement: Phase 2.1 is gated on `algo == ISO_GREEDY` (:2297), so it cannot fire again inside the GREEDY recursion; the worst case for one AUTO ISO_GREEDY child is ~(2 ladders + 1 recursion ladder) ≈ 8 plans ≈ **480s** — still ~8× a 60s AUTO budget.
- `retrackForDetail` is bounded only relative to the wrong clock: it resets `startTime` and runs under the full request `maxRunningTime` (or a 60s fallback when none), so detail retracks legally run past the plan deadline (`RoutingEngine.java:6273-6316`).

**Fix (S/M, low risk):** thread one absolute `requestDeadline` from `doRun` through `doGreedyRoundTrip` → `runGreedyAttempt` → `plan(deadline)` → `timedFindTrack` (`min(SUB_ROUTE_TIMEOUT, planDeadline, requestDeadline)`). Check it between ladder rungs, before Phase 2.1, before recursion, and inside the expansion loop (F2). This makes worst-case wall clock equal the configured request timeout instead of a multiple of it.

### F2. `runIsochroneExpansion` has no time/termination check — un-killable multi-second expansions
The expansion loop (`RoutingEngine.java:3945+`) checks only cost budget (4× radius), geo cutoff (1.5× radius) and node cap — up to **1.5M popped nodes** for ~100km-loop radii (`RoutingEngine.java:3896-3898`). No `System.currentTimeMillis()` check, no `terminated` check (the watchdog kill flag is only honored in `_findTrack`). It runs once per (step ≥ 2, attempt) — up to ~40× per plan.

**Fix (S, low risk):** deadline + `terminated` check every ~4k pops; return the partial frontier on expiry (callers already handle sparse candidate sets).

### F3. Candidate legs are routed with an undirected single-pass Dijkstra
The greedy path never sets `engine.airDistanceCostFactor` (field default 0.0, `RoutingEngine.java:184`), and `timedFindTrack` → `findTrack(name, from, to, /*costCuttingTrack*/ null, refTrack, false)` runs one pass with neither goal-direction weighting nor a cost-cutting seed (`GreedyRoundTripPlanner.java:1487`). Normal BRouter legs use `searchRoutedTrack`'s 2-pass scheme (directed pass at `pass1coefficient` ≈ 1.5 — roughly linear in distance — then a cutoff-bounded exact pass; `RoutingEngine.java:6144-6241`). An undirected search explores a cost-disk (~quadratic in leg distance) before the first destination match establishes a cutoff. This is a large per-Dijkstra constant paid ~10× per attempt, and the reason individual legs hit the 10s cap in dense networks.

**Fix (S, medium risk to route choice):** route *candidate* legs with `airDistanceCostFactor = routingContext.pass1coefficient` (heuristic is acceptable — candidates are re-scored and the accepted leg is detail-retracked anyway); optionally keep the exact 2-pass for the *return/closure* leg. Behavior change is bounded: leg geometry may differ slightly (heuristic pass), which the existing scorer + gate already tolerate; validate on the scenario corpus.

### F4. Closure rejection throws away already-routed alternatives (the unimplemented "Step 3")
`routedCandidates` is built as a ranked list precisely so a rejected closure could try the runner-up (comment at `GreedyRoundTripPlanner.java:706-711`), but the rejection paths (closure gate rejection :1046-1063, too-long :1081-1097) undo the leg and restart the attempt loop: re-expansion + re-scoring + re-routing K candidates + re-detailing, at a smaller radius. Each such cycle costs several Dijkstras; closure rejections regularly happen 2+ times per plan (there is a distress brake at 2).

**Fix (M, medium risk):** on rejection, first iterate the remaining `routedCandidates` (already routed; only detail + gate needed) before shrinking the radius and regenerating. This converts the most expensive retry cycle into a list walk.

### F5. Per-attempt expansion re-runs even though the refTrack is constant within a step
`cachedRefTrack` is built once per step (`GreedyRoundTripPlanner.java:569`) and identical across all attempts of that step, but `GraphNativeCandidateProvider` bypasses its expansion cache whenever refTrack != null (`GraphNativeCandidateProvider.java:90-94`), so **every attempt** re-runs the poisoned expansion. Radius backoff changes the expansion radius on failed attempts, but the first attempt of each step always pays a full expansion that could often be derived from the previous attempt's (larger-radius) expansion: candidates within the new window are a subset of already-expanded nodes.

**Fix (M):** cache per (position, refTrack identity) the largest completed expansion of the step and derive smaller-radius candidate windows from it (the window/sort is already applied per call, :96-100). Alternatively: drop poisoning from candidate *generation* entirely and let the scorer's reuse terms (which already exist: `visitedEdges`, `actualVisitedRatio`) do that job — one expansion per step position.

### F6. Exact costs to candidates are computed and thrown away; return distances are guessed
The expansion knows the exact profile cost (and compiled path) to every retained candidate (`bucketContourCost`/`bucketBestCost`, `RoutingEngine.java:3909,3925`); Phase-1 scoring then re-estimates leg distance as `airDist × indirectnessEst` (`GreedyRoundTripPlanner.java:622-625`). Return-to-start distances are always `airDist × indirectnessEst` — the source of "too long → undo" cycles and mistaken return-skip decisions (`RETURN_SKIP_SAFETY` fudge, :256).

**Fix (M/L, biggest algorithmic win after F1-F3):**
1. Feed the expansion's real leg cost/distance into Phase-1 scoring (data already in hand).
2. Build a **return-distance oracle** once per plan: one reverse-ish expansion from the start (the ISO_GREEDY start-pool expansion already exists; GREEDY can run one) with radius ~desiredDistance/2, storing distance-from-start per 150m cell (the `visitedCells` structure already exists — it just doesn't store distances). Projected totals then use graph-true return distances (approximate under one-ways, exact enough for closure planning), which (a) kills most too-long undo cycles, (b) makes the return-skip decision safe without the 1.5× fudge, (c) improves via placement so fewer attempts are needed at all.

### F7. AUTO children (and top-K candidate routing) are sequential but independent
The server runs one engine per request thread (`RouteServer.java`), and `runChildCandidate` already builds fully isolated child engines + contexts (`RoutingEngine.java:1306-1341`). ISO_GREEDY and GREEDY children could run concurrently (2 threads) with the same selection policy applied after both finish; ~2× on the competition's dominant path with zero route-output change (selection is already score-based over the full result set; keep WAYPOINT/ISOCHRONE as a sequential fallback wave). Within a plan, the K candidate routings are read-only with respect to each other and could run on 2-3 child engines; this requires making `plan()`'s engine interactions explicit (matchPoint/timedFindTrack mutate engine state today) — more invasive, so treat as a second stage. Memory cost: one NodesCache per extra engine (`memoryclass` MB each).

### F4b. Work ordering inside an attempt: expensive detailing happens before cheap rejection
Two related control-flow inversions:
- `detailAcceptedTrack` (1-2 guided Dijkstras + fidelity checks + paved-hostility scan) runs at `GreedyRoundTripPlanner.java:902` **before** Phase 4 computes the closure distance — but the too-long rejection (:1081) needs only the raw leg distance + return distance. Every too-long undo discards 1-3 detail searches that were never needed. Reorder: run the return check on the raw leg first; detail only legs that survive the closure decision.
- `needDetail` (:1002-1004) contains the clause `(bestFallback != null && !bestFallback.gateAccepted)`, which keeps it true on essentially every return check until a gate-accepted fallback exists. Each such check pays `detailWithFallback` on the return leg (retrack + possible reroute + second retrack), `mergeSegmentsDetoured` (full per-leg detour-map copy), and a full gate `evaluate` (1-2 × O(N²)). Consider a cheaper pre-verdict (distance + reuse only) before paying for detail + full gate.
- `retrackForDetail` and force-close also run **outside** the plan deadline: retrack sets its own `startTime` (+60s fallback budget when the caller has none, `RoutingEngine.java:6273-6316`), and force-close grants itself +10s past the deadline (:1141). A 30s plan can legally run substantially longer.

## 2. Findings — CPU hotspots (non-Dijkstra)

### F8. The O(N²) self-intersection scan family runs hundreds of times per plan
- `countSelfIntersections` is an all-pairs CCW test with no spatial index (`RoundTripQualityGate.java:570-621`); clean loops always pay the full N²/2 (≈1.4M pair tests at 50km, ~5.4M at 100km, ~29M at 200km; stride decimation caps it at 10k sampled nodes ≈ 5·10⁷ pair tests, so bounded-quadratic). Early exit only helps *dirty* loops (ceiling 20 crossings).
- Called per routed candidate on prefix+leg (`countTentativeSelfIntersections`, planner :847 — 120-240 calls/plan worst case), per return check + per return variant (:1376,1405 — up to ~144 more), inside every gate verdict (`checkShapeChaos`), **twice** on chaos rejection (the tier check re-runs the full scan, `RoundTripQualityGate.java:446-451`), and a third independent copy lives in `LoopQualityMetrics.detectCrossings` (`LoopQualityMetrics.java:188-262`, run in `populateResult`).
- The prefix×prefix pair block is recomputed for every candidate although it cannot change within an attempt.

**Fixes (S→M, no behavior change achievable):**
1. Cache the chaos scan's crossing count in `evaluate` so the tier decision reuses it (S, exact).
2. Make `countTentativeSelfIntersections` incremental: count prefix-internal crossings once per attempt; per candidate count only candidate-internal + candidate×prefix pairs, reducing per-candidate work from O((P+L)²) to O(L² + L·P). Exactness caveat: the scan's start/end exemption windows (`CROSSING_START_END_EXEMPT_M`, `RoundTripQualityGate.java:587-596`) depend on total perimeter — but for any candidate longer than the window the end window lies wholly inside the candidate leg, so the prefix-internal count (computed with the end-exemption off) is shared exactly across all candidates of the attempt; fall back to the full scan for shorter legs. The two auxiliary passes (`countTransverseNodeRevisits` node-hash, `countCorridorCrossings`) are incrementality-friendly (persistent node-id index) but need the same treatment.
3. Longer term: a 40m spatial hash over prefix segments (the code already has one in `CorridorOverlapIndex`) turns candidate×prefix into near-linear.
4. Reuse caveat (verifier-confirmed): the gate evaluates the **detailed** (retracked) merged loop, whose node geometry and bridge/tunnel metadata differ from the pre-detail return track scored in `routeReturnWithVariants` — so the return-variant crossing count cannot be reused for the gate verdict without a behavior change. Cache within, not across, geometry versions.

### F8b. Bonus correctness/perf quirk: bulge-repair connector can time out instantly
In the greedy bypass path nothing sets the engine's `startTime` field (`doRoundTrip`'s local `startTime` at `RoutingEngine.java:735` shadows it). `repairViaPinnedBulges`' connector `findTrack("bulge-repair", ...)` (:3326) then runs its timeout check against a stale/zero `startTime`, and with `maxRunningTime > 0` can throw immediately — silently caught (:3327-3331) as "connector routing failed". Repairs silently don't happen, and the failure is invisible. Set `startTime = now` around the connector search.

### F9. Detail-mode cache flips discard the decoded graph
Every accepted-leg/return retrack (`detailed=true`) and every subsequent routing call (`detailed=false`) rebuilds NodesCache decoded rows because reuse requires matching detail mode (`NodesCache.java:91`). The planner alternates constantly; each flip forces re-decoding of all touched tiles. Options: keep two NodesCache instances (one per mode) alive in the engine and swap (M; memory = 2× microcaches), or batch retracks (accept legs with raw geometry and detail-retrack once per closure candidate — behavior-relevant, needs the gate's metadata requirements rethought).

### F10. Per-candidate 17-point probe matching
`profileAwareMatchPoint` batch-matches up to 17 probe points per routed candidate (`RoutingEngine.java:1910-1926`) — up to ~240 candidate snaps × 17 = ~4000 point matches/plan, each touching microcaches. The probe rings are only *needed* when the plain snap is profile-hostile (cf ≥ 2.0), which the code already checks (:1934) — but only after matching all 17 points. Reorder: match the plain point first, probe rings only when hostile (S, exact for non-hostile snaps — the common case).

### F11. Flat 10s per-Dijkstra cap
`SUB_ROUTE_TIMEOUT_MS = 10s` regardless of leg length or remaining plan budget: two pathological searches consume 2/3 of a 30s plan. With F3 (directed search) most legs finish in well under a second; the cap should scale (e.g. `max(1s, k × airDist)`) and never exceed remaining plan/request budget (S).

## 3. Findings — JVM / memory

- **Merged-track churn:** `buildRefTrack` per step + per return check, `mergeSegmentsNoMap` per attempt, full merge + `buildMap` per `needDetail` closure — each O(P) allocations of `OsmTrack` + `ArrayList` + hash maps; tens of MB over a plan but dominated by the O(N²) scans above. Maintain one growing prefix list + undo by truncation instead of re-merging (M).
- **`expansionCache` never evicts** and each entry holds `visitedCells` + up to 144 compiled `OsmTrack`s (`GraphNativeCandidateProvider.java:59,86-89`); shared across the whole ladder → memory creep in long AUTO requests (S: cap/evict).
- **`finalTrackReuseRatio`/gate maps** box Long keys (`HashMap<Long,Integer>`); called once per plan — fine. The planner's own `VisitedEdgeStore` is already primitive open-addressing (good).
- **Diagnostics:** `String.format` + string concat per attempt/step are noise compared to the above; not worth touching except inside the per-candidate loops (there are none today).
- The isochrone expansion allocates 36×3 contour arrays + `OsmPath` objects for up to 1.5M pops per attempt — the node cap, not allocation rate, is the right lever (see F2/F5).
- **Total GC pressure:** a 100km plan allocates on the order of 10-30 GB, >90% of it `OsmPath`/`OsmPrePath` objects inside the searches and expansions. Conclusion: reducing the *number and directedness* of searches (F1-F6) dominates any micro-optimization of the remaining allocation sites.
- **refTrack-poisoning growth hypothesis refuted:** the per-edge anti-reuse check is `containsTraveledSegment`, an O(1) HashSet hit after a lazy one-time build (`OsmTrack.java:369-396`); later Dijkstras do NOT get slower from a growing refTrack. The real growth cost is the repeated O(P) refTrack re-merge + map build per step/closure (see above).

### Concurrency implementation notes (for F7)
- Single-engine parallelism is blocked by ~5 mutable single-slot engine resources overwritten per `findTrack` (startTime/maxRunningTime, airDistanceCostFactor, guideTrack, nodesCache, matchPath) — parallelism means worker *engines*, not threads on one engine.
- `ProfileCache` is a static synchronized LRU sized `2×maxthreads` by the server; a worker-engine pool must fit it or profile re-parses will thrash.
- Determinism: collect parallel candidate results indexed by their `candidateIndex` r-order, then apply the existing stable sort — byte-identical ranking to the sequential loop.
- Cancellation: the volatile `terminated` flag checked per Dijkstra pop is exactly the right hook — a coordinator can fan out `terminate()` on deadline or winner-commit, which *also* fixes today's uninterruptible AUTO children (once F2 adds the flag check to the expansion loop).

## 3b. Conceptual alternative worth prototyping: skeleton-first placement
The per-(step,attempt) cost drivers (expansion, K matches+routings, tentative scans, return checks, undo cycles) all exist because vias are chosen one at a time with commit-and-undo. The codebase already computes a start-centered isochrone with 36 direction buckets × 3 cost contours (ISO_GREEDY pool). An alternative: choose ALL vias upfront from that one expansion at ~subTarget spacing along the requested rotation (with the same convexity/heading terms as a global filter), route the S legs once (in parallel per F7), then repair only the failing legs locally. This collapses the multiplicative structure to ~S+1 routed legs + a small repair budget, and the existing quality gate remains the arbiter. Route-quality risk is real (less adaptive to terrain discovered en route — the indirectness EMA information arrives too late), so it should compete against the tuned greedy on the scenario corpus rather than replace it outright; but it is the shape of a planner whose worst case is seconds, not minutes.

## 4. Implementation status (this branch)

Implemented in three commits on `claude/greedy-round-trip-perf-epbs3d`
("thread the request budget", "cut redundant search work", "parallel AUTO
greedy children"):

- **F1/F2/F11 + F8b** — request-deadline threading through doRun → ladder →
  plan() → timedFindTrack → expansion loop (which also honours the watchdog
  kill flag now); distance-scaled per-Dijkstra caps; bounded force-close
  grace; remaining-budget fallback doRouting; startTime anchoring (bulge
  repairs actually run on servers now).
- **F3** — planner legs run goal-directed at the profile's `pass1coefficient`.
- **F8.1/F8.2** — the gate's chaos-tier double scan removed; the segment-pair
  crossing scan uses a spatial-hash grid above 512 segments with proven exact
  parity (`SelfIntersectionGridEquivalenceTest`).
- **F4/F4b** — closure-aware trial loop over the ranked routed candidates;
  raw-leg-first ordering so too-long undos pay zero detail Dijkstras.
- **F5** — per-step reuse of the largest with-refTrack expansion across
  backoff attempts; the step-1 expansion cache is bounded.
- **F6-lite** — Phase-1 scoring uses expansion-compiled real leg distances
  where available.
- **F7** — ISO_GREEDY ∥ GREEDY AUTO children with sequential-policy parity
  and terminate-on-unentitled.

Not implemented (needs scenario-corpus validation before shipping):
the full return-distance oracle (F6, distance-from-start grid) and the
skeleton-first placement prototype (§3b). Golden signatures
(`LoopGoldenSignatureTest`) must be re-captured on tiles: F3, F4/F4b, F5,
F6-lite and the budget gates deliberately change which candidate wins in
scenarios that previously hit caps or exhausted attempts.

## 5. Suggested roadmap (original)

| Order | Change | Size | Wall-clock effect | Route-output risk |
|---|---|---|---|---|
| 1 | F1 request-deadline threading (incl. F2 expansion checks, F11 cap scaling) | S/M | Worst case drops from minutes to the configured timeout; no more watchdog kills | None (only stops earlier) |
| 2 | F3 directed candidate-leg search (`pass1coefficient`) | S | Biggest per-Dijkstra win; legs stop hitting 10s caps | Low-medium; validate on corpus |
| 3 | F8 incremental/cached crossing scans | S/M | Removes tens of seconds of CPU on retry-heavy plans | None (exact) |
| 4 | F4 use routed runner-ups on closure rejection; F4b detail-after-closure-decision + cheaper pre-verdict | M | Removes the most expensive retry cycles and 1-3 wasted detail Dijkstras per undo | Low (same candidate set, better order) |
| 5 | F6 exact leg costs + return-distance oracle | M/L | Fewer attempts/undo cycles at the source | Medium (placement changes; usually improves) |
| 6 | F5 expansion reuse within a step / drop generation-side poisoning | M | Up to ~8× fewer expansions in retry-heavy steps | Medium |
| 7 | F7 parallel AUTO children | M | ~2× on AUTO's dominant path | None if selection stays score-based |
| 8 | F9/F10 cache-flip + probe-order | S/M | Constant-factor wins | None/low |

The first three items alone should turn the worst case "minutes, then killed by the
watchdog" into "bounded by the request timeout, with each 30s plan doing several times
more useful search per second".
