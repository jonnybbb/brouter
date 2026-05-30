# Result-Preserving Performance Optimization Report: Greedy & ISO_GREEDY Loop Generators

> **Implementation status (2026-05-30):** All 6 SAFE wins implemented and the
> harness is in place. Every change passes the new byte-identical route gate
> `LoopGoldenSignatureTest` (8 GREEDY/ISO_GREEDY scenarios over Dreieich /
> Berlin / Lozère, fastbike + gravel; opt-in `-Dgolden.tests=true`, baseline in
> `src/test/resources/test-data/golden/loop-signatures.txt`) plus fast parity
> unit tests in `GreedyLoopPerfInvariantTest`. `./gradlew :brouter-core:build`
> (tests + PMD + checkstyle) is green. Order followed §6: harness → SAFE-2 →
> SAFE-1 → SAFE-4 → SAFE-6 → SAFE-5 → SAFE-3. The two visited-edge HashMaps were
> folded into one primitive `VisitedEdgeStore` (occupancy-flagged
> open-addressing, backward-shift deletion). Multithreading was **not**
> attempted — see §4/§6 prerequisites. No commit made yet.

## 1. Executive Summary

This report covers 27 proposed performance optimizations for the brouter `greedy` and `iso_greedy` round-trip loop generators, each subjected to dual adversarial verification ("two skeptic lenses") against a single hard constraint: **bit-identical route output for all inputs**. The chosen route, every per-step candidate, and all telemetry must be byte-for-byte unchanged.

The verification outcome is stark and instructive:

- **6 ideas verified SAFE** — both lenses agreed they preserve bit-identical output (with implementation-discipline caveats only).
- **0 ideas landed as needs-guard** — none survived with a "no refutation but not fully convinced" status.
- **21 ideas REJECTED** — at least one lens found a concrete, reachable input that changes the route.

The dominant lesson: **every multithreading idea was rejected**, and they all failed for the same two reasons — (1) the per-sub-route **wall-clock timeout** (`SUB_ROUTE_TIMEOUT_MS`, the `System.currentTimeMillis() - startTime > timeout` checks in `_findTrack`) makes candidate survival scheduling-dependent under CPU contention, and (2) shared engine state that is not actually a pure function of `(from, to, refTrack)` — specifically `islandNodePairs.freezeCount` (island detection) and the shared `RandomAccessFile` file pointer in `PhysicalFile`. No amount of result-merging-by-index fixes a candidate that times out or throws `RoutingIslandException` differently in parallel. Parallelism is off the table for this engine without first making the timeout data-deterministic.

The wins are therefore concentrated in **single-threaded data-structure, caching, and micro-opt changes**. The top high-leverage SAFE wins are:

1. **Primitive long-keyed visited-edge maps** (replace `HashMap<Long,Integer>`/`HashMap<Long,Double>`) — kills autoboxing on the hottest pure-Java inner loop. Effort M, high confidence. *(Multiple proposals converged on this; treat as one change.)*
2. **Skip `buildMap()` for the self-intersection tentative merge** — removes a full `CompactLongMap`/`FrozenLongMap` build per routed candidate; the crossing scan never reads `nodesMap`. Effort S, high confidence.
3. **Cache the merged committed-prefix track per attempt for self-intersection counting** — reuse the prefix across all K candidates instead of re-merging K times. Effort M, high confidence.
4. **Hoist per-attempt comparator allocations** (static final comparators at the two sort sites + identity-membership back-fill in `pickDiverseTopK`). Effort S, high confidence.
5. **Precompute per-segment `calcDistance` once and reuse across the visited-ratio and hostility passes** — the feared double/int precision mismatch does not exist (both passes consume the same rounded int). Effort M, high confidence.

These five, taken together, remove the dominant non-Dijkstra allocator churn and a redundant geometry pass on the per-routed-candidate path without touching any Dijkstra, any float-summation order, any sort tie-break, or any concurrency semantics.

---

## 2. Ranked Recommendations Table

Ranked by (impact × safety / effort). All SAFE ideas rank above all rejected ideas. There are no needs-guard ideas.

| Rank | Optimization | Technique | Target (file:method) | Expected speedup | Effort | Status | Panel confidence |
|------|--------------|-----------|----------------------|------------------|--------|--------|------------------|
| 1 | Skip `buildMap()` for self-intersection tentative merge | algorithmic micro-opt | `GreedyRoundTripPlanner.java:954-964 mergeSegments` (from `countTentativeSelfIntersections`) | Removes 1 full `CompactLongMap` build per routed candidate (grows with route length) | S | safe | high |
| 2 | Hoist per-attempt comparator/closure allocations | allocation/GC | `GreedyRoundTripPlanner.java:289`, `:846-848`; `GraphNativeCandidateProvider.java:146-149` | ~40-80 small allocations/loop + O(k·n) scans removed | S | safe | high |
| 3 | Primitive long-keyed visited-edge maps (kill autoboxing) | data-structure | `GreedyRoundTripPlanner.java:190,195` + accessors `:987-1062` | ~thousands of boxed allocations/loop removed; 1.5-3× on the scan | M | safe | high |
| 4 | Cache merged committed-prefix track per attempt for self-intersection counting | caching/memoization | `GreedyRoundTripPlanner.java:407 countTentativeSelfIntersections → :941-952` | ~K-fold reduction of merge+buildMap per attempt; biggest on late steps | M | safe | high |
| 5 | Precompute per-segment `calcDistance` once, share across visited-ratio + hostility passes | data-structure | `GreedyRoundTripPlanner.java:1038-1062` + `RoundTripQualityGate.java:704-735` | Halves `CheapRuler.distance` sqrt+round calls per routed candidate | M | safe | high |
| 6 | Cache merged committed-prefix track per step (reuse `cachedRefTrack` for `refBeforeAccept` + drop tentative `buildMap`) | caching/memoization | `GreedyRoundTripPlanner.java:247,458,941-952` | Removes redundant `:458` merge + K `buildMap` calls/attempt | M | safe | medium |
| — | Parallelize route-top-K greedy-sub Dijkstras | multithreading | `GreedyRoundTripPlanner.java:315-430` | Large (≈Kx) | L | **rejected** | high |
| — | Parallelize per-candidate post-route scoring | multithreading | `GreedyRoundTripPlanner.java:359-410` | Moderate | M | **rejected** | medium |
| — | Overlap ISO_GREEDY pool build with first-step setup (AUTO mode) | multithreading | `IsochroneCandidateProvider.java:88-174` | Moderate | L | **rejected** | high |
| — | Per-plan() pool of cloned routing engines (parallelism substrate) | multithreading | `RoutingEngine.java:4931, 204-238` | Enabler | L | **rejected** | high |
| — | Primitive long open-addressing set for `seenCells` dedupe | data-structure | `GraphNativeCandidateProvider.java:126` | Modest allocation | S | **rejected** | high |
| — | Reuse a single thread-local primitive map across steps (clear, not realloc) | data-structure | `GreedyRoundTripPlanner.java:190,195`; `GraphNativeCandidateProvider.java:126` | Small (allocation) | S | **rejected** | medium |
| — | Primitive set for refTrack node membership (avoid `buildMap` rebuilds) | data-structure | `OsmTrack.buildMap():154-170` | Removes holder allocations on membership path | L | **rejected** | high |
| — | Incrementally maintain a single cached refTrack across `buildRefTrack` calls | caching/memoization | `GreedyRoundTripPlanner.java:247,…` | O(N²)→O(N) merge work | M | **rejected** | medium |
| — | Memoize `matchPoint` snapping per (ilon,ilat) within a plan() | caching/memoization | `GreedyRoundTripPlanner.java:318,915-932` | Removes redundant `matchWaypointsToNodes` scans | M | **rejected** | medium |
| — | Key GraphNative expansion cache on (pos, radius, refTrack-ref) | caching/memoization | `GraphNativeCandidateProvider.java:61-71` | Removes a Dijkstra on backoff retries | M | **rejected** | medium |
| — | Incrementally extend cached merged refTrack across steps (×3 variants) | caching / allocation / micro-opt | `GreedyRoundTripPlanner.java:247 → 936-964` | O(N²)→O(N) | M/L | **rejected** | high/medium |
| — | Thread-confined StdPath free-list pool (×2 proposals) | allocation/GC | `StdModel.java:20-22`, `RoutingEngine.java:5335,2744` | Large (highest-count allocation) | L | **rejected** | high |
| — | Dedup double `obtainNonHollowNode` in iso two-pass loop (×2 proposals) | micro-opt / IO | `RoutingEngine.java:2724,2737` | Halves materialization-path traffic | M | **rejected** | medium/high |
| — | Dedup double `obtainNonHollowNode` in `_findTrack` two-pass loop | IO/segment-loading | `RoutingEngine.java:5266-5344` | Halves materialization traffic | M | **rejected** | medium |
| — | Last-segment cache in `NodesCache.getSegmentFor` (incl. MicroCache sub-memo) | IO/segment-loading | `NodesCache.java:157-204` | Removes fileRow linear scan on warm hits | S | **rejected** | high |
| — | Scale-bucket strength reduction in `calcDistance` (CheapRuler) | micro-opt | `GreedyRoundTripPlanner.java:1045-1060` | Removes SCALE_CACHE divisions | M | **rejected** | high |
| — | "Precompute ROAD_INDIRECTNESS-scaled constants" bundled into comparator hoist | micro-opt | `GreedyRoundTripPlanner.java:266-289` | Tiny | S | **rejected** | high |

> Note on rank 5/6 overlap with rejected ideas: the SAFE versions are the **narrow** forms (parallelize/share only the proven-pure passes, with the route still produced sequentially). The rejected variants are the broader compositions (e.g. chaining onto parallel routing, or the *incremental cross-step* refTrack cache). Keep the two strictly separate when implementing.

---

## 3. Per-Idea Detail (SAFE ideas)

### SAFE-1 — Skip `buildMap()` for the self-intersection tentative merge
**Status: safe · confidence high · effort S**

**The change.** Add a private `mergeSegments(segments, finalSegment, boolean withMap)` (or a `mergeNodesOnly` variant) that builds the node list via `appendTrack` but does **not** call `merged.buildMap()`. Call it only from `countTentativeSelfIntersections` (`:950`). Keep the map-building path as the default for `buildRefTrack` (`:938`) and for the `finalTrack`/`snap.track` merges (`:567/:678/:1222`) where `containsNode`/`nodesMap` is genuinely used.

**Determinism argument.** `OsmTrack.buildMap()` (`:154-171`) only populates the separate `nodesMap` field; it never mutates `merged.nodes`, `distance`, `ascend`, `cost`, or any `OsmPathElement`. The sole consumer, `RoundTripQualityGate.countSelfIntersections` (`:491-555`), reads **only** `track.nodes` — `sampledShapeNodes` index sampling, the `i<j` scan with the `i==0 && j==n-2` closure skip, integer `ccw` long arithmetic, `samePoint`/`oppositeSigns`, and the `absoluteCeiling` early-exit. It never dereferences `nodesMap` or calls `containsNode`. The tentative track is fully local and never escapes. The crossing count — and therefore the `PARTIAL_SELF_INTERSECTION_WEIGHT * count` term in `routedScore`, the `sortByRoutedScore` ranking, and the chosen winner — is bit-identical. Both lenses found no diverging input.

**Guard test.** Run the LoopGoldStandard / roundtrip-closure-aware-baseline corpus before and after; assert byte-identical node sequence, per-step chosen candidate, and `selfX` telemetry. Add a unit test asserting `countSelfIntersections` returns the identical value for a multi-segment track built with and without `buildMap()`, and that `buildMap()` leaves `nodes`/`distance`/`ascend`/`cost` unchanged.

---

### SAFE-2 — Hoist per-attempt comparator/closure allocations
**Status: safe · confidence high · effort S**

**The change.** Three small allocations on the per-attempt path:
1. `GreedyRoundTripPlanner.java:289` — hoist `Comparator.comparingDouble(c -> c.score)` to a `static final Comparator<CandidatePoint> CMP_BY_SCORE`.
2. `GraphNativeCandidateProvider.java:146-149` — hoist the 3-stage chained comparator (`distanceError` asc, `-bucketHits`, `-sourceContour`) to a `static final Comparator<Template>` (anchor `T=Template` with a leading `(Template t)` cast).
3. `pickDiverseTopK` back-fill (`:846-848`) — replace the O(picked) linear `picked.contains(cp)` scan with an identity-membership scheme (boolean mark or skip-taken-indices) that yields the **same boolean** for the **same object**.

**Determinism argument.** `comparingDouble` captures no per-call state (pure `c.score` read), uses `Double.compare` (so −0.0 < +0.0, NaN-last identical), and `List.sort` remains stable TimSort, so equal-score ties still resolve by pre-sort provider/iteration order. The GraphNative chain compares identical fields in identical order with identical `Integer.compareTo` autoboxing; the dedupe and cap-to-36 run after the sort and are untouched. For the back-fill: `CandidatePoint` has no `equals`/`hashCode` override, so `contains` is identity-based today; both lenses confirmed the `sorted` list always holds **distinct references** (each provider allocates `new CandidatePoint()` exactly once and sort never duplicates), so any identity-preserving membership scheme reproduces the exact first-fit add sequence. The 30° angular threshold check is untouched.

**Guard test.** Diff full routed output (node sequence, chosen candidate per step, telemetry/diagnostics) on the LoopGoldStandard / LoopTestSegments corpus — assert byte-identical. Add a focused `pickDiverseTopK` unit test with two distinct `CandidatePoint`s of equal score and equal bearing (and a duplicate-by-equals-but-distinct-identity case) asserting the returned list is reference-equal element-for-element to the pre-change implementation.

---

### SAFE-3 — Primitive long-keyed visited-edge maps
**Status: safe · confidence high · effort M** *(three proposals converged on this; implement once)*

**The change.** Replace `Map<Long,Integer> visitedEdgeCounts` and `Map<Long,Double> visitedEdgeFirstPos` (`:190,195`) with purpose-built primitive `long→int` and `long→double` open-addressing maps supporting `get`/`containsKey`/`merge(+1)`/`put`/`remove`/`isEmpty`. `edgeKey` (`:1098-1104`) is unchanged. Rewrite `addVisitedEdges`/`removeVisitedEdges`/`computeTrackVisitedRatio` accessors to mirror the exact branch semantics: first-visit `prev==0` record, `count<=1` remove-both vs `count>1` decrement-keep-firstPos asymmetry, and the `firstPos != null` present check.

**Determinism argument.** Both lenses confirmed the maps are accessed **only** by keyed point operations — no `keySet`/`values`/`entrySet`/`forEach`/iteration anywhere — so table layout never feeds a decision. Keys are the identical primitive longs; values are bit-identical (`int` counts, IEEE-754 `double` `firstPos` stored verbatim). The float accumulation (`total += segLen`, `weightedReuse += segLen*posWeight`) iterates `track.nodes` in fixed index order, completely independent of map internals — no reassociation. Two non-negotiable implementation invariants make it safe:
- **`int` count map:** stored counts are provably always ≥1 (`merge` increments; `remove` floors at removal, never stores 0), so a default-0-on-missing map exactly reproduces `prev==null || prev==0`.
- **`double` firstPos map:** `firstPos` can equal `0.0` (one lens found the 1-meter first-edge case where `segLen/2` integer-divides to 0 with `cumDist==0`), so the map **MUST use an explicit present-flag / occupancy bitmap, NOT a sentinel double**. Likewise the int map must use occupancy tracking, not a magic empty-key, because `edgeKey` spans the full 64-bit range and can equal any chosen sentinel (0L, −1L, MIN_VALUE).

**Guard test.** Differential/dual-write parity test: drive a recorded `add`/`remove`/`computeTrackVisitedRatio` sequence from a real corpus run against both the `HashMap` reference and the new maps, asserting identical present/absent status and identical `Double.doubleToRawLongBits(firstPos)` and identical int count on every op — **including** a synthetic first edge of length 1m (`firstPos==0.0`), an absent-key removal, and keys deliberately equal to 0L/−1L/MIN_VALUE/MAX_VALUE. Then end-to-end diff routes + telemetry on the LoopGoldStandard corpus; require zero diff.

---

### SAFE-4 — Cache the merged committed-prefix track per attempt for self-intersection counting
**Status: safe · confidence high · effort M**

**The change.** In the route-top-K loop, `segments` is invariant across `r=0..K-1` (mutated only post-accept at `:496`). Build the merged committed prefix **once** before the r-loop (`mergedPrefix = mergeSegments(segments, null)`, or null when empty). Per candidate, produce the tentative track by copying `mergedPrefix`'s node list and appending only the candidate's nodes (replicating `appendTrack`'s first-node dedupe at `:970-979`), then run the same `countSelfIntersections` scan. Skip `buildMap()` for the tentative track (composes with SAFE-1).

**Determinism argument.** `countSelfIntersections` reads only `track.nodes`; `mergeSegments(segments, null)` folds `appendTrack` identically; appending the candidate against the identical prefix-tail node yields an element-identical node list to today's `mergeSegments(segments, candidate)`. The empty-segments branch maps to an empty prefix. The int `tentativeSelfIntersections` is bit-identical, so `routedScore`, `sortByRoutedScore`, and the winner are unchanged. Both lenses found **no semantic divergence**; the only risk is an implementation defect — appending into a **shared** prefix list instead of a fresh per-candidate copy would leak candidate N's nodes into candidate N+1. Use a fresh copy (or rigorously restore prefix length) per candidate, and never mutate the shared committed `OsmPathElement` objects.

**Guard test.** Unit test asserting `countTentativeSelfIntersections(segments, candidate) == countSelfIntersectionsForTentative(mergedPrefixNodes, candidate)` for every candidate across: empty-segments, single-segment, multi-segment with coincident vs non-coincident endpoints, and a `> MAX_SHAPE_SCAN_NODES` (1500) track that exercises the sampling path. Assert per-candidate node-list identity vs `mergeSegments(...).nodes`, and a defensive assertion that `mergedPrefix.nodes.size()` is unchanged after each per-candidate append (catches the shared-list leak). End-to-end: diff routes + telemetry on the corpus.

---

### SAFE-5 — Precompute per-segment `calcDistance` once, share across the two passes
**Status: safe · confidence high · effort M**

**The change.** `computeTrackVisitedRatio` (`:1038-1062`) and `worstContiguousCostlyMetersForScorer` (`RoundTripQualityGate.java:704-735`) run back-to-back over the **same** subTrack. Compute the per-segment integer distances once into a reused `int[]` buffer (planner field, grown geometrically, cleared/resized per candidate), populated by the visited-ratio pass, and read by the hostility pass instead of recomputing `a.calcDistance(b)`.

**Determinism argument.** The proposal's own stated HIGH risk — a double-vs-int precision mismatch between the passes — **does not exist** in this codebase. `OsmPathElement.calcDistance` returns an `int` (`(int)Math.max(1.0, Math.round(CheapRuler.distance(...)))`); there is no double-returning overload. Both consumers call the identical `double segLen = a.calcDistance(b)` (`:1048` and `RoundTripQualityGate:720`) — the same rounded int widened to double. Caching that int once is exact CSE of a referentially-transparent call; accumulation order (`total += segLen`, `current += segLen`) is untouched. **Critical implementation note (one lens):** use a **single `int[]`**, not the proposal's double+int dual buffer; populate it in the **all-segments visited-ratio pass first** (the hostility pass `continue`s before `calcDistance` on `b.message==null` segments, so it cannot safely populate). Both passes visit the same index range in the same orientation, so a positional buffer is alignment-safe.

> **Do NOT** implement the bundled "scale-bucket strength reduction" half of the original idea — see Rejected. Cache only the `calcDistance` int, never the CheapRuler scale array across edges.

**Guard test.** JUnit assertion that, for every routed subTrack, `buffer[i] == a.calcDistance(b)` for each i; include a track with at least one `b.message==null` segment to prove the null-skip does not desync the positional index. Assert the buffer is cleared/resized per candidate. End-to-end diff of routes, `actualVisitedRatio`, and `worstHostile` telemetry on the corpus.

---

### SAFE-6 — Reuse `cachedRefTrack` for `refBeforeAccept` + drop tentative `buildMap` (per-step prefix cache)
**Status: safe · confidence medium (one lens), high (other) · effort M**

**The change.** Two narrowly-scoped sub-changes:
- (a) Reuse the already-built `cachedRefTrack` (`:247`) for `refBeforeAccept` (`:458`) — both are `mergeSegments(segments, null)` on the identical `segments` list with no mutation in between.
- (b) In `countTentativeSelfIntersections`, build the merged committed prefix once and append only each candidate's nodes (this is SAFE-4) and drop `buildMap` on the tentative track (this is SAFE-1).

**Determinism argument.** `segments` is provably not mutated between `:247` and `:458` (`segments.add` is at `:496`, after `:458`). Routing does not mutate the passed `refTrack`: `RoutingEngine` builds a fresh `OsmTrack` via `addNodes` and calls `buildMap()` on the new track, so `cachedRefTrack` survives the candidate-routing calls unmutated and is byte-identical to a fresh `buildRefTrack` at `:458` (same node sequence, same `FrozenLongMap` chaining). The `:533/:548/:673/:676` `buildRefTrack` calls occur **after** `segments.add`, so they are correctly **not** reused. The medium-confidence caveat: correctness hinges on the implementer copying/truncating the shared prefix per candidate (the prefix-leak hazard from SAFE-4).

**Guard test.** Run the corpus with `DIAGNOSTIC=true` and assert the full `[greedy-diag]` stream is line-for-line identical (per-step/candidate `dist`, `worstHost`, `selfX`, `costPerM`, `score`), plus final node sequence, total distance, error, and `stampTelemetry` counters. A single differing `selfX` would prove a prefix leak. Add the SAFE-4 unit test (coincident vs non-coincident candidate first node).

---

## 4. Rejected Ideas — Cautionary List

Do not re-propose these without first removing the root cause noted.

**Multithreading (all rejected, mostly `high` confidence):**

- **Parallelize route-top-K greedy-sub Dijkstras.** Two independent killers: (1) `islandNodePairs.freezeCount` accumulates across the K candidate routings on one engine — cloned engines each start at 0, flipping `RoutingIslandException` throw/no-throw for candidate r≥1, changing the candidate set. (2) The wall-clock `SUB_ROUTE_TIMEOUT` / per-call budget: under contention a candidate that succeeds sequentially times out (returns null → dropped → different winner). "Disable the timeout in determinism mode" is not bit-identical — production sometimes times out by design.
- **Parallelize per-candidate post-route scoring.** The three named passes are genuinely pure *in isolation*, but the proposal permits chaining onto parallel routing (idea #1), which races `OsmPathElement.message` publication and corrupts `engine.startTime/maxRunningTime`. The narrow standalone form (sequential routing, parallelize only the three pure passes) was *almost* certifiable but the spec's stated target range (`:359-410`, "for all r in parallel") literally encompasses `matchPoint`/`timedFindTrack`/`scorer.score`, so it cannot be certified as written.
- **Overlap ISO_GREEDY pool build (AUTO mode).** Whether GREEDY runs at all is data-dependent on ISO_GREEDY's completed score (`:911-918`) — no correct moment to prelaunch without speculation. Child engines run under a real wall-clock budget read inside the routing loop (`:5164`, `:5096-5101`); contention flips deadline-gated branches. Speculative GREEDY also changes `all.size()` / "Also tried" telemetry baked into `foundTrack.message`.
- **Per-plan() cloned-engine pool.** The cited `oldCache` constructor (`NodesCache.java:85-88`) shares `dataBuffers` **and** `fileCache` → two workers do non-atomic `seek`+`readFully` on one shared `RandomAccessFile` (`PhysicalFile.ra` is mutable, not immutable as claimed) → torn decode → wrong costs. Plus the same timeout non-determinism. The proposal's premise that `NodesCache` allocates per-instance `dataBuffers` is factually wrong for the path it relies on.

**Data-structure / set:**

- **Primitive set for `seenCells` dedupe** and **the existing `CompactLongSet`.** A correctly-implemented set with occupancy tracking would be identical, but: `CompactLongSet.add()` returns `true` when **already present** (inverted vs `HashSet.add`), which flips the dedupe; and a sentinel-key open-addressing set collides with reachable packed cells (e.g. `cell==0L` at the lon/lat corner). Rejected as a real footgun.
- **Reuse a single thread-local primitive map across steps (clear, not realloc).** Inseparable from the not-yet-existing primitive maps; correctness hinges on `clear()` fully zeroing occupancy flags (partial clear silently corrupts the next attempt's reuse counts → different winner) and converts guaranteed-safe method-local state into shared field state on a reused provider.
- **Primitive set for refTrack membership / avoid `buildMap` rebuilds.** `mergeSegments` builds both poisoning-only refTracks AND `finalTrack`/`snap.track` output routes that escape via `populateResult` and need `getLink`/detour/voice-hint holders; a membership-only set NPEs there. No per-track membership-only flag exists.

**Caching / incremental refTrack:**

- **Incrementally extend a cached merged refTrack across steps (all 3 variants).** `segments` is not strictly append-only within a step — undo paths (`:580` closed-loop-rejected, `:618` too-long) remove-then-re-add a *different* detailed track at the same size, so a size-keyed cache serves stale content; `appendTrack`'s first-node dedupe makes each leg's node contribution variable (N or N−1) so naive truncation over-truncates and drops an id belonging to an earlier committed leg, flipping `containsNode` poisoning. Also `buildMap()` freezes into `FrozenLongMap` whose `put` throws, so "incremental map entries" is impossible without a full rebuild. The per-step (within-attempt) cache is fine — that's SAFE-4/SAFE-6; the cross-step incremental version is rejected.
- **Memoize `matchPoint` snapping per (ilon,ilat).** `matchWaypointsToNodes` is **not** a pure function of the coordinate: `WaypointMatcherImpl.start()` skips ways whose endpoint pair is in `engine.islandNodePairs`, which grows monotonically during a plan() via `freezeTempPairs()`. A stale memo returns a snap computed before island pairs were frozen, snapping to a different way → different endpoint → different route. (Would be safe only if keyed on the island-detection generation.)
- **Key GraphNative expansion cache on (pos, radius, refTrack-ref).** One lens passed it (medium); one rejected (medium) on the unenforced mutable-key invariant — the cached pool is keyed on a mutable `OsmTrack` identity, safe only while its content is frozen for the object's lifetime. Classic identity-keyed-mutable-cache trap; not certifiable for all refactors.

**Allocation / IO / micro-opt:**

- **StdPath free-list pool (both proposals).** `init()`/`addAddionalPenalty` do **not** fully reset all fields: `myElement`, `nextForLink`, `airdistance`, `treedepth`, `selev`, `originLon/originLat`, `uphillcostdiv/downhillcostdiv` are left stale or only conditionally written (early-returns at `cost=-1` skip the section loop). `airdistance` feeds the open-set priority key; `originLon` feeds turn-angle → cost. A recycled object silently corrupts cost/track. Also "recycle only losers" is not locally provable (a demoted bestPath, the lazy `origin.myElement` capture, `nextForLink` chains).
- **Dedup double `obtainNonHollowNode` (iso loop ×2, `_findTrack` loop).** The second call is a true no-op **only** for nodes made non-hollow in pass 1 (short-circuits at `line 214` before `collect()`). For neighbors still hollow with a resident segment, the second call re-runs `getSegmentFor`/`getAndClear`/`collect()`, mutating `cacheSum` and segment compaction state — `cacheSum` drives `checkEnableCacheCleaning` (the global GC/eviction schedule), so removing the second call shifts which segments are evicted and which later nodes materialize → different frontier → different route, under GC pressure / tile-boundary inputs. One iso-loop verdict was `true` (medium) because the no-op short-circuit holds for the common case, but the adversarial lens found the hollow-border case and the `_findTrack` variant added a mid-walk `addLink` list-mutation hazard.
- **Last-segment cache in `getSegmentFor`.** The file-level memo alone is safe (`OsmFile` identity is stable per `NodesCache`), but the bundled **MicroCache sub-memo** is unsound: `microCaches[subIdx]` is nulled/ghosted by GC between calls, and a stale memo skips the re-decode + `cacheSum` accounting that feeds the GC-enable threshold → divergent eviction → different route. Split out and ship only the file-level memo if pursued.
- **Scale-bucket strength reduction in `calcDistance`.** Reusing the CheapRuler scale array across "geographically adjacent" edges is wrong at `SCALE_CACHE_INCREMENT` (~0.1° / ~11 km) bucket boundaries: a stale scale changes the sqrt → changes the rounded `segLen` by ≥1m → propagates into `cumDist`/`weightedReuse`/`actualVisitedRatio` → flips the candidate winner. The only safe form recomputes the exact per-edge bucket index, which saves nothing — speedup and correctness are mutually exclusive.
- **"Precompute ROAD_INDIRECTNESS-scaled constants" (bundled with comparator hoist).** `ROAD_INDIRECTNESS` is already a folded `static final 1.3` multiplied by per-candidate-varying distances; "precomputing" can only mean reassociating float math, perturbing `cp.score` in the last ULP and flipping near-tie sorts. The comparator-hoist half is safe (it is part of SAFE-2); the constant-fold half is rejected.

---

## 5. Suggested Verification Harness — "Byte-Identical Route" Regression Check

The single highest-value infrastructure investment is a **golden-output diff harness** that lets every SAFE idea (and any future candidate) be merged with mechanical confidence.

**Corpus.** Use the existing assets:
- `LoopGoldStandardTest` / `LoopTestSegments` (the gold-standard loop suite) as the primary fixture set.
- The phase-2-v5 GPX corpus (`docs/features/phase-2-v5-gpx-corpus-validation.md`) for breadth, including hard/borderline loops.
- `docs/features/roundtrip-closure-aware-baseline.json` as a stored baseline.
- Ensure coverage of: multi-attempt (radius-backoff) steps, closed-loop-rejected undo (`:580`) and too-long undo (`:618`), paved (`fastbike`) profiles exercising `detailAcceptedTrack`/hostility, a `> MAX_SHAPE_SCAN_NODES` (1500-node) accumulated loop, and a first edge of length 1m (`firstPos==0.0`).

**What to capture and diff (the "golden output").** For every fixture, before vs after, assert byte-identical:
1. **Full route node sequence** — ordered `(ilon, ilat)` list of the chosen `OsmTrack`.
2. **Per-step chosen `candidateIndex`** and the ordering of `routedCandidates` entering `sortByRoutedScore`.
3. **Score doubles via `Double.doubleToRawLongBits`** (never `==`) — `cp.score`, `routedScore`, `actualVisitedRatio`, `worstHostile`, `tentativeSelfIntersections`.
4. **Telemetry counters** — `candidatesGenerated`, `candidatesRouted`, `routedIso`, `routedRadial`, `acceptedIsoLegs`, `acceptedRadialLegs`, `returnChecksPerformed`, `totalDistance`, `closedDistance`, `error`, `fallbackReason`, and the AUTO summary in `foundTrack.message`.

**Mechanism.** Add a test mode that serializes the above into a stable text/JSON dump per fixture, checked into the repo as the golden baseline (extend `roundtrip-closure-aware-baseline.json`). The regression test re-runs each fixture and diffs against the golden file; any non-empty diff fails. Run with a fixed seed and a **non-binding deadline** (so cache-induced speedups can't let a run cross a time gate and truncate differently — this matters because SAFE ideas legitimately make runs faster).

**Per-change unit guards.** In addition to the corpus diff, each SAFE idea carries a focused equivalence/parity unit test (detailed in §3) that pins its specific invariant — the primitive-map dual-write parity test, the `countSelfIntersections` with/without-`buildMap` equality, the `int[]` `calcDistance` buffer assertion, and the `pickDiverseTopK` reference-equality test. These are the fast feedback loop; the corpus diff is the gate.

**Determinism stress (defense in depth).** Although all SAFE ideas are single-threaded, run the corpus diff under varied conditions to catch any accidental nondeterminism a future change might introduce: `-Xint`, randomized thread counts (1/2/4/8 — must be no-ops for these changes), and repeated runs. For any future multithreading attempt, the harness MUST run under artificial CPU contention (`taskset -c 0` / `-XX:ActiveProcessorCount=1`) with the production timeout left active and an input near the deadline — that is the only configuration that surfaces the timeout-flip divergence that killed every parallel idea.

---

## 6. Suggested Implementation Order

Sequence to maximize early, low-risk wins and to let later changes compose cleanly on earlier ones.

1. **Build the verification harness first (§5).** Capture the golden baseline on `master` before touching any code. Nothing merges without a green corpus diff.

2. **SAFE-2 — Comparator hoists + identity back-fill (S).** Smallest, lowest-risk, exercises the harness end-to-end. Strip out the rejected "constant-fold" half.

3. **SAFE-1 — Skip `buildMap()` for the tentative merge (S).** Independent, high-frequency win, trivial to verify.

4. **SAFE-4 — Per-attempt committed-prefix cache for self-intersection counting (M).** Builds directly on SAFE-1 (the tentative track already skips `buildMap`). Land the prefix-leak unit guard with it.

5. **SAFE-6 — `cachedRefTrack` reuse for `refBeforeAccept` (M).** The (a) half is mechanical; the (b) half is already done via SAFE-1+SAFE-4. Verify with the `DIAGNOSTIC` line-for-line diff.

6. **SAFE-5 — Single `int[]` `calcDistance` buffer shared across the two passes (M).** Independent of the merge work; verify with the per-edge buffer assertion plus the null-`message` segment case. Explicitly do **not** add scale-bucket caching.

7. **SAFE-3 — Primitive long-keyed visited-edge maps (M).** Land last among the SAFE set because it touches the most accessors and demands the strictest implementation discipline (occupancy flags, not sentinels; present-flag for `firstPos==0.0`; default-0 equivalence for counts). Ship behind the dual-write parity test, then remove the dual-write once the corpus diff is green over the full corpus including the 1m-first-edge and full-range-key cases.

After all six: re-run the entire corpus diff plus the determinism-stress matrix one final time, then refresh the stored golden baseline. **Do not attempt any multithreading idea** until the per-sub-route timeout is made data-deterministic (step/node-count budget instead of wall-clock) and `islandNodePairs.freezeCount` is made per-candidate-reproducible — both are prerequisites the entire rejected parallel cohort failed on.