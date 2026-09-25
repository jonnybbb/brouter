# Spec: Refinement Stage for the Round-Trip Planner

**Repo:** `jonnybbb/brouter` (fork of `abrensch/brouter`)
**Status:** Draft 0.3, 2026-09-24. This is the single source of truth for the refinement work.
**Owner:** Johannes (human checkpoints)
**Implementer:** coding agent with full repository access

**Draft history**
- **0.2:** written without reading the code.
- **0.3:** checked against the code by three verification passes and three independent Codex reviews (the spec, the implementation plan, and this merged document). The implementation plan is merged in here. The owner delegated the open decisions; they are recorded in §3. A cost-oracle feasibility spike ran on 2026-09-24; its results are in §2.9.
- Where 0.3 differs from 0.2, the reason is in Appendix A. There is no other document to read.

Path shorthands: `RT/` = `brouter-core/src/main/java/btools/router/roundtrip/`, `RE` = `brouter-core/src/main/java/btools/router/RoutingEngine.java`. Line numbers refer to commit `eec40289`.

---

## 0. Start here (for the agent)

### 0.1 Setup

1. **Baseline.** Work on a branch created from **`eec40289`**, the head of PR #44 (`fork/roundtrip-gravel-residential-starts`). Fetch it with `git fetch fork roundtrip-gravel-residential-starts`.
   - `master` does **not** contain PR #44: no `docs/adr/`, no loop glossary, no closure levers, no harness switches. Do not start from `master`.
   - Rebase onto `master` once PR #44 is merged.
2. **Read first:**
   - this document;
   - `CONTEXT.md` (glossary);
   - `docs/adr/0001-profile-cost-is-the-loop-objective.md` and `docs/adr/0002-closing-first-loop-planning.md`;
   - `docs/features/roundtrips.md`;
   - `docs/developers/testing_roundtrips.md`. Its cell counts are stale; see §2.6.
3. **Build and test.** Run Gradle through `gain` instead of `./gradlew`, and read its summary before opening the logs in `.gain/`.
   - Unit tests: `:brouter-core:test`. It depends on `:brouter-map-creator:test`, which builds the Dreieich fixture.
   - Loop matrix: `:brouter-core:integrationTest`. It is local only, not run in CI, and takes about 25–40 minutes.
   - The matrix needs real `.rd5` tiles in `<repo>/segments4`, which are downloaded on demand. About 23 tiles are already present in the main checkout.
   - Use `-Dloop.forks=1` for any latency measurement.
   - **Long jobs:** a full matrix run takes longer than a single tool call is allowed to run. Start matrix and integration runs in the background, poll the `.gain/` logs, and plan for several runs per milestone. Never re-run a finished build just to read its output; read the logs instead.
   - PMD and checkstyle run on every source set as part of `build`.

### 0.2 Glossary

- Use the terms as `CONTEXT.md` defines them: *loop*, *leg*, *closing leg*, *filler leg*, *skeleton*, *final via*, *variety seed*, *direction focus*, *score jitter*, *loop cost*.
- Add the new terms from §4.1 to `CONTEXT.md` in M5.

### 0.3 Invariants (never break these)

1. **Refine off ⇒ bit-identical output.** The parity goldens (`GreedyPlannerParityTest`, 12 keys) and loop signatures (`LoopGoldenSignatureTest`, 9 scenarios) stay unchanged. Refine-off output for WAYPOINT, AUTO and explicit-via requests stays unchanged too.
2. **ADR-0001:** the profile's cost per metre is the loop objective. Tag-based road character (`RoadCharacterScore`, residential and track share) may *evaluate* a loop but never *steer* the search. RouteChoiceScore (RCS) is allowed only as a filter on **finished** loops (§3, D2).
3. **Never worse:** the shipped loop is never worse than the tier's loop under the ship predicate (§4.5). When refinement does not apply, the tier's loop ships byte-identical, with its warnings, verdict and output.
4. **Determinism, scoped:** a fixed start state, resolved direction, map and profile inputs, and configuration, with no binding timeout, always give the same output. Every binding inner timeout sets `refineTruncated`. The existing tiers already depend on the wall clock (§2.5), so request-level determinism under load is **not** promised.
5. **Direction focus:** the refined loop keeps the direction rule in §4.4.
6. **ADR-0002 and the pinned NOTEs:** do not re-attempt any of these (§2.7):
   - closing-first planning;
   - closing leg through an approach via;
   - the preferred-area heatmap;
   - the other measured dead ends listed in §2.7.

### 0.4 Stop and ask before

- changing any existing tier's output when refinement is off;
- adding a third-party dependency or a Java/Android API outside §2.8;
- recapturing parity or loop-signature goldens for any reason;
- running more than **two tuning rounds per experiment step** (counted in total, not per lever);
- starting the next milestone after a checkpoint (§6);
- merging, deploying, or pushing to `master`;
- any change that alters a row of the contract table (§4.5) or a decision in §3.

### 0.5 Recording decisions

- Record every non-trivial decision as an ADR in `docs/adr/`, in the style of the existing ones. The planned ADRs are ADR-0003 (placement), ADR-0004 (energy and ship predicate) and ADR-0005 (outcome).
- Measured-and-rejected ideas are recorded too, with their numbers.
- Keep the working notes in `docs/refine-notes.md`.

---

## 1. Goal

Add an **opt-in refinement stage** that takes the loop a planner tier shipped and tries to improve it by moving vias of its skeleton. The stage re-routes only the legs a change affects, and ships the result only if the finished loop passes the ship predicate (§4.5).

**Hypothesis to test:**
- refinement lowers loop cost/m;
- it doesn't worsen the absolute length error, self-crossings or scatter reuse (§4.5);
- it does this within ≤ 3 s of added p90 latency.

**The deliverable is a measured answer, not code.** The answer is either a shipped, opt-in feature backed by A/B numbers, or a "measured and rejected" ADR that names the exact failure mode (pricing, latency, fidelity, or search effectiveness). Both are valid outcomes.

### Non-goals

- **A second cost model, or steering by road character** (ADR-0001).
- **Fixing residential share at the start and end of the loop.** ADR-0002 measured that the start's surroundings bound both ends. Report these metrics, but don't tune for them.
- **UI changes, and changes to classic point-to-point alternatives.**
- **Parallel routing inside a request, a standalone optimizer service, or a non-BRouter leg router** (Rust/CCH). These come later, if at all.

---

## 2. What exists today (verified at `eec40289`)

### 2.1 Tiers and dispatch

- **Tier names** (`RT/RoundTripAlgorithm.java:24-76`):
  - The enum is `AUTO, BALANCED, QUALITY, WAYPOINT, ISOCHRONE, GREEDY, ISO_GREEDY`.
  - `FAST` is a parse alias for `WAYPOINT`, and FAST is the default (system property `roundtrip.default.algorithm`).
  - Unknown values fall back to AUTO.
- **Strategies**, wired at `RT/RoundTripOrchestrator.java:206-218`:
  - `FastStrategy` runs WAYPOINT and ISOCHRONE.
  - `GreedyStrategy` runs GREEDY and ISO_GREEDY.
  - `BoundedStrategy` runs BALANCED, and AUTO when resources are constrained.
  - `AutoCompetitionStrategy` runs AUTO and QUALITY.
- **Ladder** (`resolveLadder`, `:69-160`): AUTO uses WAYPOINT for fast motor profiles, and otherwise the competition, or the bounded tier on the BOUNDED preset.
- **The AUTO competition runs child engines one after another** (`AutoCompetitionStrategy.java:128-218, 361-449`):
  - Each child is a new `RoutingEngine` with `copyRequestFields()`, `roundTripSuppressDecoration=true`, and its own cold `NodesCache`.
  - Children are ranked by RCS.
  - `adoptCandidateWinner` (`:288-319`) never sets a planner result on the parent.
- **`doRoundTrip`** (`RT/RoundTripOrchestrator.java:312-666`) runs these steps in order:
  1. the ladder loop (`:408-413`);
  2. floors (`:427-464`);
  3. the gate (`:474-485`);
  4. decoration (`:490-608`), skipped in AUTO children;
  5. deferred output (`:614`);
  6. a `finally` block that publishes the result and calls `cleanupRoutingResources()`.
  The outer catch clears the route on exceptions (`:620`). WAYPOINT may write its output before the gate (`RE:393`).
- The effort presets are BOUNDED, STANDARD and MAX (`RT/RoundTripEffortPolicy.java`). BOUNDED is chosen when the budget is ≤ 10 s or `memoryclass` ≤ 48.

### 2.2 Result objects and the skeleton

- **`RoundTripResult`** exists only when greedy shipped. FAST/WAYPOINT and the AUTO parent have none.
  - It exposes the track, loop waypoints (coordinates), a copy of the matched waypoints, `getLegTracks()` and telemetry.
  - Leg tracks are **pre-cleanup**. `RoundTripTrackCleanup.finalizeAdoptedRoundTripTrack` (`RT/RoundTripTrackCleanup.java:75-117`) then removes back-and-forth, micro-detours, bulges and spur spans. It updates waypoint indices to the nearest remaining node, without updating their matched edge or crosspoint (`:119`, `:195ff`).
- **The only tier-independent skeleton** is `track.getMatchedWaypoints()` with `indexInTrack`. Its fidelity after cleanup must be validated (M0).
- **Node identity:**
  - `OsmNode.getIdFromPos()` is packed ilon/ilat, not an OSM id.
  - A `MatchedWaypoint` is an edge (`node1`, `node2`) plus a crosspoint and matching state (`brouter-mapaccess/.../MatchedWaypoint.java:20`).

### 2.3 Leg routing

- **Interface.** `RT/LegRouter.java` is implemented by the adapter at `RE:922-1200`. Every call takes already-matched `MatchedWaypoint`s. The methods:
  - `findTrackTimed(op, from, to, refTrack, budgetMs)` (`RE:3408-3422`): a single goal-directed pass, **no tags**.
  - `findTrack(...)` (`RE:3383`).
  - `findTrackUnguided`.
  - `retrackForDetail(raw, from, to, ref)` (`RE:3267-3309`): adds tags. It **ignores `refTrack`**, can return the raw track on failure, and uses a 60 s fallback budget for untimed calls (`RE:3250`).
  - `profileAwareMatchPoint`: a hostile-road relocation probe, up to 16 ring matches (`RT/WaypointSnapper.java:113-196`).
  - `matchWaypointsToNodes`.
- **The planner's timeout wrapper** caps each leg at `min(10 s, 2000 ms + 700 ms × air-km, remaining)` and returns `null` on failure (`GreedyRoundTripPlanner.java:2232-2270`).
- **Reuse penalty.** Planner legs are *selected* with a `refTrack` anti-reuse penalty (`GreedyRoundTripPlanner.java:995, 1405, 2140`; `OsmPath.java:371-377`; `RE:2362-2385`). The route a leg takes therefore depends on the other legs.
- **Cost is path-state dependent:**
  - classifier and turn history are carried along the path (`OsmPath.java:80`);
  - a leg start resets cost and turn history (`OsmPath.java:252`);
  - elevation hysteresis resets per path (`StdPath.java:36`).
  So the summed leg cost changes when the same ride is split into different legs.
- **After cleanup, `track.cost` is stale.** `recalcTrack` recomputes distance but not cost (`RE:2924`, distance at `:3007`).
- **Raw legs are not the ridden geometry.** Raw path elements end at graph nodes; clipped crosspoints and transfer nodes exist only in detail mode (`OsmPath.java:259`, `OsmPathElement.java:84`, `RE:3805`).
- **Cache resets.** Every call resets `NodesCache` (`RE:3311-3327, 3390`). A same-mode reset reuses the file caches but builds a fresh graph map (`NodesCache.java:64-101`). Search unlinks graph links (`RE:3657`), so the reset is required, not waste.
- **Not thread-safe.** `findTrack` mutates `nodesCache`, `openSet`, the nogo list, `guideTrack`, the timers and `islandNodePairs`. `islandNodePairs` is learned during the request, so snapping can depend on call order.
- **"Compiled leg cost" is not point-to-point.** It is a by-product of the one-to-many isochrone expansion (`RE:1778-1785`).
- **No per-leg timing exists** in code or tests. `RoundTripPerfBudgetTest` pins only `linksProcessed` ceilings.
- **Guided re-pricing semantics:** see the spike results in §2.9.

### 2.4 Scoring, gate and metrics

- **`RouteChoiceScore`** (`RT/RouteChoiceScore.java`): higher is better, clamped to [0,1] (`:503`). A gate-rejected loop scores 0.
  - Positive terms:
    - distance 0.18, flat within ±15 % (`:314`);
    - reuse 0.20;
    - closure 0.10;
    - continuity 0.15;
    - compactness 0.10;
    - **road character (tags) 0.17** (`:54, 378`);
    - cost/m band 0.05 (0 for mtb);
    - direction 0.05.
  - Shape penalties: out-and-back, lollipop, crossings at 0.08 each, lasso, teardrop, petal, dwell, crumple.
  - Uses: greedy closure ranking (`GreedyRoundTripPlanner.java:1878-1947`), the ISO_GREEDY internal comparison, and the AUTO ranking and second opinion.
- **`RoundTripQualityGate.evaluate`** (`RT/RoundTripQualityGate.java:188-374`) is `public static`, deterministic, and callable on any track. Its checks:
  - structural validity;
  - closure ≤ 400 m;
  - **distance ratio 0.5–1.8** (not ±5 %);
  - direct/beeline and ferry segments;
  - chaos: > 5 crossings or > 20 hairpins over 130°;
  - paved-profile hostility;
  - `ReuseClassifier`: mid-route retrace > 8 %.
  Beeline, ferry, hostility, continuity and the bridge/tunnel crossing exemption (`:440`) read `wayKeyValues`, so only **detailed** tracks get a correct verdict. The orchestrator wrapper `evaluateRoundTripGate` adds forced-corridor handling and is package-private.
- **Length.** ±5 % is only greedy's closure *tolerance* (`GreedyRoundTripPlanner.java:44`). FAST uses 10 % (`FastStrategy.java:322`). A loop that misses the tolerance ships with `withinTolerance=false`. "Q16" does not exist anywhere in the code.
- **κ** (`RT/ReturnDistanceOracle.java:34-135`) is a scalar 10th-percentile cost per **air**-metre. It is built only for ISO_GREEDY and not stored on the result.
- **`CandidateScorer`:** lower is better. `DeferredCommit` exists as a private nested class (`GreedyRoundTripPlanner.java:1973-1984`). `closureLeverStrength` is 0 for paved profiles.
- **`LoopQualityMetrics` / `LoopAnalysis`** (package-private):
  - reuse % (`:367`) and `reuseStemSplit`, stem vs scatter (`:438`);
  - self-intersections;
  - spurs;
  - `computeDirectionDelta`: the **absolute** difference between the requested bearing and the bearing to the farthest point (`:629-652`, `CheapAngleMeter.java:81`);
  - compactness, petal and dwell.
  There is no reversal metric; the gate's hairpin count is the closest.

### 2.5 Direction focus, variety seed, determinism

- **Direction resolution** (`RT/RoundTripOrchestrator.java:292-367`):
  - `direction`/`heading` set the internal `startDirection`; `heading` also forces the opening leg (`RE:3466`).
  - If absent, the profile-aware default bearing is used, except for WAYPOINT, explicit vias and same-way-back.
  - Otherwise `getRandomDirectionFromData`, which uses **`Math.random()`** (`RE:1882, 2016, 2020`).
- **Seed.** Variety seed = `max(0, alternativeidx)` (`RoutingContext.java:43-45`). Jitter is the splitmix hash `seededUnit` (`GreedyRoundTripPlanner.java:467-477`).
- **Existing perturbation bounds.** FAST knobs are bounded at ±15° phase, ±3 % radius and ±1 point (`FastStrategy.java:41-57, 91-98`). Greedy jitter is ±10 % of the heuristic score and has **no angular bound**. No existing code checks direction on a finished loop.
- **Wall clock already decides outcomes today:**
  - AUTO children and second opinions;
  - closure trials;
  - step deadlines;
  - timed Dijkstra;
  - ladder gates (`AutoCompetitionStrategy.java:121-210`; `GreedyRoundTripPlanner.java:913-1006, 1886-1900`; `GreedyStrategy.java:302, 582-588`).
- **Determinism tests** are all "fresh engine, run twice": `GreedyPlannerParityTest`, `RoundTripVarietySeedSentinelTest`, `RoundTripContractTest.deterministic`. No test uses threads.

### 2.6 Harness, goldens, CI

- **Loop matrix.** `brouter-core/src/integrationTest/java/btools/router/LoopQualityTestBase.java` has 14 per-region shards.
  - Cells: distance {30, 50, 75, 80, 100 km} × profile × direction {N, E, S, W}. S is skipped for Coastal Nice and Mallorca.
  - Cell counts: **gravel 230, fastbike 230, mtb 88**. The "460/444" figures in older texts count per-variant results.
  - The harness never sets `alternativeidx`, so every cell runs seed 0.
- **Variants.** AUTO runs only when smoke mode is off (`LoopQualityTestBase.java:218-219`). GREEDY and ISO_GREEDY run with `-Dloop.smoke` or `-Dloop.reportVariants`. M0 and M1 must include post-AUTO cases explicitly; smoke alone is not enough.
- **Switches** are forwarded in `brouter-core/build.gradle:95-114`: `loop.profiles`, `loop.profileParams`, `loop.directions=none|random`, `loop.algorithms`, `loop.smoke`, `golden.write`. Gradle itself reads `loop.forks` and `loop.heap`.
- **Results:** one JSON file per result in `brouter-core/build/reports/loops/.results/`, rendered by `generateLoopReport`.
  - Persisted: reuse, distance ratio, direction Δ, cost/m, crossings, spurs, residential/track share including head and tail.
  - **Not persisted:** request time, gate verdict, RCS, gate cost/m. RCS and gate cost/m are printed to stdout only.
  - Coordinates are simplified, so a result can't be replayed.
- **Goldens:**
  - Parity (`GreedyPlannerParityTest`): 12 keys, runs in `test`, recapture with `-Dgreedy.parity.print=true`.
  - Loop signatures (`LoopGoldenSignatureTest`): 9 scenarios, runs in `integrationTest` and `integrationSmoke` only, recapture with `-Dgolden.write=true`.
- **CI** (`.github/workflows/gradle.yml:34`) runs `build`, which includes core `test`, PMD and checkstyle, plus the map-creator and server integration tests. It does **not** run the core loop matrix or the signature goldens, so run those locally.
- **Reference numbers** (gravel, PR #44, historical `track.cost/distance`):

  | Metric | Value |
  |---|---|
  | Loop cost/m | 2.954 |
  | Mean distance ratio | 0.944 (a signed shortfall, not an absolute error) |
  | Last-15 % residential | 30.1 pt |
  | Track share | 56.5 pt |
  | Request time p90 | 9.4 s |

  They are not comparable with the oracle cost of §4.3. Re-measure in M0.

### 2.7 Measured dead ends (do not re-attempt)

- **ADR-0002.** Closing-first planning, rejected after measurement. Pricing alternative closing legs "through an approach via" overshoots ±5 % in about 80 % of cases. Local via relocation (2-opt) was dismissed because it "cannot move a bad final via without moving the whole tail". The finding: the start's surroundings bound both ends of the loop.
- **Pinned NOTEs:**
  - adopting compiled step-1 legs (`GreedyStrategy.java:139-149`);
  - the closure-lever topology fade (`GreedyRoundTripPlanner.java:1054-1059`);
  - the dynamic per-step target, which raised crossings (`:1082-1084`);
  - the GREEDY return oracle in scoring (`:429-443`);
  - snap-distance scoring (`WaypointSnapper.java:147-160`);
  - via relocation on marginal wins, which cost 6× runtime and once flipped a loop to OUT_AND_BACK; hence `VIA_RELOCATION_LOOP_FRACTION=0.25` (`WaypointSnapper.java:786-805`);
  - circle-retry relocation (`FastStrategy.java:660-676`);
  - the sub-600 m near-revisit floor (`docs/features/roundtrips.md`);
  - the preferred-area heatmap (removed in commit `3a0bfd2b`; see `CONTEXT.md`).

### 2.8 Language, dependencies, visibility

- Everything builds with `options.release = 11` on a JDK 17 toolchain.
- `brouter-core` has no third-party dependencies, and **the Android app depends on it** (`minSdk 23`, no core-library desugaring).
  - Forbidden in core: `SplittableRandom`, streams, `Optional`, `java.util.function`, `CompletableFuture` (all API 24+).
  - Use hand-rolled splitmix hashing for randomness.
- **Package-private:** `RoundTripRequest`, `LoopAnalysis`, `computeDirectionDelta` and the orchestrator's gate method. The stage adapter therefore lives in `btools.router.roundtrip`.
- **Request parameters** are parsed in `RoutingParamCollector.setParams` (`:196-280`), use `roundTripXxx` camelCase, and are copied to children in `RoutingContext.copyRequestFields` (`:709-735`). The docs table is at `docs/features/roundtrips.md:21-31`.
- **PMD** uses quickstart minus 35 exclusions, with no size rules. Rules to watch: `UnnecessaryImport` (including same-package imports), `LooseCoupling`, `NonExhaustiveSwitch`, and the naming rules.

---

### 2.9 Cost-oracle spike (2026-09-24)

A throwaway test on the Dreieich fixture ran 40 shipped loops (gravel, trekking, fastbike; GREEDY, ISO_GREEDY, AUTO; 4 directions; r = 1000/2000 m). The test code is not in the repo. Findings:

1. **Per-leg exact re-pricing works.**
   - Method: route a raw leg with `findTrackTimed(from, to, refTrack=null)`, then call `retrackForDetail(copy of raw nodes with cost lifted, from, to)` using the **same** matched waypoints.
   - The result reproduced the routed cost exactly in **40/40** loops, and returned full detail (for example 27 raw → 60 detailed nodes).
   - Cost is about 1–5 ms per loop on the fixture. That is not representative of real tiles; M0.6 measures it properly.
2. **Guided search needs a raw, junction-level guide.**
   - The guide is matched by index: `guide[treedepth + 1]`, where the start path reaches its first graph node at `treedepth` 1 (`RE:3329-3380, 3717-3733`).
   - A **detailed** track as guide fails, because its transfer nodes shift the index. This includes shipped tracks and `retrackForDetail` output.
   - Guided search also fails when the guide's first node doesn't line up with the matched waypoint's edge. For example, shipped via crosspoints sit on a different edge than the one the track leaves by.
   - Keep the guide's `cost` high: the search is capped at `guide.cost + 5000` (`RE:3428`).
3. **The existing API cannot price an arbitrary node sequence:** not a closed loop (start = end), not a cleaned shipped track, and not a split at arbitrary junctions.
   - With start = end, the search ends immediately after 1–2 nodes, because the end edge is reached at once.
   - With self-built start/end waypoints, every attempt failed.
   - Continuous pricing across vias (§4.3) therefore needs **new engine code**: a linear path walker (M0.1).
4. **The comparator problem is as large as the bar.**
   - Re-routing the shipped skeleton's legs raw gives costs **−1.1 % to +2.1 %** different from the shipped `track.cost`.
   - That is the same size as the 2 % bar, so an A/B without an exact oracle for *both* sides would measure noise.
5. **Skeleton index drift is confirmed.** After cleanup, a via's `indexInTrack` can point at a different node than its crosspoint. Seen in `GREEDY|gravel|90|1000`: node …341 instead of …891.

## 3. Decisions (2026-09-24, delegated by the owner)

- **D1: Search energy.** Oracle cost/m (§4.3) plus explicit constraints. **Not RCS.**
  - ADR-0001 explicitly rejects RCS as a planning target.
  - Using RCS would mean amending ADR-0001 first, a policy change nothing here justifies.
- **D2: Ship predicate.** The cost predicate plus an RCS guard (δ = 0.02), applied to finished loops only (§4.5).
  - ADR-0001 permits RCS for comparing already-built loops.
  - Without the guard, the parent could ship a loop that AUTO's own ranking rejects.
- **D3: Final via.** The final via stays frozen, and DELETE is excluded (DELETE can change which via is final).
  - ADR-0002 predicts little gain from moving it.
  - It is reopened only as an M4 extension, and only if M1 traces show the gain is blocked at the tail.
- **D4: Baseline.** `eec40289`.
  - The integration branch `claude/branch-integration-cleanup-t9nwan` also carries an upstream merge and recaptured goldens, which would confound refine-off parity.
- **D5: Latency and scope.**
  - Added latency ≤ **3 s p90**, paired within the same request.
  - Refinement is allowed only for GREEDY, ISO_GREEDY and AUTO/QUALITY on non-BOUNDED presets. FAST/WAYPOINT and BOUNDED never refine.
  - If M1 finds wins only above 3 s, one extra variant is allowed: QUALITY-only with ≤ 8 s added. No further budget increases.
  - A rejection ADR is an accepted outcome.
- **D6: Profiles.** Gravel for the pilot and tuning. Fastbike gets correctness checks in M1 and full numbers in M3, with no quality bars; correctness, resource and refine-off guarantees still apply.
- **D7: Default.** Opt-in. Whether AUTO refines by default is decided in M5 from the M3 numbers.
- **D8: Placement.** Option A: a stage inside `brouter-core`, recorded in ADR-0003. There is no separate service; that would duplicate the harness, the goldens and the determinism plumbing.

---

## 4. Design

### 4.1 Terms (add to `CONTEXT.md` in M5)

- **Refinement:** the post-tier stage.
- **Mutation / proposal:** one change to the skeleton.
- **Evaluation:** routing and scoring one proposal on raw legs.
- **Raw leg:** a leg from `findTrackTimed` without `refTrack`.
- **Finalist:** a candidate put through full finalization.
- **Cost oracle:** the exact re-pricing of a finished route (§4.3).
- **Chain:** one independent search run.

### 4.2 Pipeline

```text
tier ships loop (baseline, immutable)
  → eligibility check (§4.6)            → skip with reason
  → initialize: skeleton from matched waypoints, re-route n raw legs (no refTrack)
  → search over proposals on raw legs   (evaluation-count driven, no wall clock)
  → top-k raw candidates → finalize each (detail retrack, merge, cleanup, gate, oracle, RCS)
  → best finalist passing the ship predicate → publish atomically
  → otherwise baseline ships unchanged, refineApplied=false + reason
```

### 4.3 Cost oracle and search energy

- **Cost oracle.** `LoopCostOracle.price(route, ctx) → cost/m` re-prices an exact node sequence under the active profile. It is defined as:
  - one continuous path over the whole loop: no turn or elevation reset at vias;
  - clipped endpoints handled;
  - no `refTrack` penalty.
  Baseline and candidate are priced with the same function. Historical `track.cost/distance` is reported in a separate column.
- **Implementation.** The spike (§2.9) showed that guided search can't do this. M0.1 therefore adds a package-private **linear path walker** to `RoutingEngine`: from a start edge, it follows a given junction sequence link by link through `routingContext.createPath` (the same cost model the search uses) and returns the accumulated cost. It needs no open set and no search.
  - It must handle transfer nodes: it reduces a detailed or cleaned track to its junction sequence by matching link geometry.
  - It must handle closed loops.
  - It is unused when refine is off.
  - It is validated against the spike's exact per-leg result: walking a raw leg's nodes must equal that leg's routed cost.
  If the walker can't be built, the fallback oracle is **per-leg pricing** (the spike's working method). With that fallback, segmentation is fixed at one leg per via pair, and the baseline must be re-priced the same way. Record the choice in ADR-0004.
- **Search energy.** Oracle cost/m of the concatenated raw legs, with endpoints clipped, priced as one continuous path across vias.
  - A candidate that can't be priced is infeasible and never becomes a search state.
  - Skeleton constraints (§4.4) are **hard rejections** before routing. The pilot has no soft penalties, so every search state is feasible.
  - The pilot uses no weighted geometry terms. Adding any later (for example scatter reuse) requires ADR-0004 to define it, with its numeric weight, as a constraint or declared surrogate; never tags.
- **Fidelity.** The search sees raw legs; shipping sees finished routes. M1 measures, within each request:
  - ranking agreement between the two;
  - sign agreement for gains near 0.5 %;
  - the share of raw winners that survive finalization.

### 4.4 Mutations and constraints

**Pilot operator: MOVE only.**
- Offset: Gaussian, from Box–Muller on the splitmix RNG, applied to one non-start, non-final via.
- The new position is snapped with plain `matchWaypointsToNodes`, through a snap cache keyed by the quantised coordinate.

**Skeleton constraints**, checked before routing and again after snapping. A violating proposal is not routed and does not count as an evaluation (it does count as a proposal, §5).
- **Displacement:** at most `0.25 × searchRadius` **from the via's original position**, so repeated moves can't drift.
- **Via spacing:** at least 300 m between adjacent vias, measured after snapping.
- **Via count:** unchanged in the pilot.
- **Radius:** a bound derived from M0 measurements of shipped loops. On a circular loop the farthest point is `L/π ≈ 0.32 L` from the start. Don't guess the bound.
- **Direction** is not a skeleton constraint. It depends on the cleaned route, so it is checked **after finalization**, inside the ship predicate (§4.5):
  - The bearing from the start to the farthest point of the cleaned route may differ from the baseline's by at most **15°**. Use a signed difference wrapped to ±180°; comparing two `computeDirectionDelta` values can't detect an east/west flip.
  - Ties for the farthest point go to the earliest track index.
  - The check is waived, with a reason, only when the **baseline's** farthest point is under 500 m away.
  - With a forced `heading`, the opening leg is routed with `forceUseStartDirection`; internal legs are not.
- **ADR-0002:** no mutation targets the closing leg specifically, and the final via is frozen (D3).
- **Proposal limit:** `maxProposals = 4 × evaluation budget`, so an all-invalid stream still ends. A no-op snap (same matched waypoint) is not routed.

**Deferred to M4:** INSERT (never on the closing leg), DELETE, WORST_LEG, and final-via moves. For WORST_LEG, badness = the leg's oracle cost/m relative to the loop's median leg, plus its reuse share. κ is not used; it is per air-metre and missing outside ISO_GREEDY.

### 4.5 Contract table (authoritative)

| Contract | Rule |
|---|---|
| **Ship predicate** | All of these must hold for a finished candidate: `C_ref ≤ 0.995 · C_base` (oracle cost/m); the length rule below; self-crossings ≤ baseline; scatter reuse ≤ baseline; the direction rule (§4.4); the gate passes with the full request context (paved classification, `allowSamewayback`, ferry, explicit-via, forced corridor); `RCS_ref ≥ RCS_base − 0.02`. |
| **Length** | `\|L_ref/L_req − 1\| ≤ \|L_base/L_req − 1\|`: the replacement is never further from the requested length than the baseline. Tier tolerance is reporting metadata only. |
| **Reuse** | Scatter reuse = `LoopQualityMetrics.reuseStemSplit(nodes)[1]` (metres). Total and stem reuse are reported separately but aren't part of the predicate. |
| **Comparator** | Every candidate is compared with the **original tier loop**, never with a chain's own start state. |
| **Records** | Keep three separately: the current search state (always feasible in the pilot), the best finalized replacement, and the baseline. |
| **Finalization** | The raw-best top-k (k ≤ 3) go through the finalizer; the best one passing the predicate ships. |
| **Publication** | On acceptance, these are updated together, atomically: `request.track`, verdict, local `quality`, `lastResult`, matched waypoints and deferred output. On rejection or a recoverable failure, the baseline route, waypoints, verdict and serialized route output stay byte-identical. Refine diagnostics (§7) are published **separately, regardless of outcome**. With refine off, no fields are added to the existing route output. |
| **Deadlines** | Every expensive operation (snapping, routing, pricing, retrack, cleanup) is bounded by the earlier of the request deadline and the stage deadline, and returns a distinguishable *success / timeout / failure / cancelled* outcome. A raw-geometry fallback from `retrackForDetail` is **not** a successful finalization. Any timeout sets `refineTruncated`. Cancellation propagates as today. |
| **Ownership** | The baseline is never mutated. Cached raw legs are copied before finalization. The finalizer writes no engine state until `publish()`. |
| **Determinism** | See §0.3 item 4. The search reads no clock. Randomness is splitmix over (variety seed, chain index, step). |

### 4.6 Hook and eligibility

- **Hook.** In `RoundTripOrchestrator.doRoundTrip`, after the gate (~`:485`) and before decoration (`:490`), inside the `try`, so it runs before `cleanupRoutingResources()`. With refine off it is a single `if`.
- **Skip reasons**, each recorded in the diagnostics:
  - `refine_off`;
  - `auto_child` (`roundTripSuppressDecoration`);
  - `explicit_vias`;
  - `tier_not_supported`: the baseline was **produced** by WAYPOINT or ISOCHRONE, regardless of the requested algorithm;
  - `bounded_preset`;
  - `samewayback`;
  - `forced_corridor`;
  - `gate_rejected`;
  - `no_route`;
  - `too_few_vias`.
- **Eligibility** requires both an allowed *requested* algorithm (GREEDY, ISO_GREEDY, AUTO, QUALITY) and a baseline *produced* by GREEDY or ISO_GREEDY. AUTO can resolve straight to WAYPOINT or adopt a fallback (`RT/RoundTripOrchestrator.java:98-112, 158-159`), so record the producing tier through AUTO's adoption and check that.
- **Refine parameters are never propagated to AUTO children** through `copyRequestFields`.
- **Effort policy:** add a `refineAllowed` field to `RoundTripEffortPolicy` (false for BOUNDED).
- **Caches:** discard the baseline's cached gate verdict and `LoopAnalysis` before evaluating a replacement. On acceptance, keep the replacement's newly computed verdict and analysis.

### 4.7 Components

All components live in `btools.router.roundtrip`. Any optimizer subpackage takes only immutable inputs.

| Component | Role |
|---|---|
| `RefineConfig` | Every tunable parameter (§9) |
| `RefineStage` | Hook adapter: eligibility, orchestration, publish |
| `RefineSkeleton` | Immutable: start plus ordered `MatchedWaypoint` vias |
| `LegEvaluator` | Interface `route(from, to) → RawLeg`. The implementation uses `LegRouter.findTrackTimed(…, refTrack=null, …)`, so a different leg router can be plugged in later |
| `LegCache` | Per request, directional. Key: for each endpoint, the edge (`node1`, `node2` `idFromPos`) plus the crosspoint. Stores only complete successes, never timeouts |
| `LoopCostOracle` | §4.3; built on the new package-private path walker in `RoutingEngine` |
| `RefineFinalizer` | Returns `FinishedCandidate{track, matchedWaypoints, verdict, oracleCost, rcs, diagnostics}` |
| `ShipPredicate` | §4.5 |
| `RefineSearch` | Best-of-N / local search / annealing (§5) |
| `RefineDiagnostics` | §7 |

**Seams** are checked by directed-edge plus crosspoint identity, not `idFromPos` equality. On a mismatch the candidate is infeasible, and a counter is incremented.

---

## 5. Search

- **The budget is counted in evaluations, never in time.** `roundTripRefineMaxMs` is only a safety cap; hitting it sets `refineTruncated`.
- **Counting:**
  - A **proposal** counts when it is generated.
  - An **evaluation** counts once a proposal passes pre-routing validation. This includes evaluations served entirely from the cache, and evaluations whose routing or pricing then fails.
  - The proposal limit is 4 × the evaluation budget.
- **Pilot strategies**, compared at evaluation budgets of 4, 8, 16 and 32:
  1. best-of-N independent proposals from the baseline;
  2. deterministic first-improvement local search (the same operator, temperature 0);
  3. **only if (2) shows headroom:** Metropolis acceptance with T₀ derived from the pilot's measured Δ distribution. Cooling is driven by evaluation count, the temperature has a positive floor, and the best state is tracked from the first evaluation. Keep annealing only if it produces more *finalized* wins than (2) at equal total cost.
- **Chains** run sequentially on the request's engine. Default 1. The winner is chosen by (oracle cost, chain index).
- **Parallel chains** are out of scope (non-goal). No existing code routes in parallel within a request, the server assumes about one core per request, and every chain would need its own engine and cold `NodesCache`.

---

## 6. Milestones

Each milestone lists its deliverables and its **acceptance criteria**. Milestones marked **Checkpoint** end with a hand-back to the owner. Do not continue past one without approval.

### M0: Foundations (Checkpoint)

There is no search in M0. Each step may stop the project with a recorded reason.

**Verify first:** some integration facts in §2.1 and §4.5 were confirmed by only one review. Check them before building on them, and record what you find in `docs/refine-notes.md`:
- that WAYPOINT writes its output before the gate (`RE:393`);
- how `request.lastResult` and the local `quality` are used after the hook;
- that `finalizeAdoptedRoundTripTrack` writes matched waypoints into engine state (`RT/RoundTripTrackCleanup.java:102`);
- that the outer catch clears the route (`RT/RoundTripOrchestrator.java:620`).

| # | Deliverable | Acceptance |
|---|---|---|
| 0.1 | Path walker + `LoopCostOracle` | Walking a raw leg's nodes equals its routed cost, exactly, on ≥ 40 fixture legs (the §2.9 method is the reference). Unit tests pass for: segmentation invariance (the same ride as 2 vs 5 legs gives the same price, within float tolerance); closed loops; turn costs; elevation hysteresis; clipped endpoints; detailed-to-junction reduction. It prices ≥ 30 shipped loops, and the report compares oracle vs `track.cost/distance`. **If the walker can't be built,** fall back to per-leg pricing (§4.3) and record it in ADR-0004. **Stop** only if neither works. |
| 0.2 | Geometry fidelity | Tests for: mid-edge vias, same-edge endpoints, opposite arrival/departure, curved edges, removed-via spurs. Skeleton recovery after cleanup is validated on GREEDY and AUTO winners. A report of raw vs detailed distance, crossings and reuse. |
| 0.3 | `RefineFinalizer` + `publish()` + deadline contract | Failure-injection tests after each finalization step leave the route, waypoints, verdict and serialized output byte-identical, and still publish diagnostics. An ownership test: finalizing A can't change B, the cache, or the baseline. Deadline tests: a timeout in retrack or pricing yields a *timeout* outcome (not a raw fallback) and sets `refineTruncated`; cancellation propagates. |
| 0.4 | Initialization | Re-routes the n raw legs. Report the cost in ms and links, and how much the rebuilt loop differs from the baseline. |
| 0.5 | One replacement end-to-end | Dreieich fixture, a single MOVE, through finalizer, oracle, predicate and publish. Tests cover the accept path, and the reject path with the baseline byte-identical. |
| 0.6 | Full-stage cost model | About 30 stratified gravel cells: open, town, coastal, hilly; 30 and 100 km; GREEDY and post-AUTO. `-Dloop.forks=1`. Measure `baseline pricing + init + k × evaluation + finalization + publish`, in ms and links. |
| 0.7 | Docs | `docs/refine-notes.md`, ADR-0003 (placement), ADR-0004 (energy and ship predicate), and the radius bound from the measurements. |
| — | Refine off | `GreedyPlannerParityTest` passes unchanged; `LoopGoldenSignatureTest` passes locally unchanged; PMD and checkstyle are clean. |

**Diagnostic runs:** M0 may run explicitly labelled measurement runs beyond 3 s, so that over-budget initialization and finalization are measured in full rather than cut off.

**Checkpoint:** hand back the measurements.
- If init plus one finalization exceeds 3 s p90, but the QUALITY-only 8 s variant (D5) is still plausible, present that option. The owner's approval authorizes an M1 pilot of it.
- If neither fits, **stop** and propose a rejection ADR.

### M1: Pilot experiment (Checkpoint)

The smallest experiment that answers "does searching help, and at what cost?".

- **Harness first:** before the pilot, extend `LoopQualityResult`/`LoopQualityReport` to persist request ms, gate verdict, RCS, gate cost/m, oracle cost/m and the refine diagnostics (§7). Forward `-Dloop.refine*` in `brouter-core/build.gradle:95-114`. M3 reuses this unchanged.

- **Setup:** MOVE only, final via frozen, fixed via count, one chain, one starting variant. Evaluation budgets of 4, 8, 16 and 32 (§5 counting). Strategies as in §5.
- **Cells:** the M0.6 cells, plus fastbike correctness cases.
- **Measurement:** within the same request, at the real hook (no replay).
  - Per cell: oracle cost/m before and after, the predicate outcome and reason, added ms, proposals, routed legs, finalizations, and truncation.
  - Fidelity metrics as in §4.3.
  - Win rate over **all** cells (including skips, failures and truncations) and over eligible cells.
- **Traces** for 6 cells, including at least one failure case: a per-evaluation CSV (evaluation, operator, energy, accepted, feasible) and GeoJSON snapshots of the best candidate.

**Acceptance:**
- The never-worse property holds in 100 % of cells.
- Refine-off parity is unchanged.
- The determinism test passes from a fixed start state, with cache hits and misses.
- The results table plots quality against total cost for each strategy and budget.

**Checkpoint decision:**
- **Quality statistic** (the same definition in M1 and M3): the **median per-cell relative improvement in oracle cost/m** over the frozen gravel cell list.
  - Skips, failures, truncations and unpriceable baselines count as 0 % improvement and stay in the denominator.
  - Fastbike is reported separately and never enters this statistic.
- **Go:** at some budget within D5, the quality statistic is ≥ 2 %, with the never-worse property intact → M2. A value between 1 % and 2 % is a borderline result: hand back with the numbers and don't choose yourself.
- **Expand:** traces show MOVE can't reach the bad part of the loop → M4 operators first, then M1 again.
- **Reject:** ADR-0005 with the failure mode → M5.

### M2: Productise (only after Go)

- **Request params**, all clamped, with docs rows in `docs/features/roundtrips.md:21-31`:

  | Parameter | Values | Default |
  |---|---|---|
  | `roundTripRefine` | `none` \| `local` \| `anneal` (the values M1 validated) | `none` |
  | `roundTripRefineEvals` | int, 1–64 | from M1 |
  | `roundTripRefineMaxMs` | int, 500–8000 | 3000 |

  `…Chains` and `…Seeds` are deferred to M4.
- **`refineAllowed`** in the effort policy, and child suppression.
- **Diagnostics publication** through one tier-independent path (§7).

**Acceptance:**
- Tests for parameter clamps, child suppression and every skip reason.
- Refine-off coverage for WAYPOINT, AUTO and explicit-via, plus both golden sets unchanged.
- Direction tests: east/west symmetry, 359°/1° wraparound, repeated moves, cleanup changing the farthest point, forced heading.
- PMD and checkstyle clean.

### M3: Full A/B (Checkpoint)

- **Harness:** reuse the M1 extension, and add a frozen manifest (commit, tiles, profiles and params, resolved directions, seeds, hardware, fork count).
- **Runs:** gravel 230 and fastbike 230 cells, refine off vs on, paired within the request. A `directions=none` sweep is optional.

**Acceptance, i.e. the bars** (gravel; fastbike is reported without bars):

| Metric | Bar |
|---|---|
| Quality statistic (M1 definition), gravel | ≥ 2 % |
| Absolute length error | not worse |
| Self-crossings | not worse |
| RCS median | not worse |
| Gate rejections | 0 new |
| Cells worse under the ship predicate | 0 |
| Paired added latency p90 | ≤ 3 s (or ≤ 8 s for the QUALITY-only variant) |
| Truncation rate | ≤ 5 % |

Also report: end-to-end p90, win rate (over all cells and over eligible cells), mean relative gain, proposals, routed legs, finalizations, and first/last-15 % residential and track share (evaluation only).

### M4: Extensions (each separately gated)

Each extension needs its own paired A/B at equal total cost, at most two tuning rounds, and an ADR note with its numbers:
- INSERT (never on the closing leg), and DELETE together with final-via moves (D3). Segmentation invariance is guaranteed by the oracle.
- WORST_LEG (§4.4).
- Extra starting variants (`roundTripRefineSeeds`, the *total* number of variants, all counted in the latency budget).
- A dense skeleton: many short legs for cheaper evaluations.
- Documented geometry terms in the energy (ADR-0004 amendment).

### M5: Decision

- ADR-0005 records shipped or rejected, with the numbers.
- Add the §4.1 terms to `CONTEXT.md`.
- Refinement stays **off by default** unless the owner flips it.
- Fix the stale references found along the way:
  - `RoundTripVarietySeedSentinelTest` cites the variety-seed rule as "ADR-0001";
  - `docs/developers/testing_roundtrips.md:36` gives "~930 cells";
  - `RoadCharacterScore.java:17-18` says RCS is "blind to road character".

---

## 7. Diagnostics

`RefineDiagnostics` is published for every tier through one path: on `RoundTripResult` when one exists, otherwise through the request's final publication. It also goes into the harness JSON. Fields:
- `refineApplied`, and the reason (skip reason or predicate failure);
- oracle cost/m before and after;
- RCS before and after;
- proposals, invalid proposals, evaluations, legs routed, cache hit rate, finalizations;
- chains, `refineTruncated` (and the operation that timed out), elapsed ms.

---

## 8. Risks (in order)

1. **Exact pricing needs new engine code.** The spike (§2.9) proved exact per-leg pricing, and proved that continuous pricing of arbitrary sequences needs a new path walker. Without an exact oracle for both sides, comparator noise (−1.1 % to +2.1 %) is as large as the bar. M0.1 builds the walker first; per-leg pricing is the fallback.
2. **The full stage is too expensive.** Initialization (n legs) plus finalization may use up the budget before any search runs (M0.6).
   - The planning estimate of 100–300 ms per leg (5–15 evaluations in 3 s) is a hypothesis.
   - The ~40× detail-retrack figure in `GreedyRoundTripPlanner.java:1432-1440` comes from an earlier experiment, not a benchmark.
3. **Raw and finished loops diverge.** Raw winners may not survive detail retrack and cleanup (M1 fidelity).
4. **Operator reach.** MOVE with a frozen final via may not be able to fix the loops that matter (ADR-0002's finding).
5. **Cold cache after AUTO.** The parent engine hasn't routed, so the first evaluations pay for tile decoding. It is included in M0.6.

---

## 9. Starting defaults (all in `RefineConfig`)

M0 and M1 may tune **search** parameters within the existing contracts. Changing ε, δ, eligibility, an invariant, or a §3 decision needs owner approval (§0.4).

| Group | Parameter | Default |
|---|---|---|
| Budget | evaluations / max proposals / chains / max ms | from M1 (pilot: 4, 8, 16, 32) / 4 × evaluations / 1 / 3000 |
| Finalization | top-k finalists | 3 |
| Ship predicate | cost margin ε / RCS guard δ | 0.5 % / 0.02 |
| MOVE | σ / displacement cap from original | 0.5 × mean via spacing, min 150 m / 0.25 × searchRadius |
| Constraints | adjacent via spacing / direction tolerance / degenerate-direction radius | 300 m / 15° / 500 m |
| Constraints | radius bound | from M0 |
| Annealing (only if M1 keeps it) | T₀ / temperature floor / cooling | from pilot Δ distribution / > 0 / evaluation-driven |

---

## Appendix A: Changes from Draft 0.2, and why

| 0.2 said | Code shows | 0.3 does |
|---|---|---|
| PR #44 is on `master` | PR #44 is open; `master` lacks the ADRs and planner changes | Pin `eec40289` (D4) |
| Objective = `RouteChoiceScore`, "already follows ADR-0001" | RCS weights tags at 0.17; ADR-0001 rejects it as a planning target by name | Energy = oracle cost/m plus constraints (D1); RCS only as a finished-loop guard (D2) |
| One "objective" for both search and shipping | A composite energy improving doesn't prove "never worse on cost" | Separate energy and ship predicate (§4.3, §4.5) |
| Cost/m from track cost | Cost is path-state dependent; cleanup leaves `track.cost` stale; re-routed legs differ from shipped cost by −1.1 % to +2.1 % (spike) | Cost oracle with a new path walker (§4.3, §2.9) |
| Leg cache keyed by (from node, to node, profile) | Legs are selected with a `refTrack` penalty; a matched waypoint is edge + crosspoint | Legs routed without `refTrack`; key = edge + crosspoint; successes only |
| ±5 % "Q16" gate | The gate band is 0.5–1.8; ±5 % is greedy's tolerance | Length rule in §4.5 |
| 300 evals × 4 parallel chains in 3 s | ≈ 2,280 leg searches, ~5 ms each; no in-request parallel routing; engine not thread-safe | Pilot budgets 4–32, sequential chains, measure first |
| Determinism across threads and load | Tiers already depend on the wall clock; `Math.random()` picks the default direction | Scoped determinism (§0.3 item 4) |
| Direction via a centroid-of-vias bearing, reusing the jitter bound | No such bound exists; `computeDirectionDelta` is absolute; a centroid moves on INSERT | Wrapped farthest-point bearing, ±15° (§4.4) |
| `WORST_LEG` relative to κ | κ is per air-metre and exists only for ISO_GREEDY | Relative to the loop's median leg; deferred to M4 |
| Via count 3…⌈km/2⌉, radius 0.6 × L | Allows 50 vias on 100 km; 0.6 L is loose | Fixed via count in the pilot; radius bound measured |
| INSERT on any leg | INSERT on the closing leg = an ADR-0002 dead end | Never on the closing leg; deferred to M4 |
| Refine any tier's loop | No skeleton/result for WAYPOINT; AUTO parent has no planner result; package-private internals | Scope and skip reasons (§4.6, D5) |
| 30 T₀ samples, early stop after 150 | Larger than the realistic budget | Annealing only if the pilot shows headroom (§5) |
| `SplittableRandom` | API 24+, and Android's minSdk is 23 | Splitmix hashing |
| Matrices of 230 / 460 cells | Gravel 230, fastbike 230, mtb 88 | Corrected (§2.6) |
| Harness measures p90 and gate rejections | Neither is persisted | Harness extension at the start of M1 |
