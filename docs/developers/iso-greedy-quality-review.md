# Round-trip quality review: v1 (PR #903) vs. current master

Investigation into the observation that generated loops feel worse than the
first ISO_GREEDY version, while latency improved markedly.

**Baseline compared:** `e2f07ac` (merge of PR #903, "v1" below) → `dd2be35`
(current master). 161 files, +20396 / −11505; 16 commits touch
`btools/router` after the upstream revert.

**Method.** Rename-aware diff of every round-trip source file; a name-and-value
diff over every `static final` numeric constant in `btools.router`; comment-
stripped code diffs of each moved class; a read of the repo's own recorded
evidence (golden signatures, parity goldens, developer docs); and live
measurement — the offline fixture suite runs, so the numbers below marked
*measured* were produced by running the current code, not inferred.

**What could not be run.** The real-map loop-quality matrix needs `segments4`
tiles that are not available offline, so no production-scale A/B was possible.
Fixture numbers show the *direction* of an effect, not its production
magnitude.

---

## Executive summary

The ISO_GREEDY algorithm itself is essentially unchanged and, in a few places,
improved. Every scorer weight, the routed top-K, the source-fairness quota, the
pool filters, and all twelve quality-gate thresholds are numerically identical
to v1. A dedicated parity test pins the planner's full observable output
bit-for-bit against the pre-refactor baseline, and it passes at HEAD.

The quality change comes from three things around it, in descending order of
expected impact:

1. **A request that names no algorithm no longer reaches ISO_GREEDY at all** —
   the shipped default moved from AUTO to FAST.
2. **ISO_GREEDY increasingly resolves to plain GREEDY**, via a new one-way
   pool-health demotion whose bunching signal fires on loop geometry alone, and
   via two new conditions that suppress the plain-GREEDY competitor.
3. **Nothing in CI would have caught either** — the loop-quality suite was
   scoped out of CI, and a real placement bug (every FAST loop 25–40 % short)
   shipped and survived several commits before being found by hand.

---

## 1. The shipped default is FAST, not AUTO

Deliberate, and documented in `26d699b`: *"The #903 revert was driven by
on-device latency… a request that names no algorithm gets FAST."*

```java
// v1 — RoutingContext.java:263
public RoundTripAlgorithm roundTripAlgorithm = RoundTripAlgorithm.AUTO;

// master — RoutingContext.java:296
public RoundTripAlgorithm roundTripAlgorithm = RoundTripAlgorithm.defaultAlgorithm();
//                                             -> getProperty("roundtrip.default.algorithm", "FAST")
```

Nothing else in core or server writes that field — the only other writer is an
explicit `roundTripAlgorithm=` request parameter (`RoutingParamCollector:261`).
*Measured:* a fresh `RoutingContext` reports `WAYPOINT`.

### 1a. What that costs, measured

Same fixture, same request, only the tier changed (Dreieich, gravel, r=1000 m,
target loop 6283 m):

| direction | tier | length | cost/m | vs. GREEDY |
|---|---|---:|---:|---|
| 0 | **WAYPOINT (default)** | 4265 m | **8.281** | 2.6× the cost per metre |
| 0 | GREEDY | 5545 m | 3.138 | |
| 0 | ISO_GREEDY / AUTO | 5545 m | 3.127 | |
| 90 | **WAYPOINT (default)** | — | — | **no track: gate-rejected, ratio 0.25** |
| 90 | GREEDY | 5998 m | 3.034 | |
| 90 | ISO_GREEDY / AUTO | 6005 m | 3.149 | |
| 180 | **WAYPOINT (default)** | 4148 m | **7.408** | 2.5× |
| 180 | GREEDY / ISO_GREEDY / AUTO | 6242 m | 3.000 | |
| 270 | **WAYPOINT (default)** | 6362 m | **4.196** | 1.4× |
| 270 | GREEDY / ISO_GREEDY / AUTO | 6242 m | 3.000 | |

The fixture is a tiny synthetic map, so do not read the magnitudes as
production numbers. The pattern — the default tier riding markedly more
expensive ground, landing short, and failing outright in one of four
directions — is what a user comparing against v1's AUTO default would see.

### 1b. FAST is also a different loop *shape* now

FAST's primary placement is `FastWaypointPlanner.directionalLobeBearings` —
four vias fanned across a ~108° forward arc — and the encircling ring is only
a fallback (`FastStrategy:128`, *"it never encircles the start as the primary
shape"*). v1's WAYPOINT instead spread vias across the probed viable
directions (`selectSpreadDirections` + `sortDirectionsForLoop`).
`directionalLobeBearings` does not exist in v1.

So the default now yields a directional lobe where v1's default yielded a
routed circuit. The user-facing doc says so plainly: *"it is a lobe pointing
the way you asked, not a circle around the start point."*

### 1c. FAST's own measured accuracy

From `docs/features/roundtrips.md`, after the corrective pass was added:
mean length miss **14 %** (down from 19 %), with **~170 of 588** matrix cells
still shorter than 0.85× the ask. FAST also deliberately does **not** spend its
shape retry on a pure length miss (`isDistanceBandRejection`,
`FastStrategy:239`) — length is treated as the user's knob, shape as FAST's
contract.

### 1d. The mitigation works

*Measured:* `-Droundtrip.default.algorithm=AUTO` yields `AUTO`; `QUALITY`
yields `QUALITY`; an unparseable value falls back to `AUTO`. Per request,
`roundTripAlgorithm=AUTO` always wins over the property.

**Check this before reading anything below.** If the loops being compared did
not pass `roundTripAlgorithm`, the comparison is "ISO_GREEDY v1 vs. FAST", not
"ISO_GREEDY v1 vs. ISO_GREEDY v2".

---

## 2. ISO_GREEDY is collapsing into plain GREEDY

The repo records this in two independent places.

### 2a. Real-map golden signatures

`brouter-core/src/test/resources/test-data/golden/loop-signatures.txt`,
regenerated in `ec06485`. Target loop 50 265 m (r = 8000). Start coordinates
for these four cells did **not** move between versions (`ilonFor` only
overrides `mtb`, and these cells are fastbike/gravel).

| cell | length error v1 → new | cost/m v1 → new |
|---|---|---|
| `iso_greedy dreieich dir0 fastbike` | 8.5 % → 8.3 % | 1.680 → **1.878 (+11.8 %)** |
| `iso_greedy dreieich dir90 fastbike` | 17.1 % → 1.0 % | 1.846 → 1.589 (−13.9 %) |
| `iso_greedy rural_lozere gravel` | 4.4 % → 4.4 % | 2.998 → 2.998 (=) |
| `iso_greedy urban_berlin fastbike` | 6.3 % → 2.8 % | 1.459 → 1.554 (+6.5 %) |
| `greedy dreieich dir0 fastbike` | 8.5 % → **13.6 %** | 1.680 → 1.787 (+6.4 %) |
| `greedy dreieich dir90 fastbike` | 3.0 % → 1.0 % | 1.625 → 1.589 (−2.2 %) |
| `greedy rural_lozere gravel` | 2.9 % → **9.2 %** | 2.594 → **3.147 (+21.3 %)** |
| `greedy urban_berlin fastbike` | 8.1 % → 2.8 % | 1.538 → 1.554 (+1.1 %) |

Length accuracy improved on 3 cells and worsened on 2; **cost per metre
worsened on 5 of 8**. The two clearest single-cell regressions are
`iso_greedy dreieich dir0` (identical length, 11.8 % pricier ground) and
`greedy rural_lozere` (length error tripled *and* 21 % pricier, with *less*
climbing — so not an elevation trade).

And the structural signal — identical hash means ISO_GREEDY produced
byte-identical output to plain GREEDY:

| cell | v1 | master |
|---|---|---|
| dreieich dir0 | same | differ |
| dreieich dir90 | differ | **same — newly collapsed** |
| rural_lozere | differ | differ |
| urban_berlin | differ | **same — newly collapsed** |

v1: 1 of 4 cells identical. Master: 2 of 4.

The authors noticed: `ec06485` had to **add** a `blend_only` golden cell,
because *"the plain dir90 cell's golden is byte-identical to greedy's because
the internal graph-native branch wins there — without this cell no golden
covers the blended planner's dir90 output at all."*

**Caveat.** The COASTAL_NICE threshold comment in `LoopTestRegion` records that
OSM tile data moved between the two captures (*"~35.9 % reuse, up from ~30 % on
older tile data"*), and `ensureRegionPinned` was added afterwards precisely to
stop that. So these deltas are code **plus** possible data drift. The hash
collapses are not explainable by data drift — greedy and iso_greedy were
captured on the same tiles in the same run.

### 2b. Where the accepted legs actually come from

`GreedyPlannerParityTest` fingerprints the planner's telemetry against goldens
captured on the pre-refactor baseline. It runs offline and **passes at HEAD**,
so these are current numbers, and my own probe reproduces them:

| ISO_GREEDY cell | routed iso / non-iso | **accepted iso legs** | accepted non-iso | pool health |
|---|---|---|---|---|
| gravel dir0 | 2 / 11 | **1** | 2 | 0.74 |
| gravel dir90 | 2 / 11 | **0** | 3 | 0.69 |
| gravel dir180 | 2 / 6 | **0** | 2 | 0.75 |
| gravel dir270 | 1 / 7 | **0** | 2 | 0.75 |
| trekking dir270 | 3 / 10 | 1 | 2 | 0.81 |

Across these plans the isochrone pool supplies **1 of 10 committed legs**, and
three of four gravel directions commit **zero** iso legs. Three of the four
gravel directions produce a track byte-identical to plain GREEDY's.

These are 6.3 km loops on a synthetic fixture where the pool is legitimately
thin — this is not proof of production behaviour on its own. It is the same
signature the real-map goldens show.

---

## 3. `IsoPoolHealth`: a new, one-way demotion with a geometric false positive

`IsoPoolHealth` (271 lines) does not exist in v1; it was added in `ec06485`. It
scores the start-centred pool in `[0, 1]`, monotonically decreasing, with two
sticky effects:

- **< 0.55 DEGRADED** — iso candidates lose their prior-based scoring terms and
  the graph-native routed quota gains a seat.
- **< 0.30 UNHEALTHY** — the planner goes graph-native-only for the rest of the
  plan, i.e. **ISO_GREEDY becomes plain GREEDY mid-plan**.

There is no path back up. A statically DEGRADED pool starts the plan already
demoted (`selectIsoStartPolicy`: *"it engages from step 1 because the static
deduction is already in the score"*). Every firing is a move away from v1's
behaviour, so any miscalibration here is a one-way loss relative to v1.

### 3a. The sector-bunching signal cannot be satisfied

```java
/** Per accepted via landing in an already-used angular sector (bunching). */
static final double W_SECTOR_REPEAT = 0.08;      // cap 0.16
static final int ACCEPTED_SECTOR_COUNT = 8;      // 45° sectors
```

with the premise: *"a healthy loop visits a fresh sector nearly every step."*

The bearing is measured **from the loop start**
(`GreedyRoundTripPlanner:780`, `start.ilon, start.ilat`) — and the start lies
**on** the loop, not at its centre. For any convex closed curve through the
start, every other point lies on one side of the tangent at the start, so
bearings span at most 180°.

*Measured* by driving the shipped `IsoPoolHealth` with bearings from an ideal
circular loop through the start:

```
sectors any via can occupy: [0, 1, 6, 7]  ->  4 of 8
```

Half the sector space is structurally unreachable. Consequently:

| vias | distinct sectors | forced repeats | score after geometry alone |
|---|---|---|---|
| 3 | 3 | 0 | 1.00 |
| 4 | 4 | 0 | 1.00 |
| 5 | 4 | 1 | 0.92 |
| 6 | 4 | **2 (the cap)** | **0.84** |

Identical for an ellipse; worse for a teardrop; unchanged under 15 % jitter.
`selectGreedySubRouteCount` returns 6 for loops ≥ 80 km — and ≥ 30 km on mtb
(`n++`) — and 6 is also the ladder ceiling (`addUniqueCount` clamps to
`[3, 6]`), so **the 6-via case is the realistic worst case, not an outlier.**

What that costs, *measured* on a pool with perfect static shape:

| vias | → DEGRADED after | → UNHEALTHY (becomes plain GREEDY) after |
|---|---|---|
| 4 | 3 graph-native wins | not within 6 |
| 5 | 2 graph-native wins | not within 6 |
| **6** | **2 graph-native wins** | **3 quota-injected wins** |

The loop's own geometry spends one demotion step's worth of headroom before any
real evidence is collected.

### 3b. The win signal penalises the source quota for working

`W_GRAPH_NATIVE_WIN = 0.16` per mixed-source routed comparison won by a
graph-native candidate (cap 0.48 = three wins). But the source quota exists to
guarantee that candidate a routed seat so it *can* win — its own rationale in
`GreedyRoundTripPlanner`:

> "Reserving a seat changes nothing when the honest pick deserves to lose;
> phase-2 still judges on routed truth."

Since the quota fills a graph-native seat in essentially every step, nearly
every step is a mixed-source comparison. The same event is read two ways: as
the quota working as designed, and as 0.16 of evidence that the pool is stale.
*Measured* on the fixture, a single win already puts a perfect-shape pool at
0.74 — 36 % of the way to DEGRADED. On a 5–6 step plan three such wins are
unremarkable, and three is the cap.

### 3c. Minor: the health model uses the unscaled bearing

`recordAcceptedLegBearing` is fed `CheapAngleMeter.getDirection`, which uses raw
integer lon/lat diffs with no latitude scaling, while the planner's own geometry
moved to `CheapRuler.getScaledBearing` in this same refactor (10 call sites).
Elsewhere `getDirection` compares *local* angles where the distortion cancels;
here it is a tens-of-kilometres bearing bucketed into *absolute* 45° sectors.
At 47° N the longitude component is overweighted by `1/cos(lat) ≈ 1.47`. It
does not change the table in 3a (the 180° bound dominates) but makes sector
assignment latitude-dependent for no reason.

---

## 4. The AUTO competition got narrower

v1 consulted plain GREEDY on a single condition:

```java
boolean isoGreedyWeak = !isoGreedyR.accepted()
  || isoGreedyR.scoreValue() < CLEAR_ACCEPT_THRESHOLD;   // 0.85, unchanged
```

Master adds two more discard reasons (`autoPlainGreedyDiscardReason`):

```java
if (isoGreedyR.internalGraphNativeCompared()) return "ISO_GREEDY already compared graph-native branch";
if (isoGreedyAbsorbedGraphNativeTruth(isoGreedyR)) return "ISO_GREEDY absorbed graph-native truth";
```

So a **weak** ISO_GREEDY result — one v1 always raced — now skips the
competitor whenever ISO_GREEDY ran its internal graph-native branch.
`RoundTripAlgorithm`'s javadoc puts the stake at *"greedy wins ~a quarter of
the competition cells."*

The internal branch is not equivalent: it runs with `src.effectiveDirection`,
the Phase 2.0 iso-asymmetry-biased bearing that only exists for ISO_GREEDY,
whereas a real plain-GREEDY child starts unbiased. It is a graph-native ladder
anchored to the iso pool's own asymmetry — correlated with the candidate it is
meant to check.

v1 also ran the GREEDY child **speculatively in parallel** (permit-gated
thread, killed when ISO turned out strong), so consulting it was nearly free in
wall-clock terms. Master runs children sequentially (`5141eda`, `f21b3e3`),
putting the competitor on the latency path.

**The repo confirms the consequence.** From `roundtrip-strategies.md`:

> **Weak-winner second opinion** (2026-07-25): a winner below the 0.85
> clear-accept bar earns one cross-family WAYPOINT challenger — the
> greedy-family pools are usually singletons there (**absorption skip**, or
> greedy hard-failing on mtb), **so a shape defect ships unopposed** while the
> FAST tier's cascade produces a better loop nobody compares.

That is the absorption skip causing defective loops to ship. The mitigation
added was a WAYPOINT challenger (bounded to 1.25× of the incumbent's cost/m),
not restoring the GREEDY competitor.

### 4a. The chain

1. Geometry alone lowers the pool-health bar (3a).
2. Two or three graph-native wins (3b) → DEGRADED or UNHEALTHY → a hybrid plan
   whose early legs came from the iso pool and later legs did not.
3. That hybrid sets `internalGraphNativeCompared` / `graphNativeOnlyStart` —
   exactly what suppresses the plain-GREEDY child (4).

The plan the health model just declared untrustworthy also suppresses the
competitor that would have replaced it.

---

## 5. No CI guard, and a real bug got through

`8941cc1` scoped CI to the offline Mapterhorn suites:

```diff
-      run: ./gradlew build integrationTest
+      run: ./gradlew build :brouter-map-creator:integrationTest :brouter-server:integrationTest
```

`brouter-core:integrationTest` — the loop-quality matrix, the gold-standard
suite, the golden signatures, the mtb sweep — is not in that list. The commit
is explicit: *"The round-trip suite stays a local/on-demand gate."* The
reasoning is sound (it downloads hundreds of MB and asserts dev-box latency),
but the effect is that **16 round-trip commits landed after `ec06485` with no
automated loop-quality verification**, and the golden signature file has not
moved since.

What that gap costs is not hypothetical. `8996fef` fixed a placement bug
introduced by the modularization:

> the fan… sorts into a zig-zag order whose perimeter overestimates the real
> routed skeleton by ~30 %. The scale came out 0.73 instead of ~0.95, shrinking
> every FAST loop up front — the systematic undershoot vs the old routine.
>
> Basel 50 km: 29.6 km (59 %) → 42.2 km (84 %)
> Basel 100 km: 77.4 km (77 %) → 104.6 km (105 %)
> Dreieich 20 km: 15.3 km (77 %) → 19.5 km (98 %)

Every FAST loop came back 25–40 % short, in the tier that is the shipped
default, and it was found by hand rather than by a test. If the comparison that
prompted this review ran between `ec06485` and `8996fef`, that bug alone
explains it — and it is fixed now.

---

## 6. Smaller deltas worth knowing

- **AUTO can silently resolve to BOUNDED.** `RoundTripEffortPolicy.resolveAuto`
  drops to top-K 2/3, an 8 s tier budget and `skipRetryLayers` when
  `maxRunningTime ≤ 10 s` or `memoryclass ≤ 48`. The server ceiling is 60 s so
  this is inert by default, but a client passing `timeout=10`, or an Android
  device reporting a small memory class (`BRouterView:557`), asks for AUTO and
  gets bounded effort.
- **mtb loops are no longer ranked on road cost.** `costMWeight` returns 0 for
  mtb, so the positive weights sum to 0.95 there. Deliberate and evidence-backed
  (the old `[4, 9]` band already scored 55 of 64 mtb cells at zero), but it means
  cost/m no longer participates in mtb candidate selection at all.
- **Two request parameters disappeared.** `roundTripIsochrone` (an alias for
  `roundTripAlgorithm=ISOCHRONE`, still documented in
  `roundtrip-strategies.md:59` — stale) and `roundTripDensify`.
- **Explicit-via densification was removed outright** — parameter, the five
  `explicitViaDensify*` context fields, and the implementation. It went with the
  upstream revert (`84c4a47`) and was never re-added, so explicit-via loops no
  longer bulge the arcs between user vias to honour the requested length.
- **Crossing counting changed for the better.** The 10 000-node decimation
  (`MAX_SHAPE_SCAN_NODES`, `CROSS_SCAN_MAX_NODES`) is gone; `LoopAnalysis` runs
  the gate's own primitives at full resolution with an early-exit ceiling of 64.
  v1's own comment noted decimation *fabricated* crossings on switchbacks
  (gate = 21 where the true count was 0).

---

## 7. What is *not* the cause

Checked and found identical, or improved:

- **Every `CandidateScorer` weight** (`1.0, 2.0, 0.5, 3.0, 1.5, 1.5`) and the
  hostility-active policy.
- **Planner routed top-K** — `MAX_ROUTE_ATTEMPTS` 3 / `_LATE` 5; the effort
  policy's `STANDARD_PRESET` reproduces v1 exactly.
- **`GRAPH_NATIVE_MIN_ROUTED`** 1 / `_LATE` 2 source-fairness quota.
- **All twelve quality-gate thresholds** — distance ratio `[0.5, 1.8]`, closure
  400 m, `MAX_SELF_INTERSECTIONS` 5, hairpins 20, hostile fractions, contiguous
  hostile 1500 m, both cost-factor thresholds. Verified by value.
- **All eight `RouteChoiceScore` weights** and `CLEAR_ACCEPT_THRESHOLD` 0.85.
- **`ReuseClassifier` and `RoadCharacterScore`** — visibility-only changes.
- **Isochrone pool filters** — `MIN_AIR_DIST_FRAC`, `POOL_CAP`,
  `DEDUPE_GRANULARITY`, `MIN_DISTINCT_BUCKETS` 4, `MIN_ANGULAR_SPAN_DEG` 180.
- **`RoundTripTrackCleanup` and `WaypointSnapper`** — all 14 public entry points
  are extractions of methods that existed in v1's `RoutingEngine`.
- **The finalization sequence** — disclosures, shipped-crossings note, spur
  note, residual-chord note, >1.5 overshoot warning, `ensureInfoMessage`, in the
  same order, now with a try/catch so a cosmetic advisory cannot destroy a
  rideable track.

Improved:

- **Coprime angular-stride bucket order** in `IsochroneCandidateProvider` —
  fixes v1 silently dropping a contiguous 120° wedge opposite the start
  whenever the pool cap bound.
- **`CheapRuler.getScaledBearing`** replaces the latitude-distorting bearing in
  `loopSweepPenalty`.
- **Three new shape penalties** in the judge — clover/petal (0.12), far-field
  dwell (0.10), crumple (0.12) — plus compactness now measured on **net enclosed
  area** (signed shoelace), which catches out-and-back "threads" that convex-hull
  compactness reads as fat. Teardrops near the start now cost **2×**. All
  ranking-only; the judge got stricter on shape, not looser.

---

## 8. How to confirm which mechanism you are seeing

Every mechanism above is already instrumented. For one real request, grep the
log for:

| what it tells you | log line |
|---|---|
| which tier ran (§1) | `round trip algorithm:` |
| effort resolution / BOUNDED downgrade (§6) | `round trip effort:` |
| pool health, sector repeats, graph-native wins (§3) | `iso-pool health: score=… sectorRepeats=… graphWins=…` |
| mid-plan demotion (§3) | `step N: iso-pool influence reduced` |
| per-leg source (§2b) | `leg N source: iso-pool \| graph-native` |
| competitor skipped (§4) | `AUTO candidate: …`, `ISO_GREEDY absorbed graph-native truth` |

To A/B properly, run the same request three ways —
`roundTripAlgorithm=FAST`, `=AUTO`, `=QUALITY` — and compare length error and
cost/m. If FAST vs AUTO explains the gap, §1 is the whole story.

---

## 9. Recommendations

**Do first — no code change.**

1. Confirm the tier. If loops are being requested without
   `roundTripAlgorithm`, set `-Droundtrip.default.algorithm=AUTO` (verified
   working) or pass the parameter, and re-compare. This is the single largest
   lever and costs nothing to test.
2. Re-run `LoopGoldenSignatureTest` with `-Dgolden.tests=true` on pinned tiles.
   The goldens have not moved since `ec06485` despite 16 subsequent round-trip
   commits; that number should be believed only after it is re-verified.

**Then — calibration, needs measurement on the loop matrix, not a blind edit.**

3. The sector-bunching signal (§3a) is measuring loop geometry, not pool
   staleness. Either measure the via's bearing from the loop's **centroid**
   (or from the previous via) instead of from the start, or keep the signal but
   subtract the geometric floor `max(0, vias − 4)` before charging repeats.
4. Reconsider `W_GRAPH_NATIVE_WIN = 0.16` (§3b) given the quota guarantees the
   graph-native candidate a seat in nearly every step. One honest loss should
   not cost a third of the DEGRADED budget.
5. Reconsider the `internalGraphNativeCompared` / `absorbed graph-native truth`
   skips (§4). The repo already documented that they let shape defects ship
   unopposed; the WAYPOINT second opinion mitigates the symptom, but a
   direction-unbiased GREEDY child is what v1 actually ran.

**Process.**

6. The loop-quality suite is the only thing that would have caught the lobe
   radius bug, and it does not run anywhere automatically. Even a small pinned
   subset — the four golden cells plus the offline fixture parity test — on a
   nightly or pre-release job would close the gap that §5 documents.
