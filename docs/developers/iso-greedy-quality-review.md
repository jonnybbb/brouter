# ISO_GREEDY quality review: v1 (PR #903) vs. current master

Code-reading review triggered by the observation that loop quality feels worse
than the first ISO_GREEDY version (merged as PR #903 / `e2f07ac`, reverted
upstream by #945) while latency improved markedly.

Baseline compared: `e2f07ac` (v1) → `dd2be35` (current master).
Diff size: 77 files, +14440 / −10330 in `brouter-core/src/main`.

## Summary

The core ISO_GREEDY machinery survived the refactor intact. Every candidate
scorer weight, the planner's routed top-K (3 normal / 5 late), the window
bounds, the source-fairness quota, and the isochrone pool filters are numerically
identical to v1 — a name-and-value diff over every `static final` numeric
constant in `btools.router` finds no real change (only two identical constants
that moved between files, and `1.6`/`1.65` formatting noise). The angular-stride
change in `IsochroneCandidateProvider` is a fix, not a regression: v1's
adjacent-alternating bucket order filled the 24-candidate pool cap with a
contiguous band and silently dropped a 120° wedge opposite the start.

So the perceived quality loss is very unlikely to come from ISO_GREEDY's inner
loop. It comes from **what now runs instead of ISO_GREEDY, and from a new
mechanism that turns ISO_GREEDY into plain GREEDY mid-plan.** Findings are
ordered by expected impact.

---

## 1. The shipped default is no longer AUTO — it is FAST

This is almost certainly the dominant effect, and it is by design
(`26d699b`, "Round-trip: FAST is the default algorithm").

```java
// v1 — RoutingContext.java:263
public RoundTripAlgorithm roundTripAlgorithm = RoundTripAlgorithm.AUTO;

// master — RoutingContext.java:296
public RoundTripAlgorithm roundTripAlgorithm = RoundTripAlgorithm.defaultAlgorithm();
//                                             -> System.getProperty("roundtrip.default.algorithm", "FAST")
```

Nothing else in `brouter-server` or `brouter-core` assigns this field; the only
other writer is `RoutingParamCollector:261`, i.e. an explicit
`roundTripAlgorithm=` request parameter. So **a round-trip request that names no
algorithm now runs `WAYPOINT` — geometric placement with no routed-leg
evaluation — where v1 ran the full AUTO competition.**

That single change accounts for the whole symptom: much faster, visibly worse
loops, no ISO_GREEDY involved at all.

**Check this first.** If the loops being compared were produced without an
explicit `roundTripAlgorithm`, the comparison is not "ISO_GREEDY v1 vs.
ISO_GREEDY v2" — it is "ISO_GREEDY v1 vs. FAST". Re-run with
`roundTripAlgorithm=AUTO` (or `-Droundtrip.default.algorithm=AUTO` for a
deployment-wide comparison) before reading anything into findings 2–4.

---

## 2. AUTO now suppresses the plain-GREEDY competitor in two new cases

v1 consulted plain GREEDY whenever ISO_GREEDY was not clearly strong — a single
condition:

```java
// v1 — RoutingEngine.runAutoCandidateCompetition
boolean isoGreedyWeak = !isoGreedyR.accepted()
  || isoGreedyR.scoreValue() < CLEAR_ACCEPT_THRESHOLD;   // 0.85
boolean greedyNeeded = isoGreedyWeak && greedyEntitled;
```

Master adds two further discard reasons
(`AutoCompetitionStrategy.autoPlainGreedyDiscardReason`):

```java
if (isoGreedyR.scoreValue() >= CLEAR_ACCEPT_THRESHOLD) return "ISO_GREEDY strong";
if (isoGreedyR.internalGraphNativeCompared())           return "ISO_GREEDY already compared graph-native branch";
if (isoGreedyAbsorbedGraphNativeTruth(isoGreedyR))      return "ISO_GREEDY absorbed graph-native truth";
```

So a **weak** ISO_GREEDY result — one v1 would always have raced against plain
GREEDY — now skips that competitor whenever ISO_GREEDY happened to run its
internal graph-native branch. `RoundTripAlgorithm`'s own javadoc puts the stake
at "greedy wins ~a quarter of the competition cells".

The internal branch is not an equivalent substitute. It runs
(`GreedyStrategy.maybeRunInternalComparison`):

```java
runGreedyAttempt(request, start, searchRadius, desiredDistance,
                 src.effectiveDirection, ...)   // <- iso-asymmetry-BIASED bearing
```

`src.effectiveDirection` is the bearing the Phase 2.0 iso-asymmetry bias
selected, which only exists for ISO_GREEDY. A real plain-GREEDY child starts
from the unbiased direction. The internal branch is therefore a graph-native
ladder anchored to the iso pool's asymmetry bearing — correlated with the
candidate it is supposed to check, not independent of it.

Compounding this: v1 ran the GREEDY child *speculatively in parallel* (permit-gated
thread, killed when ISO_GREEDY turned out strong), so consulting it was nearly free
in wall-clock terms. Master runs children sequentially (`5141eda`, `f21b3e3`), which
puts the competitor squarely on the latency path and raises the incentive to skip it.

---

## 3. `IsoPoolHealth` is new, and it only ever demotes ISO_GREEDY

`IsoPoolHealth` (271 lines, added in `ec06485`) does not exist in v1. It scores
the start-centered pool in `[0, 1]`, monotonically decreasing, with two sticky
effects applied from the next candidate decision on:

- `< 0.55` **DEGRADED** — iso candidates lose their prior-based scoring terms and
  the graph-native routed-slot quota grows by one.
- `< 0.30` **UNHEALTHY** — the planner goes graph-native-only for the remaining
  steps, i.e. **ISO_GREEDY becomes plain GREEDY mid-plan.**

There is no counterpart that restores trust. Every firing is a move away from
v1's behaviour, so miscalibration here is a pure one-way quality loss relative
to v1.

### 3a. The sector-bunching signal is geometrically guaranteed to fire

```java
/** Per accepted via landing in an already-used angular sector (bunching). */
static final double W_SECTOR_REPEAT = 0.08;
static final double CAP_SECTOR_REPEAT = 0.16;
static final int ACCEPTED_SECTOR_COUNT = 8;          // 45° sectors
```

with the stated assumption:

> "Repeated acceptance in one 45° sector is the bunching signature — a healthy
> loop visits a fresh sector nearly every step."

That assumption does not hold for the geometry this planner produces. The
bearing is measured **from the loop start** (`GreedyRoundTripPlanner:780-781`,
`start.ilon, start.ilat`), and the start lies **on** the loop, not at its centre.
For any convex closed curve through the start, every other point of the curve
lies on one side of the tangent at the start — so the bearings span **at most
180°**, i.e. at most 5 of the 8 sectors, and in practice 4.

Simulating the shipped `sectorOf` over an ideal circular loop through the start
(`min(7, (int)(bearing/45))`), vias spaced evenly by arc length:

| vias | distinct sectors | sector repeats | deduction |
|-----:|-----------------:|---------------:|----------:|
| 4 | 4 | 0 | 0.00 |
| 5 | 4 | 1 | 0.08 |
| 6 | 4 | **2** | **0.16 (cap)** |
| 7 | 4 | 3 | 0.16 (cap) |

Identical for an ellipse; worse for a teardrop (2 distinct sectors, 4 repeats at
6 vias). Adding 15 % positional jitter over 200 samples does not change the means
(1.00 / 2.00 / 3.00 repeats at 5 / 6 / 7 vias). `distinct` saturates at 4 because
of the 180° bound, so repeats grow linearly with via count no matter how good the
loop is.

`planStep` runs `step = 1..subRouteCount`, and `selectGreedySubRouteCount`
returns 6 for loops ≥ 80 km — plus one for `mtb` profiles, so ≥ 30 km on MTB.
Six is also the ladder's ceiling (`addUniqueCount` clamps to `[3, 6]`), so the
6-via row is the realistic worst case, not an outlier. **Every such plan spends
the full `CAP_SECTOR_REPEAT` on loop geometry alone, before any real evidence.**

Effect on the demotion bar:

| geometric sector-repeats | → DEGRADED after | → UNHEALTHY after |
|---|---|---|
| 0 (short loops, ≤ 4 vias) | 3 graph-native wins | 4 quota-injected wins |
| 2 (6+ vias — the ≥ 80 km / MTB case) | **2 graph-native wins** | **3 quota-injected wins** |

### 3b. The graph-native-win signal penalises the source quota for working

`W_GRAPH_NATIVE_WIN = 0.16` per mixed-source routed comparison won by a
graph-native candidate (cap `0.48`, i.e. three wins). But the source quota exists
precisely to guarantee that candidate a routed seat so it *can* win — its own
rationale in `GreedyRoundTripPlanner`:

> "Reserving a seat changes nothing when the honest pick deserves to lose;
> phase-2 still judges on routed truth."

Since the quota guarantees a graph-native seat in essentially every step, nearly
every step is a mixed-source comparison. The same event is read two ways: as the
quota working as designed, and as 0.16 of evidence that the pool is stale. On a
5–6 step plan, three such wins are unremarkable — and they are the cap.

### 3c. The interaction is the part that bites

The three findings chain:

1. Sector-bunching fires on loop geometry (3a) → the DEGRADED/UNHEALTHY bar drops.
2. Two or three graph-native wins (3b) → UNHEALTHY → the planner goes
   graph-native-only for the remaining steps, producing a hybrid plan whose early
   legs came from the iso pool and whose later legs did not.
3. That hybrid sets `graphNativeOnlyStart` / `internalGraphNativeCompared`, which
   is exactly what makes AUTO **skip the real plain-GREEDY child** (finding 2).

So the plan the health model just declared untrustworthy also suppresses the
competitor that would have replaced it — and the surviving loop is one neither
v1's ISO_GREEDY nor v1's GREEDY would have produced.

### 3d. Minor: the health model uses the unscaled bearing

`recordAcceptedLegBearing` is fed `CheapAngleMeter.getDirection`, which uses raw
integer lon/lat diffs with no latitude scaling, while the planner's own geometry
moved to `CheapRuler.getScaledBearing` in this same refactor (10 call sites; see
the `loopSweepPenalty` change). Elsewhere `getDirection` is used for *local*
angle comparisons where the distortion cancels; here it is a tens-of-kilometres
bearing bucketed into *absolute* 45° sectors. At 47° N the longitude component is
overweighted by `1/cos(lat) ≈ 1.47`. It does not change the table in 3a (the 180°
bound dominates), but it makes sector assignment latitude-dependent for no reason.

---

## 4. AUTO can silently resolve to BOUNDED effort

`RoundTripEffortPolicy.resolveAuto` downgrades AUTO to `BOUNDED_PRESET`
(top-K 2/3 instead of 3/5, 8 s tier budget, `skipRetryLayers = true`) when:

```java
static final int  CONSTRAINED_MEMORYCLASS_MAX = 48;
static final long CONSTRAINED_BUDGET_MAX_MS   = 10_000;
```

The server's ceiling is 60 s, so this is inert for default server requests — but
a client passing `timeout=10` or lower, or an Android device reporting
`memoryclass ≤ 48` (`BRouterView:557`), gets bounded effort while having asked
for AUTO. Worth ruling out if the comparison ran on-device.

---

## What is *not* the cause

Checked and found identical or improved versus v1:

- All `CandidateScorer` weights (`1.0, 2.0, 0.5, 3.0, 1.5, 1.5`) and the
  hostility-active policy.
- Planner routed top-K at STANDARD effort (`MAX_ROUTE_ATTEMPTS` 3 /
  `..._LATE` 5) — the effort policy's `STANDARD_PRESET` reproduces v1 exactly.
- `GRAPH_NATIVE_MIN_ROUTED` 1 / `..._LATE` 2 source-fairness quota.
- Isochrone pool filters (`MIN_AIR_DIST_FRAC`, `POOL_CAP`, `DEDUPE_GRANULARITY`,
  `MIN_DISTINCT_BUCKETS` 4, `MIN_ANGULAR_SPAN_DEG` 180).
- `CLEAR_ACCEPT_THRESHOLD` 0.85.
- Step-window bounds, backoff factors, indirectness EMA.
- **Improved:** the coprime angular-stride bucket order, which fixes v1's
  dropped 120° wedge; and `CheapRuler.getScaledBearing` replacing the
  latitude-distorting bearing in `loopSweepPenalty`.
- **Improved:** self-intersection scanning no longer decimates via
  `MAX_SHAPE_SCAN_NODES` (which the old code itself noted fabricated crossings
  on switchbacks); it early-exits at `CROSSING_SCAN_CEILING = 64` instead.

---

## Suggested order of investigation

1. **Confirm what actually ran.** Re-run the comparison with an explicit
   `roundTripAlgorithm=AUTO`. If quality returns, finding 1 is the whole story
   and 2–4 are secondary.
2. **Read the logs.** Every mechanism above is already instrumented — grep a
   request's log for:
   - `round trip algorithm:` / `round trip effort:` (findings 1 and 4)
   - `iso-pool health: score=…, demotedAtStep=…` and
     `step N: iso-pool influence reduced` (finding 3)
   - `AUTO candidate: …` plus the discard reason (finding 2)
3. **If demotions show up in the logs**, the two cheapest calibration fixes are:
   - measure the sector-bunching signal against the loop's own centroid (or the
     via's bearing from the *previous* via) rather than from the start, which
     removes the 180° artefact; or leave the signal but require repeats beyond
     the geometric floor `max(0, vias − 4)`;
   - reconsider `W_GRAPH_NATIVE_WIN = 0.16` given that the quota guarantees the
     graph-native candidate a seat in nearly every step.

Both are calibration changes to a heuristic tuned against a loop-quality corpus,
so they should be measured on that matrix rather than applied blind.
