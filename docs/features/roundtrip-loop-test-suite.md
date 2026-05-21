# Round-trip loop test suite

Test coverage ensuring round-trip (loop) routing produces correct, high-quality
bike loops. Built in layers so correctness is enforced in CI while scale-dependent
quality is checked against real geography.

## Tiers

| Tier | Data | Runs in | What it checks |
|------|------|---------|----------------|
| **Invariants** | bundled `dreieich.pbf` fixture, regenerated to rd5 by the map-creator test (always version-consistent with `misc/profiles2`) | **CI** (no downloads) | structural correctness that must hold for *every* loop |
| **Quality (gated)** | downloaded v11 region tiles (`segments4`) | local / nightly, `-Dloop.tests=true` | distance-accuracy bands, road-reuse, shape, golden regressions across 5 real regions |

Full gated validation on v11 (`LoopQualityTest`, 240 cases × 3 algorithms): **0 failures**
(48 skipped where a direction/distance is infeasible). GREEDY is strong on real data
(composite 0.78–0.90, closure ~1 m). The committed golden metric baselines are byte-identical
under v11 routing — no regression.

The fixture rd5 is tiny (Dreieich only), so distance-accuracy bands are unreliable
there (the data runs out before a large requested loop can form). Scale-dependent
metrics therefore live in the gated tier; the fixture tier asserts only
terrain-independent structural properties.

## Running the suite

```bash
# CI tier — always runs, no data needed (part of the normal build):
./gradlew :brouter-core:test

# Gated tier — opt-in, needs the v11 region tiles in ./segments4:
#   E5_N50 E10_N50 E10_N45 E5_N40 E0_N40 (from brouter.de/brouter/segments4/)
./gradlew :brouter-core:test --tests '*LoopQualityTest' -Dloop.tests=true --rerun-tasks
./gradlew :brouter-core:test --tests '*LoopGoldStandardTest' -Dloop.tests=true \
    -Dloop.algorithms=probe,isochrone,greedy --rerun-tasks
```

`--rerun-tasks` is required when toggling `-D` flags (Gradle does not treat system
properties as task inputs, so a cached "skipped" result would otherwise be reused).

## Coverage (CI tier)

| Suite | Cases | Asserts |
|-------|-------|---------|
| `RoundTripInvariantTest` | 32 (4 profiles × 4 dir × 2 radii) | closure, no-beeline, reuse, non-degenerate, voice-hint sanity |
| `RoundTripContractTest` | 96 (edge radii) | success ⇒ valid loop; failure ⇒ no track; determinism |
| `RoundTripScenarioTest` | 6 | WAYPOINT / ISOCHRONE strategies, allowSamewayback, user via, roundTripLength |
| `RoutingEngineTest` (round-trip) | (existing) | closure, micro-detour, beeline, waypoint filtering, greedy/via |

## Data versioning

All routing data is **lookup v11**, matching the shipped `misc/profiles2`:
- The fixture rd5 is generated from the repo's current `lookups.dat` (v11) — automatically consistent.
- The gated tier uses freshly downloaded v11 tiles (`brouter.de/brouter/segments4/`) for the 5 test regions.

A tile's lookup version is the top 16 bits of the first 8-byte header long
(`PhysicalFile.java`); the gated suite fails fast on a mismatch.

## Invariants (CI tier — `RoundTripInvariantTest`)

For 4 profiles (trekking, fastbike, gravel, mtb) × 4 directions × radii {1000, 1500} m,
every produced loop must satisfy (thresholds calibrated from observed behaviour, set strict):

- **Closure** — start/end gap ≤ 100 m.
- **No beeline** — no consecutive segment > 1500 m, and no `BL` (beeline) voice hint.
- **Bounded reuse** — retraced distance ≤ 30% (a loop, not an out-and-back).
- **Non-degenerate** — ≥ 10 nodes.
- **Voice-hint sanity** — every hint indexes into the track `[0,n)`, and (outside
  roundabouts) carries an in-range turn angle `[-180,180]`. *Regression guard for the
  origin-chain bug that produced negative indices / out-of-range seam angles.*

## Contract (CI tier — `RoundTripContractTest`)

Across edge radii (small / medium / oversized), every round-trip request must either
produce a valid loop or fail cleanly — never report success for a non-loop:

- **Success ⇒ valid loop** — if no error, the track is non-degenerate (≥6 nodes, ≥200 m)
  and closes (start/end gap ≤ 150 m).
- **Failure ⇒ no track** — an infeasible request sets a clear error and returns no track.
- **Determinism** — identical inputs produce a byte-identical node sequence.

## Scenario coverage (CI tier — `RoundTripScenarioTest`)

Each generation strategy and mode must still produce a valid loop (or fail cleanly):
explicit **WAYPOINT** and **ISOCHRONE** strategies across all four cycling profiles, and
**allowSamewayback** out-and-back (closes at a feasible config; fails cleanly when it cannot
return). GREEDY is exercised in the gated tier where the network is large enough for it.

## Findings & fixes

- **Silent degenerate "success" (FIXED).** At constrained radii some `(profile, direction)`
  combos collapsed to a 1–3 node stub yet returned `err=null` (e.g. `trekking@0/180 r500`,
  `fastbike@180 r500`, `mtb@270 r5000` → a *single* node). Root cause: when
  `validateAndAdjustWaypoints` filtered out all intermediate waypoints (no reachable roads
  in the requested direction), routing produced a trivial track reported as success.
  Fix: `doRoundTrip` now rejects sub-loop output (`MIN_ROUNDTRIP_LOOP_NODES`/`_METERS`) with
  a clear error and no track.
- **Non-closing "loop" (FIXED).** `mtb@90 r5000` returned a 42-node track ending 884 m from
  the origin — not a loop. Fix: `doRoundTrip` rejects output whose start/end gap exceeds
  `MAX_ROUNDTRIP_CLOSURE_METERS` (400 m; real loops close within metres). Both guards are
  conservative and leave all well-formed loops (and the existing `RoutingEngineTest`) green.
- **Voice-hint sanity** holds across the whole matrix (no negative indices / out-of-range
  angles) — the earlier origin-chain fix verified by `RoundTripInvariantTest`.
- **`allowSamewayback` non-closing at some directions (OPEN, documented).** The out-and-back
  closes perfectly at e.g. dir 0/270 (gap ~1 m) but at dir 90 ends ~860 m from the origin —
  the return leg does not complete. The closure guard now reports this as a clear failure
  instead of returning a one-way stub as success; the underlying return-leg issue (in the
  same-way-back routing path) is left for review as it touches core routing logic.
- **Alpine long-loop continuity gaps (OPEN, gated tier).** `alpine_innsbruck` 100 km loops
  show large continuity gaps (maxGap 5–6 km) — beelines across unroutable mountain terrain.
  Within the lenient gated threshold today; flagged for the distance/continuity work.

_Last updated: 2026-05-22._
