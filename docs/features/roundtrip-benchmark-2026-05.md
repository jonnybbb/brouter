# Round-Trip Tier Benchmark — May 2026

Full opt-in benchmark per spec §11 across all 5 test regions, 3 cycling profiles
(fastbike, gravel, mtb), 5 target distances (30/50/75/80/100 km), 4 compass
directions, and 4 algorithm variants (probe/WAYPOINT, ISOCHRONE, GREEDY, ISO_GREEDY).

**Run command**:
```
./gradlew :brouter-core:test --tests LoopQualityTest -Dloop.tests=true
```

**Run summary**: 300 parameterized cases, 67 skipped (DREIEICH region — no
algorithm produces a valid loop on the synthetic test fixture there), 853 routes
across the 4 variants, 33 min wall time on a M1 Mac with 8 GiB test heap.

Reports written to `brouter-core/build/reports/loops/` (HTML + GeoJSON per region
and aggregated).

## Per-variant aggregates

| Variant | Routes | Avg Ratio | Avg Reuse | Wins (of 240 cells) |
|---|---|---|---|---|
| WAYPOINT (probe) | 233 | 1.18 | 4.3% | 33 |
| ISOCHRONE | 229 | 1.06 | 4.1% | 25 |
| GREEDY | 156 | **1.00** | **3.8%** | **110** |
| ISO_GREEDY | 235 | 0.82 | 9.2% | 72 |

A "win" = the variant with the highest composite (40 % distance + 40 % reuse +
20 % direction-delta) among the four for that
(region, profile, distance, direction) cell.

## Per-profile distance-ratio detail

| Profile | WAYPOINT | ISOCHRONE | GREEDY | ISO_GREEDY |
|---|---|---|---|---|
| fastbike | 1.21 | 1.17 | 1.01 | **1.01** |
| gravel | 1.30 | 1.04 | 1.00 | 0.94 |
| mtb | 1.04 | 0.98 | 1.00 | **0.48** |

**ISO_GREEDY's mtb collapse (0.48)** is the limiting finding: on the MTB profile,
ISO_GREEDY produces loops that are roughly half the requested length. Root cause
is that the isochrone candidate extraction's cost budget (`searchRadius * 4`)
translates to a much shorter air-distance frontier on MTB (because each meter of
trail costs ~3 in cost-units, vs ~1.3 for fastbike). Candidate placements end
up close to start, and the planner builds short loops out of them. Fixing this
needs a profile-cost-factor compensation on candidate distance, similar to the
existing one in `placeWaypointsFromIsochrone`. Out of scope for this iteration.

## Per-region wins

| Region | GREEDY | ISO_GREEDY | WAYPOINT | ISOCHRONE |
|---|---|---|---|---|
| ALPINE_INNSBRUCK | 15 | **25** | 11 | 9 |
| COASTAL_NICE | 24 | 19 | 7 | 10 |
| RURAL_LOZERE | **42** | 12 | 5 | 1 |
| URBAN_BERLIN | 29 | 16 | 10 | 5 |

ISO_GREEDY's hypothesis ("better in mountains") **is borne out** for
ALPINE_INNSBRUCK (25 wins vs GREEDY's 15) — but only on fastbike + gravel
profiles. On mtb in alpine, GREEDY still wins because of the ratio-collapse
issue above.

## AUTO policy decision

**Keep current AUTO unchanged** (`ISOCHRONE` for radius < 5000m, `GREEDY`
otherwise; `ISO_GREEDY` opt-in only). The benchmark data does not support
auto-selecting ISO_GREEDY in any specific regime:

- ISO_GREEDY beats GREEDY in only 30 % of cells overall and is dominated by
  GREEDY in rural and urban regions.
- ISO_GREEDY's mtb collapse (avg ratio 0.48) would silently produce
  unacceptably short loops if AUTO routed all mtb requests to it.
- ISO_GREEDY's only consistent advantage is **alpine + (fastbike|gravel)** —
  too narrow a slice to justify per-cell AUTO branching today.

Users who want QUALITY-tier should set `roundTripAlgorithm=ISO_GREEDY`
explicitly and pair it with a road-cycling profile (fastbike or gravel),
ideally on terrain with strong terrain anisotropy (mountain valleys).

## Known limitations carried forward

1. **ISO_GREEDY on mtb** — ratio 0.48; fix would need profile-cost-factor
   compensation in `IsochroneCandidateProvider.fromPool` or
   `runIsochroneExpansion`'s cost budget.
2. **DREIEICH region failure** — the synthetic in-CI fixture has too little
   geographic coverage; no algorithm produces a valid loop. Real production
   uses different segments; this is a test-data artifact.
3. **GREEDY's lower ok-count (156 vs 230+)** is from timeouts on the larger
   75/80/100km cases, not the algorithm being broken — its per-step Dijkstras
   exceed the planner's per-leg budget when the start is far from a dense
   network. Acceptable: GREEDY falls back to WAYPOINT in those cases via
   `greedySupports`.

## Telemetry from `RoundTripResult`

The `candidatesGenerated`, `candidatesRouted`, `returnChecksPerformed`, and
`runtimeMillis` counters added in this iteration are now logged via
`logInfo("greedy telemetry: ...")` and accessible programmatically on the result
object for downstream analysis. Surfacing them as columns in the HTML report
is a small follow-up.
