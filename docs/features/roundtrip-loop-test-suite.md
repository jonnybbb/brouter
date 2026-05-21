# Round-trip loop test suite

Test coverage ensuring round-trip (loop) routing produces correct, high-quality
bike loops. Built in layers so correctness is enforced in CI while scale-dependent
quality is checked against real geography.

## Tiers

| Tier | Data | Runs in | What it checks |
|------|------|---------|----------------|
| **Invariants** | bundled `dreieich.pbf` fixture, regenerated to rd5 by the map-creator test (always version-consistent with `misc/profiles2`) | **CI** (no downloads) | structural correctness that must hold for *every* loop |
| **Quality (gated)** | downloaded v11 region tiles (`segments4`) | local / nightly, `-Dloop.tests=true` | distance-accuracy bands, road-reuse, shape, golden regressions across 5 real regions |

The fixture rd5 is tiny (Dreieich only), so distance-accuracy bands are unreliable
there (the data runs out before a large requested loop can form). Scale-dependent
metrics therefore live in the gated tier; the fixture tier asserts only
terrain-independent structural properties.

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

## Findings

- **Small-radius degeneration (open):** at r=500 m, some `(profile, direction)` combos
  (`trekking@0`, `trekking@180`, `fastbike@180`) return a 2–3 node "track" with
  `err=null` — i.e. *success reported for a non-loop*. The same directions yield proper
  loops at r=1000/1500, so roads exist; the small-radius waypoints get fully filtered and
  the engine degenerates silently. Tracked by `RoundTripContractTest`; see fix notes there.

_Last updated: 2026-05-22._
