# No sub-600m floor for the near-revisit (teardrop) detectors

The round-trip teardrop/near-revisit detectors share a 600m minimum-arc floor
(`NEAR_REVISIT_MIN_ARC_M`); the 2026-06-09 loop review asked whether lowering
it would catch smaller defects. We measured instead of tuning
(`SubArcFloorProbeTest`, opt-in `-Dfloor.probe=true`: 32 AUTO routes over two
switchback-rich alpine regions and two flat-dense ones, full-resolution
geometry, bands isolated per candidate floor) and **rejected the lowering**:

- **[100, 300) arc band:** 31/32 routes flagged, ~3 spans per route (alpine
  3.5, flat 2.5; median arc ~120m). That is universal road-network
  micro-geometry — junction loops, hairpins, dual-carriageway turns — not a
  defect class. A 100m floor would charge nearly every shipped loop the
  teardrop ranking penalty, degrading AUTO selection corpus-wide.
- **[300, 600) arc band:** nearly empty — 4 spans across 2 of 32 routes, zero
  in the alpine regions, none labeled as a real defect. There is no defect
  population below 600m for a lowered floor to find.
- **[600, 10k) (today's band):** 5 spans / 32 routes — the selectivity the
  production penalty was calibrated on.

The 600m floor sits on the measured signal-to-noise cliff. This confirms two
prior lessons: the locked spur-detector decision ("do NOT keep
geometric-threshold tuning — can't converge with ~1 labeled positive") and
the debunked beeline hard gate (a geometric threshold on a smooth continuum
fired 1283×/run on legitimate geometry). Severe small detours remain covered
by `removeMicroDetours`, which REPAIRS arcs ≤1500m (50m proximity, ratio>3)
rather than penalizing them.

Revisit only with labeled sub-600m positives in hand — and then prefer a
confirming non-geometric signal (e.g. graph avoidability) over lowering the
geometric floor.
