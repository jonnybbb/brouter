# alternativeidx is a variety seed in round-trip mode, not an enumerated alternative

In classic point-to-point routing, `alternativeidx` (0–3) enumerates alternatives: the engine
generates route 0, penalizes its ways via refTracks, generates route 1, and so on. Round-trip
mode never entered that loop, so the parameter was dead there. We decided to reuse it for
round trips with **seed semantics**: any integer ≥ 0 deterministically selects one loop
variant, values carry no quality ordering, and seed 0 (or absent) is bit-identical to the
unperturbed baseline.

We rejected porting the enumeration semantics (refTrack penalty chain) because it costs
idx+1 full round-trip plans per request — round trips already run a 30-second wall-clock
budget, so idx=3 could quadruple it. Seed semantics is a single generation pass.

Consequences worth recording:

- One parameter, two semantics by engine mode. Classic routing keeps the 0–3 clamp;
  round-trip mode reads the raw value (negative clamps to 0).
- **Seed-0-inert contract**: seed 0/absent must stay bit-identical to the pre-feature
  planner output, protecting gold-standard tests and existing clients.
- **Direction-focus invariant**: the seed never influences the start-direction draw;
  variety comes from seeded score jitter (greedy family) and bounded geometry knobs
  (WAYPOINT/ISOCHRONE). Reproducibility requires passing `startDirection` + seed.
- Variety is best-effort: in sparse networks where no scoring decision is a near-tie,
  different seeds may return the same loop. The quality gate is unchanged — a seeded
  loop that fails the gate is returned with a Warning (lenient mode), exactly like today.

Calibration evidence (2026-06-10, `VarietySeedCalibration*Test`, 32 cases × seeds 0–4 =
160 AUTO-lenient plans over FREIBURG/BASEL/URBAN_BERLIN/ALPINE_INNSBRUCK at 30/50 km ×
fastbike/gravel × N/E): jitter amplitude 0.10 confirmed — median divergence vs seed 0
was 0.81–0.85 (1 − node Jaccard), 25–28/32 cases per seed produced a structurally
different loop (>30% divergence), only 0–6/32 were inert (<5%). Quality cost: zero —
160/160 formed, 0 gate warnings (seed 0 also 0), paired drift vs seed 0 within ±0.05
cost/m and ±0.006 RouteChoiceScore. Reproducibility verified: a full re-run of the
Freiburg shard reproduced every metric of all 8 cases × 5 seeds bit-identically.
