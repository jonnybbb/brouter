# BRouter Routing Engine

Offline-capable bicycle/foot routing engine over OSM data, with profile-scripted cost models and a generated round-trip (loop) mode.

## Language

### Round-trip variety

**Variety seed**:
The `alternativeidx` value in round-trip mode: an arbitrary non-negative integer that deterministically selects one loop variant. Values have no quality ordering; seed 0 (or absent) means the unperturbed baseline route.
_Avoid_: alternative index (in round-trip context), random seed

**Alternative index**:
The `alternativeidx` value in classic point-to-point routing: an enumeration 0–3 where index k is the k-th alternative produced by penalizing ways used in earlier alternatives.
_Avoid_: seed (in classic-routing context)

**Direction focus**:
The compass bearing a round trip heads toward, resolved from `startDirection` (or a random/area-info draw when absent) plus `roundTripDirectionAdd`. Invariant across variety seeds: changing the seed never changes the direction focus.
_Avoid_: heading, orientation

**Score jitter**:
Deterministic seeded noise applied to a greedy candidate's heuristic score, used to vary which candidates get routed. Flips only near-tie decisions; never applied to measured (routed) cost comparisons.
_Avoid_: randomization, noise injection

**Direction-focus invariant**:
The rule that any seeded perturbation (score jitter, geometry knobs) must keep the loop's direction focus; perturbations affecting bearing are bounded accordingly.
