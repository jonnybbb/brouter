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

### Round trips

**Loop**:
A route that starts and ends at the same point with no user via points, generated from a requested length and (optionally) a heading.
_Avoid_: round trip route, circuit, ring

**Leg**:
One planner-committed section of a loop, routed between two consecutive vias.
_Avoid_: sub-route, segment, section

**Skeleton**:
The ordered via sequence of a loop (start → vias → start) that the legs are routed between.
_Avoid_: waypoint list, via list, route plan

**Final via**:
The last via of a skeleton — the point the closing leg runs home from. Its position decides the closing leg's length and character.
_Avoid_: last waypoint, return point, turn-around point

**Closing leg**:
The final leg of a loop, from the final via back to the start.
_Avoid_: return leg, return path, way home

**Closing-first plan**:
A skeleton planned from the final via backward toward the start, so the closing leg is known exactly before the other legs are chosen.
_Avoid_: backward plan, reverse plan, return-first

**Filler leg**:
A short late leg whose only purpose is to reach the requested length after the loop turned home too early — a defect, not a design element.
_Avoid_: stub, padding

**Preferred road**:
A road the active profile prices at or near its minimum cost factor, i.e. what the profile would choose given the chance.
_Avoid_: gravel road, good road, track (a preferred road for gravel is not necessarily `highway=track`)

**Preferred area**:
A region dense in preferred-road length per unit area, as seen by the active profile. A single preferred road through hostile surroundings is not a preferred area.
_Avoid_: gravel area, desirable area, appeal, heatmap (the heatmap is the data structure, not the concept)

**Loop cost**:
The active profile's cost per metre of the shipped loop — the objective a loop is optimised for. Tag-based road character is an evaluation lens, never a planning input.
_Avoid_: appeal, desirability score, quality (quality is the gate's pass/fail vocabulary)
