# BRouter

Offline bicycle/foot routing over OpenStreetMap data with user-editable cost
profiles. This glossary covers the terms specific to this project; the
round-trip (loop) planner is where most of the contested vocabulary lives.

## Language

### Round trips

**Loop**:
A route that starts and ends at the same point with no user via points, generated from a requested length and (optionally) a heading.
_Avoid_: round trip route, circuit, ring

**Leg**:
One planner-committed section of a loop, routed between two consecutive vias.
_Avoid_: sub-route, segment, section

**Closing leg**:
The final leg of a loop, from the last via back to the start.
_Avoid_: return leg, return path, way home

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
