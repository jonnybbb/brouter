---
status: rejected (measured 2026-08-30)
---

# Plan the loop from the final via backward (closing-first)

The greedy loop planner commits legs from the start outward and routes the
closing leg last, as a plain shortest path from wherever the length budget ran
out. Measured on the gravel matrix (2026-08), that makes the last 15 % of a loop
~30 % residential against ~22 % overall, and two post-hoc repairs failed for
the same reason: pricing alternative closing legs (through an approach via)
overshoots the ±5 % length contract in ~80 % of cases, because the final via's
position — decided by length, not by road quality — already fixes what the
closing leg can be. We decided to add a second planner lineage that chooses
the final via first (K candidates in the home ring, far half-plane, ranked by
the routed closing leg's cost per metre) and builds the skeleton backward
toward the start, so the closing leg is exact when every other leg is planned
and the opening leg gets today's priced-closure treatment mirrored. It runs for
non-paved profiles under the STANDARD/MAX effort presets and ships only when it
beats the forward plan on `RouteChoiceScore`.

## Considered options

- Hard-target forward planning (final via fixed, outbound legs planned toward
  it): keeps one planner, but the outbound legs still drift on estimated
  lengths until the last step.
- Beam search over partial skeletons: the general fix at 3–5× the routing
  work; kept as the escalation if closing-first moves the tail but not enough.
- Local via relocation (2-opt): cannot move a bad final via without moving the
  whole tail.

## Consequences

- Two planner classes share one abstract core (`AbstractGreedyPlanner`); the
  forward subclass must stay bit-identical (parity goldens, matrices).
- Backward candidates come from a forward expansion around the tail head, a
  proxy for "nodes that reach it"; routed legs are exact, so a one-way error
  only costs a wasted candidate.
- The user's heading remains a departure direction: the mirrored direction
  fade applies to the opening leg; loop orientation is decided by the winning
  final via.

## Outcome (2026-08-30)

Built as agreed (extraction refactor `2b9d5d73`, bit-identical on both
matrices; `ClosingFirstGreedyPlanner` with mirrored orientation hooks; K = 3/5
final-via candidates ranked by routed closing-leg cost; ships only when it
beats the forward loop). Two tuning rounds on the 230-cell gravel matrix:

| round | shipped in | last-15 % resid (all) | in shipped cells | cost/m (all) | p90 latency |
|---|---|---|---|---|---|
| 1: RCS wins | 59 cells | 30.1 → 30.4 % | 24.1 → 27.4 % | 2.954 → 2.987 | 9.4 → 12.9 s |
| 2: + opening-leg guard, must not cost more | 16 cells | 30.1 → 30.3 % | 16.0 → 21.4 % | 2.954 → 2.942 | 9.4 → 12.7 s |

Bar was −5 points. Even where closing-first won on score *and* cost per metre
(round 2), the residential share of both loop ends went up (head 21.5 → 28.8 %,
tail 16.0 → 21.4 %) while crossings, reuse and distance ratio improved: the
planner chose a cheaper closing leg, but the cheap roads near a residential
start are still residential, and the opening leg — now the one nobody plans —
absorbed what the closing leg shed. The defect is not which end is planned
first; it is that the start's surroundings bound both ends. Reverted (the
extraction commit too, so no abstraction remains without a second
implementation; both stay in history). Beam search, the agreed escalation, is
not warranted: its condition ("moves the tail but not enough") did not occur.
