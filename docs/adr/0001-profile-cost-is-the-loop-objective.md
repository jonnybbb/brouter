---
status: accepted
---

# Profile cost per metre is the loop objective; road character only evaluates

The round-trip planner has three yardsticks that disagree: the profile's cost
per metre (what Dijkstra minimises), tag-based road character
(`RoadCharacterScore`: tracks good, residential bad, per profile family) and the
quality gate's shape rules. We decided that a loop is optimised for the active
profile's cost per metre only — the *loop cost* — and that tag-based road
character is an evaluation lens (matrix metrics, AUTO's `RouteChoiceScore`
comparison of already-built loops), never a planning input. Reason: the cost
model is the single thing a profile author controls; if a loop is cheap by cost
but poor by tags, the profile is wrong and gets fixed there, whereas a
planner-side tag preference would be a second, conflicting cost model that
custom profiles cannot influence. The same reasoning rejects any planner-side
"leave the built-up area" rule: that is `avoid_towns` in the profile.

## Considered options

- Tag-based desirability as the placement objective — profile-independent and
  ungameable, but it would make the planner disagree with the profile the
  rider chose.
- The existing `RouteChoiceScore` composite as the objective — it already mixes
  tags into the score; fine for choosing between finished loops, but as a
  planning target it would steer placement by tags through the back door.

## Consequences

- Every planner scoring term (phase-1 compiled-leg cost, hostility, the
  preferred-area density of a candidate) is derived from profile cost.
- A profile that prices residential streets like tracks will get loops through
  residential streets; that is by design and is a profile bug.
