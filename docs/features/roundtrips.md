---
parent: Features
---

# Round-trip and loop routing

Most route planning answers the question *how do I get from A to B?*. Cyclists
and hikers, however, often have a different one: *I have an afternoon and want a
nice loop of about 40km that brings me back to where I started.* There is no
destination, only a starting point and a rough idea of how far you want to go.
BRouter can plan such round-trips.

Given a single start point and a desired length, BRouter places a ring of
intermediate waypoints around the start and routes a closed loop through them,
following the same [configurable cost function](costfunctions.md) as any other
route. The loop therefore respects your personal preferences on surface, hills
and road type just like a normal A-to-B route does.

You control the loop with a few request parameters:

| parameter | meaning |
| :----- | :----- |
| `roundTripLength` | desired total loop length in meters (takes precedence over `roundTripDistance`) |
| `roundTripDistance` | search radius in meters; the loop length is roughly `2π × radius` |
| `roundTripPoints` | target number of intermediate waypoints (3–20, default 5) |
| `startDirection` / `heading` | compass bearing to bias the direction the loop heads out |
| `roundTripDirectionAdd` | angle offset added to an auto-detected start bearing |
| `roundTripAlgorithm` | planning strategy: `AUTO`, `FAST`, `BALANCED`, `QUALITY` (the internal engine names `WAYPOINT`, `GREEDY`, `ISO_GREEDY`, `ISOCHRONE` are also accepted — see below) |

If you instead supply more than one waypoint, BRouter treats those as explicit
[via-points](vianogo.md) the loop must pass through in order, and the generated
ring is not used. The same length settings then act only as guidance.

## Planning strategies

Generating a good loop is harder than routing between two fixed points: the
waypoints are not given, so the planner has to *invent* a set of intermediate
targets and then check whether the resulting route is actually a pleasant,
closed loop. BRouter offers several strategies that trade speed for quality:

- **FAST** — places the ring of waypoints geometrically and scores them by
  straight-line distance only. Cheap enough for limited mobile hardware, but the
  loop quality is modest.
- **BALANCED** — an iterative planner that proposes candidate waypoints, routes
  only the most promising ones, and checks how well the loop closes before
  committing each leg. A good compromise and the usual production default.
- **QUALITY** — like BALANCED, but candidate waypoints are drawn from the area
  that is genuinely reachable from the start within a cost budget rather than
  from an abstract circle. This pays off in constrained terrain such as
  mountains, islands or coastlines, at the cost of more computation.
- **AUTO** — lets BRouter pick. It tries the higher-quality strategies first and
  falls back to a faster one only when the better result is not worth the extra
  effort.

The four names above are the recommended values. For parity with the engine, the
parser also accepts the internal algorithm names directly: `FAST` = `WAYPOINT`,
`BALANCED` = `GREEDY`, `QUALITY` = `ISO_GREEDY`, plus `ISOCHRONE` (direct
isochrone-frontier waypoint placement, also selectable with `roundTripIsochrone=1`
and AUTO-selected for small loops). Matching is case-insensitive and any
unrecognised value falls back to `AUTO`.

## Loop quality

A round-trip should look like a loop, not like an out-and-back with a detour.
BRouter applies several quality checks while planning:

- **Closure** — the route must return close to the start; grossly open routes
  are rejected.
- **No retracing** — a loop that travels back along roads it already used is
  penalised, so the way out and the way back differ.
- **Clean shape** — self-intersections in the developing loop are penalised to
  favour simple, non-tangled geometry.
- **Real loops only** — a valid loop encloses some area, so it needs the start
  plus at least two intermediate waypoints.

These checks make round-trip planning reliable enough to use without manually
tweaking the result, while still leaving the actual road choices entirely to
your routing profile.
