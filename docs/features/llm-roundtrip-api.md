# BRouter Round-Trip API — LLM Usage Guide

Generate a closed-loop bike route from a single start point.

## Endpoint

```
GET http://<host>:17777/brouter
```

## Minimum viable request

```
GET /brouter
  ?lonlats=<lon>,<lat>
  &profile=<profile>
  &engineMode=4
  &roundTripLength=<meters>
```

Example — 30 km fastbike loop starting in Berlin, heading east:

```
http://localhost:17777/brouter?lonlats=13.400,52.520&profile=fastbike&engineMode=4&roundTripLength=30000&direction=90&format=geojson
```

## Required parameters

| Param | Meaning |
|---|---|
| `lonlats` | Start point as `lon,lat` (decimal degrees, WGS84). Extra points after `\|` become forced via waypoints on the loop. |
| `profile` | Profile filename from `misc/profiles2/` without `.brf`. Common: `fastbike`, `gravel`, `mtb`, `trekking`. |
| `engineMode=4` | **Required.** Without this the server does point-to-point routing, not a round-trip. |
| `roundTripLength` **or** `roundTripDistance` | Loop sizing (see next section). |

## Loop sizing — pick ONE

**Prefer `roundTripLength`:**

- `roundTripLength=<meters>` — target **total loop distance**. Converted internally to radius = length / (2π). This is the parameter a user actually thinks in.

**Legacy:**

- `roundTripDistance=<meters>` — this is a **search RADIUS**, not a loop length. Actual loop ≈ 2π × radius. Default: `1500` (≈ 9.4 km loop) if both are omitted.

If both are set, `roundTripLength` wins.

## Direction (soft hint)

| Param | Default | Notes |
|---|---|---|
| `direction` (alias: `heading`) | `-1` (random) | Start bearing in degrees: `0`=N, `90`=E, `180`=S, `270`=W. |
| `roundTripDirectionAdd` | `45` | Degrees added to the randomly picked direction when `direction` is unset. Ignored when `direction` is explicit. |

**Critical semantic:** direction is a **hint**, not a constraint. The router may legitimately traverse the loop in the opposite direction (the loop still covers the requested bearing as its principal axis, just CW vs CCW). In asymmetric terrain (coast, mountain dead-end, sparse network) it may also ignore the hint entirely. Do not promise the user a specific traversal order.

## Algorithm selection

| Param | Values | Default |
|---|---|---|
| `roundTripAlgorithm` | `AUTO`, `GREEDY`, `ISOCHRONE`, `WAYPOINT` (case-insensitive) | `AUTO` |
| `roundTripIsochrone` | `0` \| `1` (legacy) | unset → maps `1` to `ISOCHRONE` |

`AUTO` picks by radius: `GREEDY` if radius ≥ 5000 m (≈ 31 km loop), otherwise `ISOCHRONE`. Leave on `AUTO` unless the caller explicitly wants a specific strategy.

## Optional parameters

| Param | Range / values | Default | Effect |
|---|---|---|---|
| `roundTripPoints` | 3–20 | auto: `max(5, min(15, radius/1500 + 3))` | Number of intermediate waypoints the planner places around the loop. More points = more control, slower. |
| `allowSamewayback` | `0` \| `1` | `0` | `1` produces a simple out-and-back on the same roads (one tip point at `searchRadius` in `direction`), not a round loop. |
| `format` | `gpx` \| `kml` \| `geojson` \| `csv` | `gpx` | Output encoding. |
| `trackname` | string | profile-derived | Name embedded in the track output. |
| `exportWaypoints` | `0` \| `1` | `0` | Include the internal waypoint sequence in the response. |

## Output

Response body is the track in the requested `format`, containing the full loop polyline (start == end), plus distance, ascent/descent, and cost as profile-computed metadata. HTTP `Content-Type` matches the format.

## Gotchas

1. **Loop length is approximate.** Constrained road networks (islands, dead-end valleys, coast) can produce loops significantly longer than requested. If actual / expected > 1.5, the server logs a warning and attaches a `message` to the track suggesting a shorter distance or out-and-back.
2. **Start point must be near a road.** Points > 250 m from the road graph produce beeline segments to the nearest node.
3. **Timeout.** Default 60 s per request (configurable via `-DmaxRunningTime=<seconds>` on the server JVM). Long loops on dense networks can hit this.
4. **Single start point only.** For round-trip, only the first `lonlats` entry is the start. Extra `|`-separated points are inserted as via waypoints along the loop, ordered by bearing from start — not in request order.
5. **Profile must exist on the server.** The profile name is resolved against the server's profile directory; a wrong name returns an error.

## Parameter cheat-sheet

```
lonlats              START[|VIA|VIA...]   required
profile              <name>               required (no .brf suffix)
engineMode           4                    required
roundTripLength      <meters>             preferred sizing (total loop)
roundTripDistance    <meters>             alternative (search RADIUS; default 1500)
direction            0-360 | -1           soft hint; -1 = auto
roundTripDirectionAdd  0-360              offset when direction=-1 (default 45)
roundTripAlgorithm   AUTO|GREEDY|ISOCHRONE|WAYPOINT  default AUTO
roundTripIsochrone   0|1                  legacy; prefer roundTripAlgorithm
roundTripPoints      3-20                 auto if unset
allowSamewayback     0|1                  1 = out-and-back
format               gpx|kml|geojson|csv  default gpx
```
