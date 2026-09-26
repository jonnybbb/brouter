---
parent: Developers
title: HTTP server
---

# Run the BRouter HTTP server

Helpers scripts are provided in `misc/scripts/standalone` to quickly spawn a
BRouter HTTP server for various platforms.

* Linux/Mac OS: `./misc/scripts/standalone/server.sh`
* Windows (using Bash): `./misc/scripts/standalone/server.sh`
* Windows (using CMD): `misc\scripts\standalone\server.cmd`

The API endpoints exposed by this HTTP server are documented in the
`ServerHandler.java`

Please see also [IBRouterService.aidl](./android_service.md) for calling parameter.

## Segment gradients and filtering

CSV and GeoJSON `messages` include a final `Gradient` column. Values are signed
integers in **tenths of a percent**: `50` means a 5.0% ascent, `-50` a 5.0%
descent, and `0` a measured flat segment. Unavailable gradients are empty strings,
not zero. GeoJSON message values remain strings, including this empty value.

Gradient uses elevation change divided by horizontal distance. Reporting refreshes
after final elevation and geometry processing, and uses interpolated elevations
at clipped endpoints. It does not change route selection or profile costs.
Consecutive messages with the same way tags are aggregated using total elevation
change divided by total distance. Ascents and descents can cancel within one
aggregate. If any positive-distance portion has unavailable elevation, the
aggregate gradient is unavailable. These rows follow way-tag boundaries; they
are not a list of detected climbs.

The following optional query parameters filter CSV rows and GeoJSON `messages`:

| Parameter | Unit | Meaning |
|---|---|---|
| `minLength` | metres, integer | Inclusive minimum aggregate length |
| `maxLength` | metres, integer | Inclusive maximum aggregate length |
| `minGradient` | percent | Inclusive minimum absolute aggregate gradient |
| `maxGradient` | percent | Inclusive maximum absolute aggregate gradient |

For example, `&format=geojson&minLength=2000&minGradient=5` keeps message rows
at least 2,000 m long with an average gradient magnitude of at least 5%.
A `minGradient` of `5` uses percent, whereas output `Gradient` uses tenths of
a percent. Both uphill and downhill segments can match.

All supplied limits apply together. Omitted or negative limits are disabled.
Rows with unavailable gradient are excluded when either gradient limit is active,
but remain eligible for length-only filters. Without filters, all aggregated
messages are returned. Filters do not change route coordinates, distances, route
choice, or GPX/KML output.

Existing inverse-routing message assembly can omit the first segment after an
intermediate waypoint. Filters operate on the available message rows; they do
not restore that missing coverage.
