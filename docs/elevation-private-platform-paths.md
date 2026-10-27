# Private offshore-platform paths

The approved routing policy excludes confirmed private offshore-platform paths before encoding or cutting routing ways. It does not assign sea-level elevation to platform decks and does not waive the elevation gate for retained nodes.

The motivating source is [OSM way 828174763](https://www.openstreetmap.org/way/828174763), a `highway=path` linking nodes 7731566750 and 1042145845. The first node explicitly has `man_made=offshore_platform` and `access=private`; the other has `location=platform` and `seamark:type=platform`. These source tags were rechecked through the OSM API on 2026-10-27. The way itself has no access or platform tag. No object ID or coordinate is part of the exclusion rule. Verification against the exact regional PBF and full-region acceptance remain required.

## Admission rule

An excluded way must satisfy all of these conditions:

- Its raw `highway` is `path`, `footway`, `steps`, `cycleway` or `pedestrian`, with at least two distinct nodes.
- Every referenced node is explicitly tagged `man_made=offshore_platform`, or has both `location=platform` and `seamark:type=platform`.
- At least one of those nodes has both `man_made=offshore_platform` and `access=private`.
- Neither the way nor its platform nodes has a public/unknown access override or a conditional tag. Explicit `no` and `private` values remain restrictions. Access checks include the common foot, bicycle, horse, vehicle, motor-vehicle and ski modes; directional or other mode-prefixed keys conservatively preserve the way.
- The way has no meaningful `route`, `railway` or `waterway` tag. Explicit `no` values are absent; all other values preserve the way for normal admission and validation.

Missing platform evidence is not inferred from proximity, elevation gaps, sea coordinates, a private tag alone, or untagged intermediate nodes. Relation membership and area containment are not inferred. Such paths remain in the normal pipeline and must pass its existing checks.

The source-node index is bounded at 100,000 explicitly tagged platform nodes; overflow fails with an explicit error. Strictly increasing raw node IDs followed by ways prove node uniqueness without indexing all regional IDs. If ordering or uniqueness becomes ambiguous before any exclusion, exclusion is permanently disabled and ways remain for normal validation. Ambiguity after an exclusion fails the cut explicitly. Every raw node and way participates in this check, including untagged records. Raw tags are inspected before pseudo-tags or lossy dictionary encoding. The cutter reports excluded way IDs and a final count.

Excluding a way does not globally exclude its nodes. A platform node shared with any retained land way still reaches the elevation gate and must have valid elevation.

## Validation

Real-format PBF regressions run the actual parser, `OsmFastCutter`, and `RoutingElevationGate`. They verify the removed path and unused nodes are absent, ordinary private paths and land connections remain, public/conditional overrides remain, mixed transport tags remain, unmarked intermediate nodes prevent exclusion, either endpoint can carry the private marker, and shared land nodes still fail without elevation. Existing ferry regressions remain mandatory.

This source change requires a new published builder SHA/digest and fresh exact-artifact acceptance. Previous candidate receipts do not approve a different builder. Production promotion and Europe resumption still require the regional, native-reader, backup/restore and live-route checks in the infrastructure release runbook.
