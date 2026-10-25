# Elevation source evidence and routing coverage

This change addresses two distinct failures: floating-point rejection of exactly
half-covered output samples, and a BEF-only hole heuristic that cannot prove whether
missing terrain is expected. It does not repair defective source heights.

## Conversion contract (version 2)

- Output sample footprints use exact rational longitude intersections, including
  PMTiles header bounds. Latitude projection uses StrictMath. Covered and total
  area use the same additions; no epsilon or hole budget is applied.
- Version 1 converter provenance cannot be resumed as version 2.
- A completed cell has a BEF, `.mapterhorn.properties`, and `.coverage.gz`.
  Coverage evidence binds the archive identity, converter version, sampling mode,
  grid geometry, BEF SHA-256, source tile presence, source tile SHA-256 and invalid
  source sample counts. Per-output flags record expected validity and whether any
  support came from absent or invalid source pixels. Unsampled output is explicit.
- The evidence is generated from source support before publication. A streaming
  read validates every BEF sample against expected validity, including edge pixels
  and whole-tile absence. It also checks all BEF bytes against the completed hash.
- Conversion publishes coverage before marking converter provenance complete.
  Resuming a completed cell requires its coverage evidence to validate.
- Absence is not a claim of water. Invalid-source support remains a quality defect
  even when enough valid support produced a finite height.

## Build-specific gate

`RoutingElevationGate nodes55 waytiles55 elevation lookups.dat quarantine.csv report`
reads actual cutter records before PosUnifier. It uses ElevationRaster's real
four-sample interpolation, including corners whose interpolation weight is zero.
The caller must provide a private hash-verified elevation snapshot; a read-only
mount of a mutable shared producer directory is insufficient.

Missing land cells, no-data required by land nodes, invalid source support, and
known finite source defects block the build. A node is exempt only when all its
incident accepted ways are explicitly ferry or waterway-only. A road/ferry shared
node is land-required; unknown classifications and unmatched problem nodes fail.
WayCutter5 writes each way to every tile it touches, so the matching `.wt5` contains
all accepted incident ways for a node tile.

Quarantine CSV bounds are expanded by the interpolation support margin. They must
cover known finite halos, not just values already below the decoder's -1000 m
cutoff. Legitimate below-sea-level terrain remains valid. No fallback, zero-fill,
or source repair is implicit in passing the gate.

The gate processes one cell at a time and caps retained problem nodes at one
million. Truncated records, missing evidence, hash/geometry mismatches, resource
limits and unclassified problem nodes fail closed. Each invocation requires a new
report directory. Reports contain causes, node/way IDs, coordinates, explicit
water-only exemptions, required cells, missing cells, and a pass/fail summary.

## Release integration boundary

The infrastructure publisher and build wrapper must bind this evidence to a
content-addressed release, copy verified inputs into private consumer storage,
and bind the gate result and final rd5 hashes to the same PBF, profiles, lookups,
converter/source revision and builder image digest. A gate report alone is not a
promotion receipt. Production promotion and Europe schedule resumption remain
blocked until real candidate coverage and staging-route validation succeed.

Tests use actual PMTiles fixtures, actual BEF encoding and encoded cutter records.
They cover both hemispheres, aligned meridians, archive clipping, shared seams,
all 16 missing-tile corner patterns against an independent area reference,
planted corruption, water-only/shared-node classification, finite quarantine halos,
negative terrain, record truncation and resource limits.
