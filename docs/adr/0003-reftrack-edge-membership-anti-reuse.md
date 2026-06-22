# ADR-0003: refTrack anti-reuse uses edge-membership (round-trip now, general routing pending proof)

## Status

Accepted — edge-membership is **gated to round-trip mode**; general (non-round-trip)
routing keeps the historic node-membership test until the benefit is proven and
signed off (see "Path to global", below).

## Context

When BRouter computes a second alternative (`alternativeidx >= 1`) or a round-trip
return leg, it discourages re-using the roads of a previously-found track. The
mechanism is a per-link **anti-reuse penalty** in `OsmPath.addAddionalPenalty`:
a link that "belongs to" the reference track (`refTrack`) costs an extra
`linkdist × refTrackCostFactor`.

Historically "belongs to" meant **both-endpoints node membership**:

```java
if (refTrack.containsNode(sourceNode) && refTrack.containsNode(targetNode)) { ... }
```

This has a **false-positive** flaw. A fresh connector road `(A, Z)` whose two
junctions `A` and `Z` both happen to lie on the reference track — but which the
reference track **never drove** (it reached `A` and `Z` at different, non-adjacent
points) — is taxed as if it were a re-used road. The penalty then steers the
alternative *away* from a perfectly good connector and onto a more contrived detour.

The round-trip loop work introduced an **edge-membership** test that fixes this:

```java
if (refTrack.containsTraveledSegment(sourceNode.id, targetNode.id)) { ... }
```

A link is taxed only when the reference track actually **traveled** that segment.

That change had silently been on the general routing path too, which alters plain
`alternativeidx` output — contrary to the "behave-preserving for non-round-trip"
contract. We therefore gated it behind `RoutingContext.roundTrip` (commit gating
the edge form to engineMode 4), pinned the historic general-routing behavior with
`RoutingEngineTest.generalAlternativeRefTrackPenaltyIsHistoric`, and wrote down why
the global flip is expected to be a strict improvement.

## The subset lemma (why the global flip is safe)

For a **raw** `refTrack` (junction nodes only — what general alternatives use):

> the set of links **edge-membership** penalizes is a strict **subset** of the set
> **node-membership** penalizes.

*Proof.* A link `(s, t)` is edge-penalized iff `(s, t)` is a consecutive traveled
pair of the reference track. If it is, then both `s` and `t` are nodes of the
track, so it is node-penalized too: `edge-penalized ⊆ node-penalized`. The reverse
fails exactly for the false-positive connectors above (both endpoints on-track, never
driven consecutively). ∎

Three consequences:

1. **Retrace-avoidance is identical.** Every road the reference track actually drove
   is penalized the same in both modes — the alternative is pushed away from genuine
   re-tracing exactly as before.
2. **Per-path cost is monotone.** For any candidate path `P`,
   `cost_edge(P) ≤ cost_node(P)`, since the edge penalty term is pointwise ≤ the node
   penalty term.
3. **The optimum never worsens.** With an exact (admissible) search,
   `cost_edge(A_edge) ≤ cost_edge(A_node) ≤ cost_node(A_node)`, where `A_x` is the
   optimal alternative under penalty `x`. So edge-membership never yields a costlier
   alternative, and the only routes that change are those where node-membership wrongly
   taxed a natural connector and forced a detour.

In short: same divergence from the primary, fewer spurious penalties ⇒ equal-or-better
alternatives. Edge-membership is strictly less distorted, never more.

## Empirical confirmation

`AlternativeRefTrackMembershipTest` (integrationTest) is the proof harness. It uses
`RoutingContext.roundTrip` as a zero-code-change lever: it routes the same A→B
`alternativeidx=1` request once with the flag off (node-membership) and once on
(edge-membership) over an Urban-Berlin (grid) + Dreieich + Freiburg corpus on real
tiles (short to ~25 km), and for each pair measures the alternative's penalised cost,
real distance, and **overlap with the primary** (fraction of the alternative's traveled
edges the primary also drove — the divergence metric). It asserts the lemma
(`cost_edge ≤ cost_node` for every pair) and that overlap does not rise in aggregate.

Run it:

```bash
JAVA_HOME=<jdk17> ./gradlew :brouter-core:integrationTest \
  --tests 'btools.router.AlternativeRefTrackMembershipTest' \
  -Dloop.segments.nodownload=true --rerun-tasks
```

**Result (2026-06-22, 18 pairs): the lemma holds with `costViolations=0`, and
`differ=0` — node- and edge-membership produce byte-identical alternatives on every
pair**, including the dense Berlin grid and the 15–25 km routes. So the fix is
provably safe (the lemma) *and empirically inert* for general `alternativeidx`
routing: it changes nothing.

### Why general alternatives see no benefit (and round-trips do)

The membership tests differ only on a **chord**: an edge `(A, Z)` whose endpoints are
both on the reference track but were not driven consecutively. For a **shortest-path
primary**, exploitable chords are vanishingly rare — if a cheap shortcut between two of
its nodes existed, the primary would already have taken it. So the penalty sets
coincide and the alternative is identical.

A chord becomes common only when the reference track **self-approaches** — its outbound
and return passing near each other — which is the structural signature of a
**round-trip**, not a simple A→B route. That is exactly where the loop work needed the
fix (asymmetric return-leg wiggle): the return leg wants roads near the outbound
corridor, and node-membership wrongly taxed the connectors there.

## Decision

Keep edge-membership **permanently gated** to round-trip mode. The evidence shows
globalising it would change nothing for general `alternativeidx` routing, so the
historic node-membership test stays the default — simpler, and now backed by 18
byte-identical alternatives. The fix applies where it actually matters (self-approaching
round-trip refTracks).

This reverses an earlier expectation that general alternatives would benefit: they do
not, and the harness is the reason we know rather than assume.

## Consequences

- General `alternativeidx` output is unchanged — empirically confirmed, not just argued.
- The gate is the correct long-term design, not a temporary guard.
- `AlternativeRefTrackMembershipTest` stays as a regression guard: if a future change
  makes general alternatives diverge under the two tests, or violates the cost lemma,
  it fails loudly. (`reftrack-example.{geojson,html}` is emitted only if a divergent
  pair ever appears.)
