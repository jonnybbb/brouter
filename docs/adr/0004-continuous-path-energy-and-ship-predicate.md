---
status: accepted (spike 2026-09-24, validated M0 2026-09-25)
---

# Continuous path walker cost oracle, composite search energy, and finished candidate ship predicate

ADR-0001 established that the profile's cost per metre is the sole loop
planning objective. However, historical `track.cost / distance` suffers from
path-state dependency: turn cost and elevation hysteresis reset at each via
leg start, raw legs lack detail transfer nodes, and `RoundTripTrackCleanup`
leaves `track.cost` stale. Comparing raw leg costs directly against shipped
track costs introduces a comparator noise of −1.1 % to +2.1 %, which is as
large as the target improvement threshold.

We decided to implement a dedicated **linear path walker** (`walkPathCost`)
in `RoutingEngine` and use it to power `LoopCostOracle`. It evaluates a continuous
link-by-link traversal across all vias without artificial resets, handles
clipped endpoints, and prices both baseline and candidate loops under the
exact same cost function.

Furthermore, we established a strict separation between **search energy**
(which guides proposal acceptance on raw legs) and the **ship predicate**
(which gates final publication of detailed candidates).

## Considered options

- **Guided search for re-pricing (rejected):** Attempted during the spike.
  Failed because detailed tracks and cleanup insert/remove transfer nodes,
  causing index drift in `guide[treedepth + 1]`.
- **Per-leg pricing fallback:** Summing separately routed legs. Feasible, but
  causes pricing to vary when the same geometry is partitioned into different
  leg counts (violating segmentation invariance). Kept only as a backup; the
  continuous path walker succeeded in M0.1 and was adopted.
- **RCS as search energy (rejected):** Rejected per ADR-0001. RCS weights tags
  at 0.17 and would steer planning toward arbitrary tag heuristics rather
  than the rider's chosen profile.
- **Single objective for search and shipping (rejected):** A composite search
  energy improving does not prove that the finished loop is better on cost,
  length, or shape.

## Search Energy Definition

The search energy $E$ for an intermediate proposal is the continuous oracle
cost per metre of the concatenated raw legs:
$$E = \text{LoopCostOracle.price}(\text{concatenated raw legs})$$

All skeleton constraints (displacement $\le 0.25 \times R$, adjacent via
spacing $\ge 300\text{ m}$, radius bound $\le 0.60 \times L$) are enforced
as hard pre-routing filters. Every evaluated search state is therefore feasible.

## Ship Predicate Contract

A candidate loop is finalized through detail retrack, junction merge, and
`RoundTripTrackCleanup`. To replace the baseline, it must satisfy all clauses
of the authoritative ship predicate (§4.5):

1. **Cost improvement:** $C_{\text{ref}} \le 0.995 \cdot C_{\text{base}}$
   (at least 0.5% lower oracle cost/m).
2. **Length fidelity:** $|L_{\text{ref}} / L_{\text{req}} - 1| \le |L_{\text{base}} / L_{\text{req}} - 1|$
   (replacement is never further from the requested distance).
3. **Shape & reuse:** self-crossings $\le \text{crossings}_{\text{base}}$
   and scatter reuse $\le \text{scatter}_{\text{base}}$.
4. **Direction fidelity:** bearing to farthest point within 15° of baseline
   (waived only if baseline farthest point is $< 500\text{ m}$).
5. **Quality gate:** passes `RoundTripQualityGate.evaluate` with the full
   request context.
6. **RCS guard (D2):** $RCS_{\text{ref}} \ge RCS_{\text{base}} - 0.02$
   (prevents shipping loops that AUTO's own ranking would discard).

## Consequences

- Unit and integration tests verify exact pricing reproduction (0.00% delta
  on $\ge 40$ fixture legs) and segmentation invariance.
- The published route is guaranteed to be never worse than the baseline under
  the multi-attribute ship predicate.
- When refinement fails or finds no candidate meeting all criteria, the
  baseline loop ships 100% byte-identical.
