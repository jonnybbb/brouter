---
status: accepted
---

# Post-tier refinement stage outcome and opt-in activation

## Context

Following ADR-0001 (profile cost objective), ADR-0002 (frozen final via and closing leg invariance), ADR-0003 (in-core engine stage placement), and ADR-0004 (continuous path energy and ship predicate), `docs/roundtrip-refine-spec.md` specified a post-tier refinement stage to optimize shipped loop skeletons.

The project established milestones to evaluate whether local search could improve loop cost without introducing regressions:
- **M0 (Foundations):** Built and validated the `LoopCostOracle` continuous path walker, geometry fidelity invariants, failure-resilient `RefineFinalizer`, and leg re-routing `RefineInitializer`.
- **M1 (Pilot Experiment):** Measured the MOVE operator across 32 stratified gravel cells and 4 fastbike cells over budgets {4, 8, 16, 32}. Verified the never-worse invariant (100% compliance) and confirmed that `local` search at 16 evaluations provided the optimal trade-off between cost improvement (+3.2% median gain on eligible cells) and latency (~1.1s added p90).
- **M2 (Productisation):** Implemented request parameter plumbing with strict clamping:
  - `roundTripRefine` (`none`, `local`, `anneal`, `best_of_n`; default `none`)
  - `roundTripRefineEvals` (1–64; default 16)
  - `roundTripRefineMaxMs` (500–8000; default 3000)
  Enforced child suppression (`roundTripSuppressDecoration` causes skip with `auto_child`), verified all 10 skip reasons, and implemented corridor REPLACE, sub-chain 2-OPT, and orthogonal segment INSERT proposal operators.
- **M3 (Full Matrix Evaluation):** Evaluated paired baseline vs refined routes across regional cells. Refinement met all quality and safety criteria:
  - 0 cells worse under the ship predicate.
  - 0 new quality gate rejections.
  - Absolute length error $\le$ baseline on 100% of refined loops.
  - Self-crossings $\le$ baseline on 100% of refined loops.
  - Paired added latency p90 $\le 3.0$s.
  - Truncation rate $\le 5\%$.

## Decision

1. **Retain refinement as an opt-in capability:**
   In accordance with D7, refinement stays **off by default** (`roundTripRefine=none`).
   Callers explicitly opting in via `roundTripRefine=local` (or `anneal`, `best_of_n`) receive the post-tier refinement stage.
2. **Zero-overhead baseline:**
   When `roundTripRefine=none` (the default), the engine skips refinement immediately with reason `refine_off`, executing zero additional Dijkstra searches, zero leg routing, and allocating zero extra data structures. Parity tests (`GreedyPlannerParityTest`, `LoopGoldenSignatureTest`) remain 100% bit-identical.
3. **Strict invariant preservation:**
   The closing leg and final via remain immutable anchors in all search mutations. The ship predicate acts as a hard filter ensuring no candidate with higher oracle cost, worse length error, increased self-crossings, or gate violations ever replaces the baseline.

## Consequences

- The BRouter round-trip planner gained a proven, robust local search stage that reliably improves loop quality on demand.
- Mobile clients (e.g. Android app on API 23) can run refinement safely within configurable evaluation and time ceilings without risk of UI thread stall or memory overhead.
- HTTP server and CLI users can request refined loops via standard URL parameters.
- Future work (M4 deferred operators like DELETE or dense skeletons) can build directly on top of the established `RefineSearch` and `MoveProposalOperator` infrastructure.
