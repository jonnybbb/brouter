---
status: accepted (decided 2026-09-24, confirmed M0 2026-09-25)
---

# In-core engine placement for post-tier round-trip refinement (Option A)

The round-trip refinement stage needs to take the finished loop produced by a
planner tier (GREEDY, ISO_GREEDY, or AUTO/QUALITY) and iteratively optimize it
by perturbing skeleton via points and re-routing affected legs. We had to decide
where this refinement stage resides in the system architecture.

We decided on **Option A: a stage inside `brouter-core`** (`RefineStage` in
`btools.router.roundtrip`), invoked in `RoundTripOrchestrator.doRoundTrip`
immediately after the quality gate and before track decoration.

## Considered options

- **Option A: In-core engine stage (accepted).** Placed inside `brouter-core`
  within `RoundTripOrchestrator`. It operates directly on the active
  `RoutingEngine` instance, sharing its loaded `NodesCache`, profile context,
  and routing operators without process or IPC overhead.
- **Option B: Standalone microservice or sidecar.** A separate optimizer
  process (in Java or Rust/CCH) communicating via HTTP/IPC. Rejected because:
  - It would duplicate the entire routing harness, `NodesCache` tile decoding,
    and profile expression evaluation.
  - It would break the single-process offline deployment model required by the
    Android app (`brouter-routing-app`, API 23).
  - IPC serialization of graph nodes and tracks would consume significant latency
    within the strict 3.0s budget.
- **Option C: Post-processing in the server layer (`brouter-server`).**
  Rejected because it would bypass the Android app and CLI tools entirely,
  preventing mobile and headless users from benefiting from refinement.

## Consequences

- Refinement components live in package `btools.router.roundtrip` with strict
  Android API 23 compatibility (no Java 8 streams/lambdas, no `Optional`,
  no `SplittableRandom`).
- Determinism is preserved: when refinement is disabled (`roundTripRefine=none`),
  the stage is a single boolean check and output is 100% bit-identical.
- Fast/WAYPOINT baselines and child engines in AUTO competition are skipped
  automatically without performing unnecessary cache resets.
- Speculative candidate evaluations never mutate engine state; all state
  transitions are published atomically only upon passing the ship predicate.
