package btools.router.roundtrip;

/**
 * One rung of the round-trip tier ladder (FAST &lt; BALANCED &lt; AUTO &lt;
 * QUALITY). This is the sanctioned flexibility point of the subsystem (see
 * {@code .agents/adr-no-shared-planner-interface.md}): tiers are
 * interchangeable here; the planners below them are deliberately not.
 */
interface RoundTripStrategy {

  /**
   * Whether a strategy's {@link #attempt} still needs the orchestrator's
   * shared finalization (floors + uniform quality gate + advisories), or
   * already finalized the result itself. Replaces a bare {@code boolean}
   * return so the two meanings are named at every call site instead of
   * requiring tribal knowledge of which strategies self-finalize.
   */
  enum Outcome {
    /** Track/error left on the request; the orchestrator still runs the
     *  shared floors, uniform quality gate, and advisory decoration. */
    NEEDS_SHARED_FINALIZATION,
    /** The strategy finalized the result itself (the AUTO competition gates
     *  its candidates internally and decorates the winner) — the
     *  orchestrator must not run the shared floors/gate again. */
    SELF_FINALIZED
  }

  /**
   * Run one tier attempt under the given slice, leaving the outcome on the
   * request (track XOR error). See {@link Outcome} for what the return value
   * means for the orchestrator's finalization pipeline.
   */
  Outcome attempt(RoundTripRequest request, TierSlice slice);
}
