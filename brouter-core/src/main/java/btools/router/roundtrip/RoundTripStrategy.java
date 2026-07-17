package btools.router.roundtrip;

/**
 * One rung of the round-trip tier ladder (FAST &lt; BALANCED &lt; AUTO &lt;
 * QUALITY). This is the sanctioned flexibility point of the subsystem (see
 * {@code .agents/adr-no-shared-planner-interface.md}): tiers are
 * interchangeable here; the planners below them are deliberately not.
 */
interface RoundTripStrategy {

  /**
   * Run one tier attempt under the given slice, leaving the outcome on the
   * request (track XOR error). Returns true when the outcome still needs the
   * orchestrator's shared floors and uniform quality gate; false when the
   * strategy finalized the result itself (the AUTO competition gates its
   * candidates internally and decorates the winner).
   */
  boolean attempt(RoundTripRequest request, TierSlice slice);
}
