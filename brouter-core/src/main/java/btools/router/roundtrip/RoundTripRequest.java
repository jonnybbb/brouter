package btools.router.roundtrip;

import btools.router.OsmTrack;

/**
 * Mutable state of one round-trip request, owned by the
 * {@link RoundTripOrchestrator} (one instance per {@code doRoundTrip} call).
 * Replaces the shared engine fields the orchestrator used to mutate through
 * the seam: the engine only receives what its search loops read (published as
 * runtime hints) plus the final telemetry values at request end.
 */
final class RoundTripRequest {

  /** Active effort preset, seeded from the engine's request-entry value. */
  RoundTripEffortPolicy effortPolicy = RoundTripEffortPolicy.STANDARD_PRESET;

  /** Active search radius in meters (published to the engine as a hint). */
  double searchRadius;

  /** Wall-clock request deadline (epoch ms), seeded from doRun; 0 = unbounded. */
  long requestDeadline;

  /** True in explicit-via mode (published to the engine as a hint). */
  boolean explicitVia;

  /** Per-leg guide tracks from a greedy adoption (published as a hint). */
  OsmTrack[] greedyLegTracks;

  /** Per-leg routing budget (ms) for the round-trip fallthrough; 0 = untimed. */
  long routingBudgetMs;

  /** Bounded tier's gate verdict, consumed once by the shared gate. */
  RoundTripQualityResult boundedGateVerdict;

  /** Forced same-way-back corridor latch from the greedy adoption. */
  boolean forcedCorridorAccepted;

  /** Last gate-rejected track; published to the engine at request end. */
  OsmTrack lastRejectedTrack;

  /** Planner result telemetry; published to the engine at request end. */
  RoundTripResult lastResult;
}
