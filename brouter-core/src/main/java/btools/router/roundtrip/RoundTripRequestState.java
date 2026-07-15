package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;

/**
 * Engine seam role: mutable request state shared between the round-trip
 * planners and the engine — result track/error, budgets, effort policy, and
 * the cross-tier latches. Intentionally the "shame list" of the seam split;
 * Phase A2 moves it into an orchestrator-owned request object. Two fields
 * have their read-only getters on {@link EngineContext}
 * ({@code roundTripSearchRadius}, {@code explicitViaRoundTrip}); only their
 * setters are here.
 */
public interface RoundTripRequestState {

  /** Live request waypoint list; the orchestrator appends generated vias. */
  List<OsmNodeNamed> waypoints();

  /**
   * The engine's result track; null while unset. Exactly one of foundTrack
   * and errorMessage is set on a finished request.
   */
  OsmTrack foundTrack();

  void setFoundTrack(OsmTrack track);

  /** The engine's error message; see {@link #foundTrack()} for the contract. */
  String errorMessage();

  void setErrorMessage(String message);

  /** Matched waypoints of the final track. */
  List<MatchedWaypoint> matchedWaypoints();

  void setMatchedWaypoints(List<MatchedWaypoint> waypoints);

  /** Engine start wall-clock millis. */
  long startTime();

  /** Engine run budget in millis; 0 = untimed. */
  long maxRunningTime();

  void setMaxRunningTime(long maxRunningTimeMillis);

  /** Wall-clock deadline for the whole round-trip request; 0 = untimed. */
  long roundTripRequestDeadline();

  void setRoundTripRequestDeadline(long deadlineMillis);

  /** Per-leg routing budget for the round-trip fallthrough. */
  long roundTripRoutingBudgetMs();

  void setRoundTripRoutingBudgetMs(long budgetMs);

  /** Active effort policy preset; null outside bounded dispatch. */
  RoundTripEffortPolicy roundTripEffortPolicy();

  void setRoundTripEffortPolicy(RoundTripEffortPolicy policy);

  /** Getter lives on {@link EngineContext#roundTripSearchRadius()}. */
  void setRoundTripSearchRadius(double searchRadius);

  /** Getter lives on {@link EngineContext#explicitViaRoundTrip()}. */
  void setExplicitViaRoundTrip(boolean explicitVia);

  /** Forced-corridor acceptance latch from the greedy adoption. */
  boolean roundTripForcedCorridorAccepted();

  void setRoundTripForcedCorridorAccepted(boolean accepted);

  /** Bounded tier's gate verdict, published for the shared doRun advisory. */
  RoundTripQualityResult boundedGateVerdict();

  void setBoundedGateVerdict(RoundTripQualityResult verdict);

  /** Per-leg guide tracks from a greedy adoption, consulted by doRouting. */
  OsmTrack[] greedyLegTracks();

  void setGreedyLegTracks(OsmTrack[] tracks);

  /** Last gate-rejected track (diagnostics). */
  OsmTrack lastRejectedTrack();

  void setLastRejectedTrack(OsmTrack track);

  /** Publish the planner result for telemetry. */
  void setLastRoundTripResult(RoundTripResult result);

  /** Wall-clock bound for the next isochrone expansion; 0 clears it. */
  void setTransientExpansionDeadline(long deadlineMillis);
}
