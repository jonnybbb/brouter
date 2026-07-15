package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;

/**
 * The mutable request-state slice of the engine seam: result track/error,
 * budgets, effort policy, and the cross-tier latches. One of the four role
 * interfaces composed by {@link RoundTripEngineOps}.
 *
 * <p>This is intentionally the "shame list" of the seam split — every method
 * here is shared mutable state on the engine that Phase A2 wants to move into
 * an orchestrator-owned request object. Read-only views of two of these
 * fields ({@code explicitViaRoundTrip}, {@code roundTripSearchRadius}) live on
 * {@link EngineContext}; only their setters are here.
 */
public interface RoundTripRequestState {

  /** The live request waypoint list (the orchestrator appends generated vias). */
  List<OsmNodeNamed> waypoints();

  /** The engine's current result track (null while unset / after a reset). */
  OsmTrack foundTrack();

  /** Set the engine's result track. */
  void setFoundTrack(OsmTrack track);

  /** The engine's current error message (track-XOR-error contract). */
  String errorMessage();

  /** Set the engine's error message. */
  void setErrorMessage(String message);

  /** The engine's published matched waypoints. */
  List<MatchedWaypoint> matchedWaypoints();

  /** Publish the matched waypoints of the final track on the engine. */
  void setMatchedWaypoints(List<MatchedWaypoint> waypoints);

  /** Engine start wall-clock millis (deadline arithmetic). */
  long startTime();

  /** Set the engine start wall-clock (planner saves and restores around a leg). */
  void setStartTime(long startTimeMillis);

  /** Engine run budget in millis, 0 = untimed. */
  long maxRunningTime();

  /** Set the engine run budget (planner saves and restores around a leg). */
  void setMaxRunningTime(long maxRunningTimeMillis);

  /** Absolute wall-clock deadline for the whole round-trip request; 0 = untimed. */
  long roundTripRequestDeadline();

  /** Set the request deadline. */
  void setRoundTripRequestDeadline(long deadlineMillis);

  /** Per-leg routing budget for the round-trip fallthrough. */
  long roundTripRoutingBudgetMs();

  /** Set the per-leg routing budget. */
  void setRoundTripRoutingBudgetMs(long budgetMs);

  /** The active effort policy preset (null outside bounded dispatch). */
  RoundTripEffortPolicy roundTripEffortPolicy();

  /** Set the active effort policy preset. */
  void setRoundTripEffortPolicy(RoundTripEffortPolicy policy);

  /** Set the active search radius (consulted by engine-side matching hooks). */
  void setRoundTripSearchRadius(double searchRadius);

  /** Mark/unmark the explicit-via round-trip mode. */
  void setExplicitViaRoundTrip(boolean explicitVia);

  /** The forced-corridor acceptance latch from the greedy adoption. */
  boolean roundTripForcedCorridorAccepted();

  /** Set the forced-corridor acceptance latch. */
  void setRoundTripForcedCorridorAccepted(boolean accepted);

  /** The bounded tier's gate verdict, published for the shared doRun advisory. */
  RoundTripQualityResult boundedGateVerdict();

  /** Set the bounded tier's gate verdict. */
  void setBoundedGateVerdict(RoundTripQualityResult verdict);

  /** Per-leg guide tracks from a greedy adoption, consulted by doRouting. */
  OsmTrack[] greedyLegTracks();

  /** Set the per-leg guide tracks. */
  void setGreedyLegTracks(OsmTrack[] tracks);

  /** The last gate-rejected track (diagnostics surface). */
  OsmTrack lastRejectedTrack();

  /** Set the last gate-rejected track. */
  void setLastRejectedTrack(OsmTrack track);

  /** Publish the planner result for telemetry consumers. */
  void setLastRoundTripResult(RoundTripResult result);

  /** Wall-clock bound for the next isochrone expansion; 0 clears it. */
  void setTransientExpansionDeadline(long deadlineMillis);

  /** Pass-1 air-distance cost factor currently active on the engine. */
  double airDistanceCostFactor();

  /** Set the pass-1 air-distance cost factor (planner saves and restores it). */
  void setAirDistanceCostFactor(double factor);
}
