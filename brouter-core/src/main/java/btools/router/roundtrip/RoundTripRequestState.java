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

  /** Wall-clock deadline set by doRun for the whole request; request-entry seed. */
  long roundTripRequestDeadline();

  /**
   * The single publication point for the request fields the engine's search
   * loops read mid-search (radius, deadline, explicit-via, guide tracks).
   */
  void setRoundTripRuntimeHints(RoundTripRuntimeHints hints);

  /** Per-leg routing budget (ms) computed by doRun; request-entry seed. */
  long roundTripRoutingBudgetMs();

  /** Effort preset request-entry seed / child-engine handoff. */
  RoundTripEffortPolicy roundTripEffortPolicy();

  void setRoundTripEffortPolicy(RoundTripEffortPolicy policy);

  /** End-of-request publication for {@code getLastRejectedTrack()}. */
  void setLastRejectedTrack(OsmTrack track);

  /** Publish the planner result for telemetry. */
  void setLastRoundTripResult(RoundTripResult result);
}
