package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoutingContext;

/**
 * The round-trip planners' seam to the routing engine: coarse delegates to the
 * engine's leg router, matcher, isochrone expansion, timers, and logging, so
 * {@link GreedyRoundTripPlanner} and {@link GraphNativeCandidateProvider} never
 * touch engine internals directly and the engine's members stay
 * package-private. The production adapter is {@code RoutingEngine#roundTripOps()}
 * — the engine's single public hand-out for this package (same pattern as
 * {@link FastPlacementOps}). Method names mirror the engine members they
 * delegate to.
 */
public interface RoundTripEngineOps {

  /** The request context (read-only use; the planner reads profile knobs). */
  RoutingContext routingContext();

  /** Engine log line; a no-op in {@code quite} child engines (AUTO candidates). */
  void logInfo(String msg);

  /** True when the engine was terminated (watchdog/cancel). */
  boolean isTerminated();

  /** Engine start wall-clock millis (deadline arithmetic). */
  long startTime();

  /** Engine run budget in millis, 0 = untimed. */
  long maxRunningTime();

  /** The active round-trip search radius (0 outside round-trip requests). */
  double roundTripSearchRadius();

  /** True when the engine runs in round-trip mode. */
  boolean isRoundTripMode();

  /** True while an explicit-via round trip is being generated. */
  boolean explicitViaRoundTrip();

  /** Recalculate a track's totals (engine primitive shared with normal routing). */
  void recalcTrack(OsmTrack track);

  /** Merge duplicate round-trip voice hints (needs VoiceHint package internals). */
  void consolidateRoundTripVoiceHints(OsmTrack track);

  /** Publish the matched waypoints of the final track on the engine. */
  void setMatchedWaypoints(List<MatchedWaypoint> waypoints);

  /** Set the engine start wall-clock (planner saves and restores around a leg). */
  void setStartTime(long startTimeMillis);

  /** Set the engine run budget (planner saves and restores around a leg). */
  void setMaxRunningTime(long maxRunningTimeMillis);

  /** Wall-clock bound for the next isochrone expansion; 0 clears it. */
  void setTransientExpansionDeadline(long deadlineMillis);

  /** Pass-1 air-distance cost factor currently active on the engine. */
  double airDistanceCostFactor();

  /** Set the pass-1 air-distance cost factor (planner saves and restores it). */
  void setAirDistanceCostFactor(double factor);

  /** One leg search — the engine's internal findTrack primitive. */
  OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                     OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc);

  /** Re-run a raw track at full detail between its endpoints. */
  OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                            OsmTrack refTrack);

  /** Profile-aware road snap for a single point. */
  MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist);

  /** Reset the engine's node cache. */
  void resetCache(boolean detailed);

  /**
   * One leg search with the engine's live guide track suspended — a local
   * repair search must not be steered by the track it is repairing.
   */
  OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp);

  /** Batch waypoint matching through the engine's node cache + island pairs. */
  void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance);

  /** Start-centered isochrone expansion (see {@code RoutingEngine#runIsochroneExpansion}). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                 OsmTrack refTrack, boolean includeCandidateTracks);
}
