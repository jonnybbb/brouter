package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;

/**
 * The leg-search slice of the engine seam: single-leg searches, waypoint
 * matching, isochrone expansion, and the engine kill switch. One of the four
 * role interfaces composed by {@link RoundTripEngineOps}.
 */
public interface LegRouter {

  /** One leg search — the engine's internal findTrack primitive. */
  OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                     OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc);

  /**
   * One leg search with the engine's live guide track suspended — a local
   * repair search must not be steered by the track it is repairing.
   */
  OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp);

  /** Re-run a raw track at full detail between its endpoints. */
  OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                            OsmTrack refTrack);

  /** Profile-aware road snap for a single point. */
  MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist);

  /** Batch waypoint matching through the engine's node cache + island pairs. */
  void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance);

  /** Reset the engine's node cache. */
  void resetCache(boolean detailed);

  /** Start-centered isochrone expansion, placement variant (2-arg overload). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius);

  /** Start-centered isochrone expansion (see {@code RoutingEngine#runIsochroneExpansion}). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                 OsmTrack refTrack, boolean includeCandidateTracks);

  /** Terminate the engine's current search (volatile kill flag). */
  void terminate();

  /** True when the engine was terminated (watchdog/cancel). */
  boolean isTerminated();
}
