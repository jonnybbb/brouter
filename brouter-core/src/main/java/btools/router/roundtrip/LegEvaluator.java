package btools.router.roundtrip;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;

/**
 * Interface for routing a single leg between two waypoints (§4.7).
 * Decouples the leg router implementation from the search and finalization stages.
 */
public interface LegEvaluator {

  /**
   * Route a raw leg between {@code from} and {@code to} without refTrack.
   *
   * @param from starting matched waypoint
   * @param to ending matched waypoint
   * @param timeoutMs maximum time or deadline in milliseconds
   * @return routed raw OsmTrack, or null if routing failed or timed out
   */
  OsmTrack route(MatchedWaypoint from, MatchedWaypoint to, long timeoutMs);
}
