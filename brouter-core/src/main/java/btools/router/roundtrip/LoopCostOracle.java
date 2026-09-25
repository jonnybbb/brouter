package btools.router.roundtrip;

import java.util.List;
import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;

/**
 * LoopCostOracle re-prices an exact node sequence under the active profile.
 * It provides continuous loop pricing across vias (no turn or elevation resets),
 * handles clipped endpoints, and applies no refTrack penalties.
 */
public final class LoopCostOracle {

  private LoopCostOracle() {
  }

  /**
   * Prices a closed loop composed of consecutive legs between waypoints,
   * returning cost per meter (cost/m).
   */
  public static double price(LegRouter router, List<OsmTrack> legs, List<MatchedWaypoint> waypoints) {
    if (router == null || legs == null || legs.isEmpty() || waypoints == null || waypoints.size() < 2) {
      return -1.0;
    }
    int totalCost = router.walkLoopCost(legs, waypoints);
    if (totalCost < 0) {
      return -1.0;
    }
    int totalDistance = 0;
    for (int i = 0; i < legs.size(); i++) {
      OsmTrack leg = legs.get(i);
      if (leg != null) {
        totalDistance += leg.distance;
      }
    }
    if (totalDistance <= 0) {
      return -1.0;
    }
    return (double) totalCost / totalDistance;
  }

  /**
   * Prices a closed loop composed of consecutive legs between waypoints,
   * returning total continuous walked cost.
   */
  public static int priceCost(LegRouter router, List<OsmTrack> legs, List<MatchedWaypoint> waypoints) {
    if (router == null || legs == null || legs.isEmpty() || waypoints == null || waypoints.size() < 2) {
      return -1;
    }
    return router.walkLoopCost(legs, waypoints);
  }

  /**
   * Prices a single leg or track, returning cost per meter (cost/m).
   */
  public static double price(LegRouter router, OsmTrack track, MatchedWaypoint startWp, MatchedWaypoint endWp) {
    if (router == null || track == null || startWp == null || endWp == null || track.distance <= 0) {
      return -1.0;
    }
    int cost = router.walkPathCost(track, startWp, endWp);
    if (cost < 0) {
      return -1.0;
    }
    return (double) cost / track.distance;
  }

  /**
   * Prices a single leg or track, returning exact walked cost.
   */
  public static int priceCost(LegRouter router, OsmTrack track, MatchedWaypoint startWp, MatchedWaypoint endWp) {
    if (router == null || track == null || startWp == null || endWp == null) {
      return -1;
    }
    return router.walkPathCost(track, startWp, endWp);
  }

  /**
   * Prices a full loop track given its waypoint list (from start to end).
   */
  public static double price(LegRouter router, OsmTrack track, List<MatchedWaypoint> waypoints) {
    if (waypoints == null || waypoints.size() < 2) {
      return -1.0;
    }
    return price(router, track, waypoints.get(0), waypoints.get(waypoints.size() - 1));
  }

  /**
   * Prices a full loop track given its waypoint list, returning exact walked cost.
   */
  public static int priceCost(LegRouter router, OsmTrack track, List<MatchedWaypoint> waypoints) {
    if (waypoints == null || waypoints.size() < 2) {
      return -1;
    }
    return priceCost(router, track, waypoints.get(0), waypoints.get(waypoints.size() - 1));
  }
}
