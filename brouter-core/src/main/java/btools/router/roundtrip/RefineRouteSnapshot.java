package btools.router.roundtrip;

import btools.router.OsmTrack;
import btools.router.RouteRideAnalysis;

/** Immutable measurements captured at the refinement hook, before decoration or another request. */
public final class RefineRouteSnapshot {
  public final RouteRideAnalysis ride;
  public final java.util.List<Point> geometry;

  public static final class Point {
    public final int ilon;
    public final int ilat;
    public final short elevation;

    private Point(btools.router.OsmPathElement node) {
      ilon = node.getILon();
      ilat = node.getILat();
      elevation = node.getSElev();
    }
  }
  public final int distance;
  public final int historicalCost;
  public final int crossings;
  public final int scatterReuse;
  public final double rcs;
  public final boolean gateAccepted;
  public final long geometrySignature;

  private RefineRouteSnapshot(OsmTrack track, double requestedDistance, String profile,
                              double direction, RoundTripQualityResult quality) {
    ride = RouteRideAnalysis.measure(track, requestedDistance);
    java.util.List<Point> points = new java.util.ArrayList<>(track.nodes.size());
    for (btools.router.OsmPathElement node : track.nodes) points.add(new Point(node));
    geometry = java.util.Collections.unmodifiableList(points);
    distance = track.distance;
    historicalCost = track.cost;
    crossings = RoundTripQualityGate.countSelfIntersections(track);
    scatterReuse = LoopQualityMetrics.reuseStemSplit(track.nodes)[1];
    rcs = RouteChoiceScore.score(track, requestedDistance, profile, quality, direction).score();
    gateAccepted = quality != null && quality.isAccepted();
    geometrySignature = LoopCostOracle.geometrySignature(track);
  }

  public static RefineRouteSnapshot capture(OsmTrack track, double requestedDistance, String profile,
                                             double direction, RoundTripQualityResult quality) {
    return track == null || track.nodes == null || track.nodes.isEmpty() ? null
      : new RefineRouteSnapshot(track, requestedDistance, profile, direction, quality);
  }
}
