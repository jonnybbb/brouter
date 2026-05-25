package btools.router;

import btools.mapaccess.OsmPos;
import btools.util.CheapRuler;

/**
 * A road-native candidate node extracted from a {@link RoutingEngine#runIsochroneExpansion}
 * Dijkstra. Each candidate represents the farthest air-distance node Dijkstra reached in
 * a particular angular bucket within a particular cost contour from the start point.
 *
 * <p>Used by {@link IsochroneCandidateProvider} as the QUALITY-tier candidate pool —
 * a road-on-the-ground replacement for the radial provider's geometric ring.
 *
 * <p>Implements {@link OsmPos} so it interoperates with the engine's distance/identity
 * helpers (e.g., {@code mwp.calcDistance(candidate)} and {@code getIdFromPos()}).
 */
final class IsoCandidate implements OsmPos {

  /** Longitude (1e6 ilon units). */
  final int ilon;
  /** Latitude (1e6 ilat units). */
  final int ilat;
  /** Compass bearing from the loop start to this candidate, in {@code [0, 360)}. */
  final double bearingFromStart;
  /** Air distance from the loop start in meters. */
  final double airDistanceFromStart;
  /** Routing cost from the loop start in cost units (≈ meters × profile costfactor). */
  final int costFromStart;
  /** Angular bucket index (0-35 for the default 10°-wide buckets). */
  final int bucket;
  /** Population count of this bucket — confidence signal. {@code <3} ≈ one-shot dead-end. */
  final int bucketHits;
  /**
   * Source contour the candidate came from (25, 50, 75 = cost-budget percentage;
   * 100 = frontier-max). Used by the filter pipeline to prefer farthest candidates
   * within each bucket.
   */
  final int sourceContour;

  IsoCandidate(int ilon, int ilat, double bearingFromStart, double airDistanceFromStart,
               int costFromStart, int bucket, int bucketHits, int sourceContour) {
    this.ilon = ilon;
    this.ilat = ilat;
    this.bearingFromStart = bearingFromStart;
    this.airDistanceFromStart = airDistanceFromStart;
    this.costFromStart = costFromStart;
    this.bucket = bucket;
    this.bucketHits = bucketHits;
    this.sourceContour = sourceContour;
  }

  @Override
  public int getILon() {
    return ilon;
  }

  @Override
  public int getILat() {
    return ilat;
  }

  /** Elevation is not tracked during isochrone expansion. */
  @Override
  public short getSElev() {
    return Short.MIN_VALUE;
  }

  @Override
  public double getElev() {
    return Short.MIN_VALUE / 4.0;
  }

  @Override
  public int calcDistance(OsmPos p) {
    return (int) (CheapRuler.distance(ilon, ilat, p.getILon(), p.getILat()) + 0.5);
  }

  @Override
  public long getIdFromPos() {
    return ((long) ilon) << 32 | (ilat & 0xFFFFFFFFL);
  }
}
