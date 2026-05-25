package btools.router;

/**
 * Per-direction reachability score produced by {@link RoutingEngine#probeReachableDirections}.
 *
 * <p>Multi-probe summary: each compass direction was tested at distances {0.7R, 1.0R, 1.3R};
 * this record carries which fraction of those probes snapped, the best (smallest) snap
 * distance, and the mean matched road distance from the start. The FAST tier uses
 * {@link #successfulProbeCount} to drop one-shot directions when enough strong
 * alternatives exist, so the chosen waypoint ring avoids fragile sea/dead-end picks.
 */
final class ProbeDirection {

  /** Compass bearing of this probe in {@code [0, 360)}. */
  final double direction;
  /** Mean matched road air-distance from start across the successful probes, in meters. */
  final double matchedDistance;
  /** Best (smallest) snap distance among the successful probes, in meters. */
  final double snapDistance;
  /** Number of probe distances where a road snapped within tolerance (1–3). */
  final int successfulProbeCount;
  /** Composite confidence in {@code [0, 1]}; higher = a sector with multiple reachable points. */
  final double confidence;

  ProbeDirection(double direction, double matchedDistance, double snapDistance,
                 int successfulProbeCount, double confidence) {
    this.direction = direction;
    this.matchedDistance = matchedDistance;
    this.snapDistance = snapDistance;
    this.successfulProbeCount = successfulProbeCount;
    this.confidence = confidence;
  }
}
