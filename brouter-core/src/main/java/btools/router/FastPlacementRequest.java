package btools.router;

/**
 * Input to the optimized FAST waypoint placement (see
 * {@code .agents/fast-waypoint-planner-spec.md}). The caller resolves every
 * knob — variety-seed jitter is already applied to radius/direction, and the
 * {@code roundtrip.fast.*} system properties are already read — so placement
 * is a pure function of this request plus the road network.
 */
final class FastPlacementRequest {

  /** The loop's start (and closing) waypoint. */
  final OsmNodeNamed start;

  /** Effective search radius in meters (post-variety-seed). */
  final double searchRadius;

  /** Effective requested direction in degrees, or {@code < 0} for none. */
  final double direction;

  /** Target waypoint count for the loop (start included). */
  final int targetPoints;

  /** Directional-lobe mode ({@code roundtrip.fast.directional}, resolved). */
  final boolean directional;

  /** Via cap for the directional lobe ({@code roundtrip.fast.maxvias}, resolved). */
  final int lobeViaCap;

  FastPlacementRequest(OsmNodeNamed start, double searchRadius, double direction,
                       int targetPoints, boolean directional, int lobeViaCap) {
    this.start = start;
    this.searchRadius = searchRadius;
    this.direction = direction;
    this.targetPoints = targetPoints;
    this.directional = directional;
    this.lobeViaCap = lobeViaCap;
  }
}
