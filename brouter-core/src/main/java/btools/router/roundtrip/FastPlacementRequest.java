package btools.router.roundtrip;

import btools.router.OsmNodeNamed;

/**
 * Input to the optimized FAST waypoint placement. The caller resolves every knob
 * (variety-seed jitter already applied, {@code roundtrip.fast.*} properties
 * already read), so placement is a pure function of this request plus the road
 * network.
 */
public final class FastPlacementRequest {

  /** The loop's start (and closing) waypoint. */
  final OsmNodeNamed start;

  /** Effective search radius in meters (post-variety-seed). */
  final double searchRadius;

  /** Effective requested direction in degrees, or {@code < 0} for none. */
  final double direction;

  /** Target waypoint count for the loop (start included). */
  final int targetPoints;

  /** Directional-lobe mode: on when the caller supplied a start bearing. */
  final boolean directional;

  /** Via cap for the directional lobe ({@code roundtrip.fast.maxvias}, resolved). */
  final int lobeViaCap;

  public FastPlacementRequest(OsmNodeNamed start, double searchRadius, double direction,
                       int targetPoints, boolean directional, int lobeViaCap) {
    this.start = start;
    this.searchRadius = searchRadius;
    this.direction = direction;
    this.targetPoints = targetPoints;
    this.directional = directional;
    this.lobeViaCap = lobeViaCap;
  }
}
