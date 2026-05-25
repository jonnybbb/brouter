package btools.router;

import java.util.Collections;
import java.util.List;

/**
 * Result of {@link RoutingEngine#runIsochroneExpansion}.
 *
 * <p>Holds both the per-bucket frontier table (used by the legacy {@link
 * RoutingEngine#placeWaypointsFromIsochrone} debug-only path) and the road-native
 * candidate pool (used by ISO_GREEDY via {@link IsochroneCandidateProvider}).
 * Returning both as one value keeps callers from reading stale data via a
 * side-channel field.
 */
final class IsochroneExpansionResult {

  /** Per-bucket frontier: {@code [direction_deg, airDist_m, cost, hits]}. */
  final double[][] frontier;

  /** Road-native candidates extracted from intermediate cost contours + the frontier max. */
  final List<IsoCandidate> candidates;

  IsochroneExpansionResult(double[][] frontier, List<IsoCandidate> candidates) {
    this.frontier = frontier;
    this.candidates = (candidates != null) ? candidates : Collections.emptyList();
  }
}
