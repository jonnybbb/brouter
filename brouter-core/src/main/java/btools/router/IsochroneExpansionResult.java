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

  /**
   * Per-bucket frontier table. Each entry is {@code [direction_deg, airDist_m,
   * cost, hits, ilon, ilat]} — the first four fields are the legacy contract
   * (existing readers keep working); the trailing {@code ilon}/{@code ilat}
   * expose the road-native coordinate of the selected frontier node so direct
   * ISOCHRONE placement can avoid synthesizing waypoint positions via
   * {@link btools.util.CheapRuler#destination CheapRuler.destination}.
   *
   * <p>Probe-only entries injected by {@link RoutingEngine#mergeIsochroneWithProbe}
   * remain 4-element (no road-native data available); callers must guard with
   * {@code entry.length >= 6} before reading {@code ilon}/{@code ilat}.
   */
  final double[][] frontier;

  /** Road-native candidates extracted from intermediate cost contours + the frontier max. */
  final List<IsoCandidate> candidates;

  IsochroneExpansionResult(double[][] frontier, List<IsoCandidate> candidates) {
    this.frontier = frontier;
    this.candidates = (candidates != null) ? candidates : Collections.emptyList();
  }
}
