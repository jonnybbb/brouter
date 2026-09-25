package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;

/**
 * Initializes the refinement stage by re-routing the n raw legs without refTrack (§4.2, M0.4).
 * Rebuilds the loop and measures execution time, links processed, and fidelity vs baseline.
 */
public final class RefineInitializer {

  private RefineInitializer() {
  }

  /**
   * Extract RefineSkeleton from track or matched waypoints list.
   */
  public static RefineSkeleton extractSkeleton(OsmTrack track, List<MatchedWaypoint> mwps) {
    List<MatchedWaypoint> list = (track != null && track.getMatchedWaypoints() != null)
        ? track.getMatchedWaypoints()
        : mwps;
    if (list == null || list.size() < 3) {
      return null;
    }
    return new RefineSkeleton(list.get(0), list.subList(1, list.size() - 1), list.get(list.size() - 1));
  }

  /**
   * Re-routes the n legs of the skeleton and builds the initial refined state.
   */
  public static RefineInitResult initialize(
      RoundTripEngineOps ops,
      LegEvaluator evaluator,
      RefineSkeleton skeleton,
      RoundTripTrackCleanup cleanup,
      OsmTrack baselineTrack,
      double searchRadius,
      String profileName,
      double requestedDirection,
      double requestedDistance,
      long deadlineMs) {
    return initialize(ops, evaluator, skeleton, cleanup, baselineTrack, searchRadius,
      profileName, requestedDirection, requestedDistance, deadlineMs, false);
  }

  /**
   * Re-routes the n legs of the skeleton and builds the initial refined state.
   */
  public static RefineInitResult initialize(
      RoundTripEngineOps ops,
      LegEvaluator evaluator,
      RefineSkeleton skeleton,
      RoundTripTrackCleanup cleanup,
      OsmTrack baselineTrack,
      double searchRadius,
      String profileName,
      double requestedDirection,
      double requestedDistance,
      long deadlineMs,
      boolean finalizeRebuilt) {

    long startMs = System.currentTimeMillis();
    int startLinks = ops != null ? ops.getLinksProcessed() : 0;

    if (skeleton == null || skeleton.getWaypoints() == null || skeleton.getWaypoints().size() < 2) {
      return RefineInitResult.failure("too_few_waypoints", 0, 0);
    }

    List<MatchedWaypoint> waypoints = skeleton.getWaypoints();
    int numLegs = waypoints.size() - 1;
    List<OsmTrack> rawLegs = new ArrayList<>(numLegs);
    LegCache legCache = new LegCache();

    for (int i = 0; i < numLegs; i++) {
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return RefineInitResult.failure("init_timeout_before_leg_" + i,
          System.currentTimeMillis() - startMs,
          ops != null ? ops.getLinksProcessed() - startLinks : 0);
      }
      MatchedWaypoint from = waypoints.get(i);
      MatchedWaypoint to = waypoints.get(i + 1);

      long remaining = deadlineMs > 0 ? (deadlineMs - System.currentTimeMillis()) : 10000L;
      OsmTrack leg = evaluator.route(from, to, remaining);
      if (leg == null || leg.nodes == null || leg.nodes.size() < 2) {
        return RefineInitResult.failure("init_leg_failed_" + i,
          System.currentTimeMillis() - startMs,
          ops != null ? ops.getLinksProcessed() - startLinks : 0);
      }
      legCache.put(from, to, leg);
      rawLegs.add(leg);
    }

    long elapsedMs = System.currentTimeMillis() - startMs;
    int linksProcessed = ops != null ? ops.getLinksProcessed() - startLinks : 0;

    // Price baseline track
    double baselineCostPerMeter = -1.0;
    int baselineCrossings = -1;
    int baselineScatterReuse = -1;
    double baselineDistance = -1.0;

    if (baselineTrack != null && baselineTrack.nodes != null && !baselineTrack.nodes.isEmpty()) {
      baselineDistance = baselineTrack.distance;
      baselineCostPerMeter = LoopCostOracle.price(ops, baselineTrack, waypoints);
      baselineCrossings = RoundTripQualityGate.countSelfIntersections(baselineTrack);
      int[] stemSplit = LoopQualityMetrics.reuseStemSplit(baselineTrack.nodes);
      baselineScatterReuse = (stemSplit != null && stemSplit.length > 1) ? stemSplit[1] : 0;
    }

    // Finalize rebuilt loop from raw legs only when explicitly requested (e.g. M0.4 tests)
    FinishedCandidate rebuilt = null;
    double rebuiltCostPerMeter = -1.0;
    int rebuiltCrossings = -1;
    int rebuiltScatterReuse = -1;
    double rebuiltDistance = -1.0;

    if (finalizeRebuilt) {
      List<OsmTrack> copyOfLegs = new ArrayList<>(rawLegs.size());
      for (OsmTrack l : rawLegs) {
        copyOfLegs.add(LegCache.copyTrack(l));
      }

      rebuilt = RefineFinalizer.finalizeCandidate(
        copyOfLegs, skeleton, ops, cleanup, searchRadius, profileName,
        requestedDirection, requestedDistance, deadlineMs);

      if (rebuilt.isSuccess() && rebuilt.getTrack() != null) {
        OsmTrack rt = rebuilt.getTrack();
        rebuiltDistance = rt.distance;
        rebuiltCostPerMeter = rebuilt.getOracleCostPerMeter();
        rebuiltCrossings = RoundTripQualityGate.countSelfIntersections(rt);
        int[] stemSplit = LoopQualityMetrics.reuseStemSplit(rt.nodes);
        rebuiltScatterReuse = (stemSplit != null && stemSplit.length > 1) ? stemSplit[1] : 0;
      }
    }

    return new RefineInitResult(
      true, null, rawLegs, legCache, elapsedMs, linksProcessed, rebuilt,
      baselineCostPerMeter, rebuiltCostPerMeter, baselineDistance, rebuiltDistance,
      baselineCrossings, rebuiltCrossings, baselineScatterReuse, rebuiltScatterReuse);
  }
}
