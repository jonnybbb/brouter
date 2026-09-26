package btools.router.roundtrip;

import java.util.Locale;

import btools.router.OsmPathElement;
import btools.util.CheapRuler;

/**
 * Authoritative ship predicate for candidate evaluation (§4.5).
 */
public final class ShipPredicate {

  /** Evaluation result with accept/reject boolean and diagnostic message. */
  public static final class Result {
    private final boolean accepted;
    private final String reason;

    public Result(boolean accepted, String reason) {
      this.accepted = accepted;
      this.reason = reason;
    }

    public boolean isAccepted() {
      return accepted;
    }

    public String getReason() {
      return reason;
    }
  }

  private ShipPredicate() {
  }

  /**
   * Evaluate whether candidate meets all ship predicate criteria over baseline.
   */
  public static Result evaluate(FinishedCandidate candidate, FinishedCandidate baseline,
                                RefineConfig config, double requestedDistance) {
    if (candidate == null || candidate.getOutcome() != FinalizationOutcome.SUCCESS) {
      return new Result(false, candidate != null ? candidate.getReason() : "candidate_null");
    }
    if (baseline == null || baseline.getTrack() == null) {
      return new Result(false, "baseline_null");
    }

    // 1. Gate passes with full request context
    if (candidate.getQualityVerdict() == null || !candidate.getQualityVerdict().isAccepted()) {
      return new Result(false, "gate_rejected");
    }

    // 2. Symmetric pricing method check (§4.3, ADR-0004)
    String baseMethod = baseline.getPricingMethod();
    String candMethod = candidate.getPricingMethod();
    if (baseMethod == null || candMethod == null
        || !"continuous".equals(baseMethod) || !"continuous".equals(candMethod)
        || !baseMethod.equals(candMethod)) {
      return new Result(false, String.format(Locale.US, "pricing_method_mismatch: ref=%s != base=%s",
        candMethod, baseMethod));
    }

    // 3. Oracle cost/m: C_ref <= (1 - costMargin) * C_base
    double costThreshold = (1.0 - config.costMargin) * baseline.getOracleCostPerMeter();
    if (!Double.isFinite(candidate.getOracleCostPerMeter()) || !Double.isFinite(costThreshold)
        || candidate.getOracleCostPerMeter() <= 0 || costThreshold <= 0
        || candidate.getOracleCostPerMeter() > costThreshold) {
      return new Result(false, String.format(Locale.US, "cost_not_improved: ref=%.4f > thr=%.4f (base=%.4f)",
        candidate.getOracleCostPerMeter(), costThreshold, baseline.getOracleCostPerMeter()));
    }

    // 3. Length rule: |L_ref/L_req - 1| <= |L_base/L_req - 1|
    double targetDist = requestedDistance > 0 ? requestedDistance : baseline.getTrack().distance;
    if (targetDist > 0) {
      double refLenErr = Math.abs(candidate.getTrack().distance / targetDist - 1.0);
      double baseLenErr = Math.abs(baseline.getTrack().distance / targetDist - 1.0);
      if (Double.isNaN(refLenErr) || Double.isNaN(baseLenErr) || refLenErr > baseLenErr) {
        return new Result(false, String.format(Locale.US, "length_error_worse: refErr=%.4f > baseErr=%.4f",
          refLenErr, baseLenErr));
      }
    }

    // 4. Self-crossings <= baseline
    int refCrossings = (candidate.getQualityVerdict() != null && candidate.getQualityVerdict().getSelfIntersections() >= 0)
        ? candidate.getQualityVerdict().getSelfIntersections()
        : RoundTripQualityGate.countSelfIntersections(candidate.getTrack());
    int baseCrossings = (baseline.getQualityVerdict() != null && baseline.getQualityVerdict().getSelfIntersections() >= 0)
        ? baseline.getQualityVerdict().getSelfIntersections()
        : RoundTripQualityGate.countSelfIntersections(baseline.getTrack());
    if (refCrossings > baseCrossings) {
      return new Result(false, String.format(Locale.US, "crossings_worse: ref=%d > base=%d",
        refCrossings, baseCrossings));
    }

    // 5. Scatter reuse <= baseline
    int[] refStemSplit = (candidate.getTrack() != null && candidate.getTrack().nodes != null)
        ? LoopQualityMetrics.reuseStemSplit(candidate.getTrack().nodes) : null;
    int[] baseStemSplit = (baseline.getTrack() != null && baseline.getTrack().nodes != null)
        ? LoopQualityMetrics.reuseStemSplit(baseline.getTrack().nodes) : null;
    int refScatter = (refStemSplit != null && refStemSplit.length > 1) ? refStemSplit[1] : 0;
    int baseScatter = (baseStemSplit != null && baseStemSplit.length > 1) ? baseStemSplit[1] : 0;
    if (refScatter > baseScatter) {
      return new Result(false, String.format(Locale.US, "scatter_reuse_worse: ref=%dm > base=%dm",
        refScatter, baseScatter));
    }

    // 6. Direction rule (§4.4)
    Result dirResult = checkDirection(candidate, baseline, config);
    if (!dirResult.isAccepted()) {
      return dirResult;
    }

    // 7. RCS guard: RCS_ref >= RCS_base - 0.02
    double rcsThreshold = baseline.getRcs() - config.rcsGuard;
    if (Double.isNaN(candidate.getRcs()) || Double.isNaN(rcsThreshold)
        || candidate.getRcs() < rcsThreshold) {
      return new Result(false, String.format(Locale.US, "rcs_degraded: ref=%.4f < thr=%.4f (base=%.4f)",
        candidate.getRcs(), rcsThreshold, baseline.getRcs()));
    }

    return new Result(true, "accepted");
  }

  private static Result checkDirection(FinishedCandidate candidate, FinishedCandidate baseline, RefineConfig config) {
    OsmPathElement baseStart = baseline.getTrack().nodes.get(0);
    int baseMaxDist = -1;
    int baseBestIdx = 0;
    for (int i = 0; i < baseline.getTrack().nodes.size(); i++) {
      int d = baseStart.calcDistance(baseline.getTrack().nodes.get(i));
      if (d > baseMaxDist) {
        baseMaxDist = d;
        baseBestIdx = i;
      }
    }

    // Waived if baseline's farthest point is under 500m away
    if (baseMaxDist < config.degenerateDirectionRadius) {
      return new Result(true, "direction_check_waived_farthest_under_500m");
    }

    OsmPathElement candStart = candidate.getTrack().nodes.get(0);
    int candMaxDist = -1;
    int candBestIdx = 0;
    for (int i = 0; i < candidate.getTrack().nodes.size(); i++) {
      int d = candStart.calcDistance(candidate.getTrack().nodes.get(i));
      if (d > candMaxDist) {
        candMaxDist = d;
        candBestIdx = i;
      }
    }

    double baseBearing = CheapRuler.getScaledBearing(baseStart.getILon(), baseStart.getILat(),
      baseline.getTrack().nodes.get(baseBestIdx).getILon(), baseline.getTrack().nodes.get(baseBestIdx).getILat());
    double candBearing = CheapRuler.getScaledBearing(candStart.getILon(), candStart.getILat(),
      candidate.getTrack().nodes.get(candBestIdx).getILon(), candidate.getTrack().nodes.get(candBestIdx).getILat());

    double diff = (candBearing - baseBearing + 540.0) % 360.0 - 180.0;
    if (Math.abs(diff) > config.maxDirectionDeltaDegrees) {
      return new Result(false, String.format(Locale.US, "direction_delta_exceeded: delta=%.1f deg > max=%.1f deg",
        Math.abs(diff), config.maxDirectionDeltaDegrees));
    }

    return new Result(true, "direction_ok");
  }
}
