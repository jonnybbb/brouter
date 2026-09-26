package btools.router.roundtrip;

import java.util.Collections;
import java.util.List;

import btools.router.OsmTrack;

/**
 * Result of the refinement initialization stage (§4.2, §4.7, M0.4).
 */
public final class RefineInitResult {

  private final boolean success;
  private final String failureReason;
  private final List<OsmTrack> rawLegs;
  private final LegCache legCache;
  private final long elapsedMs;
  private final int linksProcessed;
  private final FinishedCandidate rebuiltCandidate;

  // Comparison metrics vs baseline
  private final double baselineOracleCostPerMeter;
  private final double rebuiltOracleCostPerMeter;
  private final double baselineDistance;
  private final double rebuiltDistance;
  private final int baselineCrossings;
  private final int rebuiltCrossings;
  private final int baselineScatterReuse;
  private final int rebuiltScatterReuse;
  private final String baselinePricingMethod;

  public RefineInitResult(
      boolean success,
      String failureReason,
      List<OsmTrack> rawLegs,
      LegCache legCache,
      long elapsedMs,
      int linksProcessed,
      FinishedCandidate rebuiltCandidate,
      double baselineOracleCostPerMeter,
      double rebuiltOracleCostPerMeter,
      double baselineDistance,
      double rebuiltDistance,
      int baselineCrossings,
      int rebuiltCrossings,
      int baselineScatterReuse,
      int rebuiltScatterReuse) {
    this(success, failureReason, rawLegs, legCache, elapsedMs, linksProcessed, rebuiltCandidate,
      baselineOracleCostPerMeter, rebuiltOracleCostPerMeter, baselineDistance, rebuiltDistance,
      baselineCrossings, rebuiltCrossings, baselineScatterReuse, rebuiltScatterReuse, "unknown");
  }

  public RefineInitResult(
      boolean success,
      String failureReason,
      List<OsmTrack> rawLegs,
      LegCache legCache,
      long elapsedMs,
      int linksProcessed,
      FinishedCandidate rebuiltCandidate,
      double baselineOracleCostPerMeter,
      double rebuiltOracleCostPerMeter,
      double baselineDistance,
      double rebuiltDistance,
      int baselineCrossings,
      int rebuiltCrossings,
      int baselineScatterReuse,
      int rebuiltScatterReuse,
      String baselinePricingMethod) {
    this.success = success;
    this.failureReason = failureReason;
    this.rawLegs = rawLegs != null ? rawLegs : Collections.<OsmTrack>emptyList();
    this.legCache = legCache;
    this.elapsedMs = elapsedMs;
    this.linksProcessed = linksProcessed;
    this.rebuiltCandidate = rebuiltCandidate;
    this.baselineOracleCostPerMeter = baselineOracleCostPerMeter;
    this.rebuiltOracleCostPerMeter = rebuiltOracleCostPerMeter;
    this.baselineDistance = baselineDistance;
    this.rebuiltDistance = rebuiltDistance;
    this.baselineCrossings = baselineCrossings;
    this.rebuiltCrossings = rebuiltCrossings;
    this.baselineScatterReuse = baselineScatterReuse;
    this.rebuiltScatterReuse = rebuiltScatterReuse;
    this.baselinePricingMethod = baselinePricingMethod != null ? baselinePricingMethod : "unknown";
  }

  public static RefineInitResult failure(String reason, long elapsedMs, int linksProcessed) {
    return new RefineInitResult(false, reason, null, null, elapsedMs, linksProcessed, null,
      -1.0, -1.0, -1.0, -1.0, -1, -1, -1, -1, "none");
  }

  public boolean isSuccess() {
    return success;
  }

  public String getFailureReason() {
    return failureReason;
  }

  public List<OsmTrack> getRawLegs() {
    return rawLegs;
  }

  public LegCache getLegCache() {
    return legCache;
  }

  public long getElapsedMs() {
    return elapsedMs;
  }

  public int getLinksProcessed() {
    return linksProcessed;
  }

  public FinishedCandidate getRebuiltCandidate() {
    return rebuiltCandidate;
  }

  public double getBaselineOracleCostPerMeter() {
    return baselineOracleCostPerMeter;
  }

  public double getRebuiltOracleCostPerMeter() {
    return rebuiltOracleCostPerMeter;
  }

  public double getBaselineDistance() {
    return baselineDistance;
  }

  public double getRebuiltDistance() {
    return rebuiltDistance;
  }

  public int getBaselineCrossings() {
    return baselineCrossings;
  }

  public int getRebuiltCrossings() {
    return rebuiltCrossings;
  }

  public int getBaselineScatterReuse() {
    return baselineScatterReuse;
  }

  public int getRebuiltScatterReuse() {
    return rebuiltScatterReuse;
  }

  public String getBaselinePricingMethod() {
    return baselinePricingMethod;
  }
}
