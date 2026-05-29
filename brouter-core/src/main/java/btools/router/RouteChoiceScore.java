package btools.router;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Soft ranking score for choosing among accepted round-trip candidates.
 *
 * <p>This is a deliberately separate concern from {@link RoundTripQualityGate}.
 * The gate makes the hard accept/reject decision (beelines, broken closure,
 * profile-hostile surfaces, accidental retraces). Any candidate that fails
 * the gate is excluded from scoring entirely. Among candidates that DO pass,
 * the score ranks them — it does not gate.
 *
 * <p>Design constraints (see docs/features/roundtrip-auto-quality-redesign.md):
 * <ul>
 *   <li>numeric score + a reason breakdown the caller can surface for
 *       debugging or explanation;</li>
 *   <li>hard-coded production-neutral preferred bands, no dependency on
 *       {@code LoopTestRegion} or benchmark constants;</li>
 *   <li>direction is a weak factor only — it cannot dominate severe reuse,
 *       closure, or distance issues;</li>
 *   <li>profile-specific cost/m is a soft preference;</li>
 *   <li>scores the final {@link OsmTrack} only, not pre-final planner state;</li>
 *   <li>penalises route-shape disclosures that indicate surprising
 *       behavior (LOLLIPOP and SCENIC_OUT_AND_BACK are still acceptable
 *       but rank below clean STRICT_LOOP all else equal).</li>
 * </ul>
 *
 * <p>The score is in {@code [0, 1]}. Higher is better. AUTO should use
 * {@link #score(OsmTrack, double, String, RoundTripQualityResult, double)} so
 * the requested direction participates as the weak direction factor; the
 * four-argument overload is retained for callers that do not have a requested
 * direction. The returned {@link Verdict} carries both the numeric score and
 * the per-component contributions.
 */
public final class RouteChoiceScore {

  // ---- Component weights -------------------------------------------------
  // Sum to 1.0 across the positive contributions. Direction is intentionally
  // small (5%) so it cannot dominate. The shape-penalty is subtracted on top
  // of the positive sum, so a clean STRICT_LOOP can score above 1.0 in raw
  // terms — clamped at the end.

  /** Distance closeness to requested. */
  static final double W_DISTANCE   = 0.25;
  /** Road reuse / retrace penalty (low reuse = high score). */
  static final double W_REUSE      = 0.20;
  /** Closure quality (small closure gap = high score). */
  static final double W_CLOSURE    = 0.10;
  /** Continuity / max-gap (no synthetic beelines + small max-gap). */
  static final double W_CONTINUITY = 0.15;
  /** Compactness within reasonable range. */
  static final double W_COMPACTNESS = 0.10;
  /** Profile-specific cost/m soft preference. */
  static final double W_COSTM      = 0.15;
  /** Direction delta — weak; capped so it cannot dominate. */
  static final double W_DIRECTION  = 0.05;
  /** Shape disclosure penalty: LOLLIPOP/SCENIC down-weight vs STRICT_LOOP. */
  static final double SHAPE_PENALTY_LOLLIPOP = 0.05;
  static final double SHAPE_PENALTY_SCENIC   = 0.15;

  // ---- Profile-typical cost-per-meter bands ------------------------------
  // Used to compute the cost/m component. A route on roads the profile
  // actively prefers (cost/m at or below the lower bound) scores 1.0; above
  // the upper bound scores 0; linearly interpolated in between.
  //
  // These values mirror the existing {@link LoopQualityMetrics#computeCostMatchScore}
  // bands but per-profile, because gravel/MTB have higher cost-per-meter
  // baselines on their preferred surfaces (a 1.5 cost/m gravel route may
  // be on perfect terrain; the same value on fastbike means rough roads).

  static double[] costMBand(String profileName) {
    if (profileName == null) return new double[]{1.5, 4.0};
    String n = profileName.toLowerCase(Locale.ROOT);
    if (n.contains("fastbike") || n.contains("road") || n.contains("racing")) {
      return new double[]{1.2, 3.0};   // tight band: paved-friendly profiles
    }
    if (n.contains("gravel")) {
      return new double[]{2.0, 5.0};
    }
    if (n.contains("mtb")) {
      return new double[]{4.0, 9.0};
    }
    if (n.contains("trekking")) {
      return new double[]{1.5, 4.0};
    }
    return new double[]{1.5, 4.0};     // default — matches LoopQualityMetrics
  }

  // ---- Verdict -----------------------------------------------------------

  /** A single component contribution. */
  public static final class Reason {
    public final String label;
    public final double weight;
    public final double rawValue;       // the underlying metric value
    public final double scoreContribution; // weight * normalised, in [-1, weight]

    Reason(String label, double weight, double rawValue, double scoreContribution) {
      this.label = label;
      this.weight = weight;
      this.rawValue = rawValue;
      this.scoreContribution = scoreContribution;
    }

    @Override
    public String toString() {
      return String.format(Locale.US, "%s (raw=%.3f, weight=%.2f, +%.3f)",
        label, rawValue, weight, scoreContribution);
    }
  }

  /** Result of scoring one candidate. */
  public static final class Verdict {
    private final double score;
    private final List<Reason> reasons;

    Verdict(double score, List<Reason> reasons) {
      this.score = score;
      this.reasons = Collections.unmodifiableList(new ArrayList<>(reasons));
    }

    public double score() { return score; }
    public List<Reason> reasons() { return reasons; }

    /** Multi-line human-readable breakdown. Suitable for logging. */
    public String describe() {
      StringBuilder sb = new StringBuilder(256);
      sb.append(String.format(Locale.US, "score=%.3f%n", score));
      sb.append("reasons:%n".replace("%n", "\n"));
      for (Reason r : reasons) {
        sb.append("  ").append(r).append('\n');
      }
      return sb.toString();
    }

    @Override
    public String toString() {
      return String.format(Locale.US, "RouteChoiceScore[%.3f, %d reasons]",
        score, reasons.size());
    }
  }

  private RouteChoiceScore() {}

  // ---- Public API --------------------------------------------------------

  /**
   * Score a candidate. The candidate must already have passed
   * {@link RoundTripQualityGate} ({@code qualityGate.isAccepted() == true});
   * scoring a rejected candidate is meaningless — return a zero-score Verdict.
   *
   * @param track             the final routed track for this candidate
   * @param requestedDistance the loop distance the cyclist requested (meters)
   * @param profileName       the active profile name (used for cost/m bands)
   * @param qualityGate       the gate verdict (used for shape penalty + disclosure
   *                          penalty); may be null in which case shape defaults
   *                          to STRICT_LOOP and no disclosure penalty applies
   */
  public static Verdict score(OsmTrack track, double requestedDistance,
                              String profileName, RoundTripQualityResult qualityGate) {
    return score(track, requestedDistance, profileName, qualityGate, 0);
  }

  public static Verdict score(OsmTrack track, double requestedDistance,
                              String profileName, RoundTripQualityResult qualityGate,
                              double requestedDirection) {
    if (track == null || track.nodes == null || track.nodes.size() < 2) {
      List<Reason> empty = new ArrayList<>();
      empty.add(new Reason("no track", 0, 0, 0));
      return new Verdict(0.0, empty);
    }
    if (qualityGate != null && !qualityGate.isAccepted()) {
      List<Reason> empty = new ArrayList<>();
      empty.add(new Reason("gate rejected: " + qualityGate.getRejectionReason(),
        0, 0, 0));
      return new Verdict(0.0, empty);
    }

    LoopQualityMetrics m = LoopQualityMetrics.compute(track, (int) requestedDistance, requestedDirection);
    List<Reason> reasons = new ArrayList<>(8);
    double total = 0;

    // 1. Distance closeness. score 1.0 when ratio ≈ 1.0, decays linearly to
    //    0.0 at ratio difference ≥ 0.5 (i.e. half or 1.5×). Production-
    //    neutral band.
    double distScore = 1.0 - Math.min(1.0, Math.abs(m.getDistanceRatio() - 1.0) / 0.5);
    double distContrib = W_DISTANCE * distScore;
    reasons.add(new Reason("distance ratio " + fmt(m.getDistanceRatio())
      + " (preferred band [0.5, 1.5])", W_DISTANCE, m.getDistanceRatio(), distContrib));
    total += distContrib;

    // 2. Reuse. 1.0 at 0%, 0.0 at ≥ 50% reuse.
    double reuseScore = 1.0 - Math.min(1.0, m.getRoadReusePercent() / 50.0);
    double reuseContrib = W_REUSE * reuseScore;
    reasons.add(new Reason("road reuse " + fmt(m.getRoadReusePercent()) + "%",
      W_REUSE, m.getRoadReusePercent(), reuseContrib));
    total += reuseContrib;

    // 3. Closure. 1.0 at 0m, 0.0 at ≥ 400m (the existing MAX_CLOSURE_METERS).
    double closureScore = 1.0 - Math.min(1.0,
      m.getClosureDistanceMeters() / (double) RoundTripQualityGate.MAX_CLOSURE_METERS);
    double closureContrib = W_CLOSURE * closureScore;
    reasons.add(new Reason("closure " + m.getClosureDistanceMeters() + "m",
      W_CLOSURE, m.getClosureDistanceMeters(), closureContrib));
    total += closureContrib;

    // 4. Continuity (no synthetic beelines) + maxGap.
    double maxGapScore = 1.0 - Math.min(1.0, m.getMaxGapMeters() / 1500.0);
    double contScore = 0.75 * m.getContinuityScore() + 0.25 * maxGapScore;
    double contContrib = W_CONTINUITY * contScore;
    reasons.add(new Reason("continuity " + fmt(m.getContinuityScore())
      + " (maxGap " + m.getMaxGapMeters() + "m)",
      W_CONTINUITY, m.getContinuityScore(), contContrib));
    total += contContrib;

    // 5. Compactness — convex hull area vs ideal-circle area. Already in [0,1].
    double compactContrib = W_COMPACTNESS * m.getCompactnessScore();
    reasons.add(new Reason("compactness " + fmt(m.getCompactnessScore()),
      W_COMPACTNESS, m.getCompactnessScore(), compactContrib));
    total += compactContrib;

    // 6. cost/m within profile-typical band.
    double[] band = costMBand(profileName);
    double costM = m.getAverageCostPerMeter();
    double costMScore;
    if (costM <= band[0]) costMScore = 1.0;
    else if (costM >= band[1]) costMScore = 0.0;
    else costMScore = (band[1] - costM) / (band[1] - band[0]);
    double costMContrib = W_COSTM * costMScore;
    reasons.add(new Reason("cost/m " + fmt(costM)
      + " (preferred band [" + fmt(band[0]) + ", " + fmt(band[1]) + "])",
      W_COSTM, costM, costMContrib));
    total += costMContrib;

    // 7. Direction delta — weak. Score 1.0 at delta 0°, 0.0 at delta 180°.
    //    Direction is intentionally a small weight so it cannot dominate
    //    other factors.
    double dirScore = 1.0 - Math.min(1.0, m.getDirectionDeltaDegrees() / 180.0);
    double dirContrib = W_DIRECTION * dirScore;
    reasons.add(new Reason("direction delta " + fmt(m.getDirectionDeltaDegrees()) + "°",
      W_DIRECTION, m.getDirectionDeltaDegrees(), dirContrib));
    total += dirContrib;

    // 8. Shape disclosure penalty: cyclist sees the route shape; a clean
    //    STRICT_LOOP ranks above a LOLLIPOP above a SCENIC_OUT_AND_BACK
    //    all else equal. INVALID_RETRACE would have been gate-rejected
    //    above and wouldn't reach here.
    if (qualityGate != null) {
      RouteShape shape = qualityGate.getShape();
      double penalty = 0;
      if (shape == RouteShape.LOLLIPOP) penalty = SHAPE_PENALTY_LOLLIPOP;
      else if (shape == RouteShape.SCENIC_OUT_AND_BACK) penalty = SHAPE_PENALTY_SCENIC;
      if (penalty > 0) {
        reasons.add(new Reason("shape " + shape + " penalty",
          -penalty, 0, -penalty));
        total -= penalty;
      }
    }

    double clamped = Math.max(0.0, Math.min(1.0, total));
    return new Verdict(clamped, reasons);
  }

  private static String fmt(double v) {
    return String.format(Locale.US, "%.2f", v);
  }
}
