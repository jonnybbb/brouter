package btools.router.roundtrip;

import btools.router.OsmPathElement;
import btools.router.OsmTrack;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Soft ranking score for choosing among round-trip candidates that already PASSED
 * {@link RoundTripQualityGate} — it ranks, it does not gate. The gate makes the hard
 * accept/reject decision; failed candidates are excluded from scoring.
 *
 * <p>Design: score in {@code [0, 1]}, higher better, with a reason breakdown; production-neutral
 * preferred bands (no {@code LoopTestRegion}/benchmark deps); direction a weak factor that cannot
 * dominate reuse/closure/distance; cost/m a soft preference; scores the final {@link OsmTrack}
 * only; penalises surprising shapes (LOLLIPOP/OUT_AND_BACK rank below a clean STRICT_LOOP).
 *
 * <p>AUTO should use {@link #score(OsmTrack, double, String, RoundTripQualityResult, double)} so
 * the requested direction participates; the four-arg overload is for callers without one. The
 * {@link Verdict} carries the score and per-component contributions.
 */
public final class RouteChoiceScore {

  // ---- Component weights -------------------------------------------------
  // Sum to 1.0 across the positive contributions. Direction is intentionally
  // small (5%) so it cannot dominate. The shape-penalty is subtracted on top
  // of the positive sum, so a clean STRICT_LOOP can score above 1.0 in raw
  // terms — clamped at the end.

  // Weights sum to 1.0. 2026-06-15 rebalance (user directive): "distance can be tolerated — an
  // appealing route should score higher." Distance dropped 0.25→0.18 AND made tolerant (flat band,
  // below), road-character lifted 0.07→0.17, cost/m down 0.08→0.05. Net: a clean, appealing loop now
  // out-ranks one that buys a few % more distance by detouring through a town (e.g. basel_80km_E,
  // where AUTO used to ship the Lörrach-dipping iso_greedy over the city-skirting greedy).
  /** Distance closeness to requested (now tolerant — see the flat band in #1). */
  static final double W_DISTANCE   = 0.18;
  /** Road reuse / retrace penalty (low reuse = high score). */
  static final double W_REUSE      = 0.20;
  /** Closure quality (small closure gap = high score). */
  static final double W_CLOSURE    = 0.10;
  /** Continuity / max-gap (no synthetic beelines + small max-gap). */
  static final double W_CONTINUITY = 0.15;
  /** Compactness within reasonable range. */
  static final double W_COMPACTNESS = 0.10;
  /**
   * Road-CHARACTER preference (profile-aware highway desirability from the route's tags) — the
   * "would a real cyclist want to ride this" dimension. Now the dominant soft term so an appealing
   * route out-ranks a marginally-better-distance one. Validated against the user's eye on the
   * Basel/Freiburg matrix (residential-avoiding loops score higher).
   */
  static final double W_CHARACTER  = clamp01x(0.17, 0.30);
  /** Profile-specific cost/m soft preference. */
  static final double W_COSTM      = 0.05;
  /** Direction delta — weak; capped so it cannot dominate. */
  static final double W_DIRECTION  = 0.05;

  private static double clamp01x(double v, double hi) { return Math.max(0.0, Math.min(hi, v)); }
  /** Shape disclosure penalty: LOLLIPOP/SCENIC down-weight vs STRICT_LOOP. */
  static final double SHAPE_PENALTY_LOLLIPOP = 0.05;
  public static final double SHAPE_PENALTY_OUT_AND_BACK = 0.15;
  /**
   * Self-intersection penalty per crossing — a route-shape defect the cyclist sees that no other
   * term captures (reuse measures co-linear retrace, not transverse crossings). Subtracted per
   * crossing so AUTO prefers the cleaner of two comparable candidates.
   *
   * <p>Measured (full loop-quality matrix, offline AUTO, 2026-06): 0.08/crossing takes shipped
   * crossed loops 12.8%→3.7% (residual = both candidates cross) with no reuse/distance regression.
   *
   * <p>Ranking-only — RCS never gates acceptance ({@link RoundTripQualityGate} does), so it cannot
   * cause a no-route; the gate caps accepted loops at
   * {@link RoundTripQualityGate#MAX_SELF_INTERSECTIONS}, bounding the penalty.
   */
  static final double SHAPE_PENALTY_PER_SELF_INTERSECTION =
    0.08;

  /**
   * Lasso surcharge, ADDITIVE on top of {@link #SHAPE_PENALTY_PER_SELF_INTERSECTION}, for crossings
   * whose enclosed arc is short (≤ {@link LoopQualityMetrics#SMALL_LOOP_MAX_ARC_METERS}) — a small
   * detour loop the cyclist rides around, a more visible defect than a structural crossing. Keeping
   * it additive preserves that term's measured calibration for structural crossings.
   *
   * <p><b>Size-faded</b>: per-lasso severity = {@code 1 − enclosedArc / SMALL_LOOP_MAX_ARC_METERS},
   * so a 400m circle pays ~the full surcharge (~2.5× a structural crossing) while a near-cap 3.5km
   * loop pays almost nothing. Ranking-only, excluded from {@link Verdict#qualityScore()}.
   */
  static final double SHAPE_PENALTY_LASSO_EXTRA =
    0.12;

  /**
   * Near-revisit / "teardrop" penalty — catches the defect
   * {@link #SHAPE_PENALTY_PER_SELF_INTERSECTION} MISSES: a loop that returns within
   * {@link #NEAR_REVISIT_EPS_M} of an earlier point and rejoins without a <em>transverse</em>
   * crossing (a clean teardrop, scored 0 by {@link RoundTripQualityGate#countSelfIntersections}).
   * Motivating case: Basel 30 km gravel ships a 7.3 km teardrop to Ettingen while a teardrop-free
   * alternative exists.
   *
   * <p>Severity-weighted by excursion arc (Σ min(1, arc / {@link #NEAR_REVISIT_NORM_M})), not a
   * flat count, so a 7 km teardrop outweighs a 600 m wiggle. Ranking-only, excluded from
   * {@link Verdict#qualityScore()}. Default is provisional pending corpus A/B calibration.
   */
  static final double SHAPE_PENALTY_PER_TEARDROP =
    0.12;
  /** Near-revisit detector thresholds (mirror the probe; 10 km cap catches Basel's 7.3 km). */
  static final double NEAR_REVISIT_EPS_M = 60.0;
  static final double NEAR_REVISIT_MIN_ARC_M = 600.0;
  static final double NEAR_REVISIT_MAX_ARC_M = 10000.0;
  /** Excursion arc (m) at which a teardrop is "full severity" (1.0). */
  static final double NEAR_REVISIT_NORM_M = 5000.0;
  /** A near-revisit span covering more than this fraction of the perimeter is the loop's own
   *  start≈end closure (only reachable on loops shorter than the arc cap), not a teardrop. */
  static final double CLOSURE_EXCLUSION_FRACTION = 0.85;

  // ---- Profile-typical cost-per-meter bands ------------------------------
  // Used to compute the cost/m component. A route on roads the profile
  // actively prefers (cost/m at or below the lower bound) scores 1.0; above
  // the upper bound scores 0; linearly interpolated in between.
  //
  // These values mirror the existing {@link LoopQualityMetrics#computeCostMatchScore}
  // bands but per-profile, because gravel/MTB have higher cost-per-meter
  // baselines on their preferred surfaces (a 1.5 cost/m gravel route may
  // be on perfect terrain; the same value on fastbike means rough roads).

  /**
   * Weight of the cost/m term for a profile — {@link #W_COSTM} everywhere except
   * <b>mtb, where it is deliberately ZERO</b>.
   *
   * <p>Cost per metre measures how much the profile dislikes the ground. For a paved
   * profile that is a quality signal: low cost/m means good roads. <b>For mtb it is not
   * a quality signal at all</b> — it measures how EASY the terrain is, and an mtb rider
   * is not looking for easy terrain. Rewarding low cost/m there steers the scorer toward
   * forest road over technical trail, which is backwards for the profile.
   *
   * <p>Measured, 2026-07-25, 588-cell AUTO matrix A/B: giving the term a live band for
   * mtb ({@code [8,14]} instead of the old {@code [4,9]}, which saturated at zero) moved
   * 15 cells — length |dev| 8.2% → 7.0%, but mean compactness 0.342 → 0.286, worse on 10
   * cells and better on 5. It bought 1.2 points of length accuracy with 16% of loop
   * shape. Zeroing the weight is the honest form of that result: the dimension does not
   * rank mtb loops, and saying so explicitly stops a future reader "fixing" a band that
   * looks mis-calibrated but is inert on purpose.
   *
   * <p>Consequence to know: the positive weights therefore sum to 0.95 for mtb, not 1.0,
   * so mtb scores sit ~0.05 below an otherwise identical paved-profile score against the
   * absolute bars ({@code CLEAR_ACCEPT_THRESHOLD}, {@code MIN_RCS_PASS}). That matches
   * the behaviour the corpus was calibrated on — the old band already scored 55 of 64
   * mtb cells at zero.
   */
  static double costMWeight(String profileName) {
    return profileName != null && profileName.toLowerCase(Locale.ROOT).contains("mtb")
      ? 0.0 : W_COSTM;
  }

  public static double[] costMBand(String profileName) {
    if (profileName == null) return new double[]{1.5, 4.0};
    String n = profileName.toLowerCase(Locale.ROOT);
    // "road" must match as a whole word, not a substring: profiles like
    // "Trekking-SmallRoads" are NOT paved-friendly racing profiles. The same
    // whole-token name convention classifies the bike family in
    // RoadCharacterScore.family. (RoundTripQualityGate decides paved-ness from the
    // profile cost model now, not the name, so it is deliberately not referenced here.)
    if (n.contains("fastbike") || n.matches(".*\\broad\\b.*") || n.contains("racing")) {
      return new double[]{1.2, 3.0};   // tight band: paved-friendly profiles
    }
    if (n.contains("gravel")) {
      return new double[]{2.0, 5.0};
    }
    if (n.contains("mtb")) {
      // The measured range: 64 mtb loop cells (Finale/Freiburg/Garmisch/Girona
      // x town vs trailhead x 4 headings x FAST/AUTO) run 7.98-15.18 cost/m,
      // median 10.20. Reported for logging only — the term carries ZERO weight
      // for mtb, see costMWeight.
      return new double[]{8.0, 14.0};
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
    final double weight;
    final double rawValue;       // the underlying metric value
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
    private final double qualityScore;
    private final List<Reason> reasons;

    public Verdict(double score, double qualityScore, List<Reason> reasons) {
      this.score = score;
      this.qualityScore = qualityScore;
      this.reasons = Collections.unmodifiableList(new ArrayList<>(reasons));
    }

    /** Ranking score, including the self-intersection penalty (AUTO uses it to prefer the cleaner
     *  of two comparable candidates). */
    public double score() { return score; }

    /**
     * Multi-dimension quality score EXCLUDING the self-intersection penalty, for soft quality-floor
     * checks (e.g. MIN_RCS_PASS). Crossings are already hard-gated by {@link RoundTripQualityGate},
     * so the floor must not double-count them; {@link #score()} keeps the penalty for ranking.
     * Other shape penalties (LOLLIPOP/OUT_AND_BACK) remain in both.
     */
    public double qualityScore() { return qualityScore; }
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
   * Score a candidate that already passed {@link RoundTripQualityGate}; scoring a rejected
   * candidate returns a zero-score Verdict.
   *
   * @param profileName the active profile name (used for cost/m bands)
   * @param qualityGate gate verdict for the shape + disclosure penalty; null defaults shape to
   *                    STRICT_LOOP with no disclosure penalty
   */
  public static Verdict score(OsmTrack track, double requestedDistance,
                              String profileName, RoundTripQualityResult qualityGate) {
    return score(track, requestedDistance, profileName, qualityGate, 0);
  }

  public static Verdict score(OsmTrack track, double requestedDistance,
                              String profileName, RoundTripQualityResult qualityGate,
                              double requestedDirection) {
    return score(track, requestedDistance, profileName, qualityGate, requestedDirection, true);
  }

  /**
   * Best-effort ranking variant: scores a gate-REJECTED track on its real geometry (bypasses the
   * rejected-gate zero guard) while still using the gate's shape for the disclosure penalty.
   * Passing {@code null} would also bypass the guard but silently lose the LOLLIPOP/OUT_AND_BACK
   * penalty and rank a rejected corridor as a strict loop.
   */
  public static Verdict scoreBestEffort(OsmTrack track, double requestedDistance,
                                        String profileName, RoundTripQualityResult qualityGate,
                                        double requestedDirection) {
    return score(track, requestedDistance, profileName, qualityGate, requestedDirection, false);
  }

  private static Verdict score(OsmTrack track, double requestedDistance,
                               String profileName, RoundTripQualityResult qualityGate,
                               double requestedDirection, boolean rejectedGateZeroes) {
    if (track == null || track.nodes == null || track.nodes.size() < 2) {
      List<Reason> empty = new ArrayList<>();
      empty.add(new Reason("no track", 0, 0, 0));
      return new Verdict(0.0, 0.0, empty);
    }
    if (rejectedGateZeroes && qualityGate != null && !qualityGate.isAccepted()) {
      List<Reason> empty = new ArrayList<>();
      empty.add(new Reason("gate rejected: " + qualityGate.getRejectionReason(),
        0, 0, 0));
      return new Verdict(0.0, 0.0, empty);
    }

    // ONE crossing analysis for this verdict: the gate's stamped analysis when
    // it evaluated this same track, else computed here once — feeds the
    // metrics, the self-intersection penalty (#9), and the lasso surcharge
    // (#9b) instead of three independent full-track scans.
    LoopAnalysis analysis = (qualityGate != null && qualityGate.getLoopAnalysis() != null)
      ? qualityGate.getLoopAnalysis()
      : LoopAnalysis.of(track);
    LoopQualityMetrics m = LoopQualityMetrics.compute(track, (int) requestedDistance,
      requestedDirection, analysis);
    List<Reason> reasons = new ArrayList<>(8);
    double total = 0;

    // 1. Distance closeness — TOLERANT (user directive: distance can be tolerated). Flat 1.0 within
    //    ±15% of target, then decays to 0.0 at ±50%. So a clean 0.95× loop and an on-target 0.98×
    //    loop score identically on distance, and road-character decides between them — an appealing
    //    route is no longer out-ranked just because it is a few % short.
    double distDelta = Math.abs(m.getDistanceRatio() - 1.0);
    double distScore = distDelta <= 0.15 ? 1.0 : 1.0 - Math.min(1.0, (distDelta - 0.15) / 0.35);
    double distContrib = W_DISTANCE * distScore;
    reasons.add(new Reason("distance ratio " + fmt(m.getDistanceRatio())
      + " (full score within ±15%, zero outside [0.5, 1.5])", W_DISTANCE, m.getDistanceRatio(), distContrib));
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

    // 5. Compactness — NET enclosed area (isoperimetric), not the convex
    //    hull. The hull credits exactly the defect classes this term exists
    //    to price: a V-thread hulls into a fat triangle (raw 0.35 for a loop
    //    enclosing ~nothing — pinned by threadLoopGetsNoCompactnessCredit),
    //    a serpentine crumple fills its hull (Vosges 75 mtb W: hull 0.25 vs
    //    net 0.03), a necked two-lobe likewise (Crete 75 gravel S: 0.29 vs
    //    0.13). Display/report surfaces keep the hull metric; only the
    //    ranking term reads net area. Already in [0,1].
    double netCompact = LoopQualityMetrics.enclosedAreaCompactness(track.nodes, track.distance);
    double compactContrib = W_COMPACTNESS * netCompact;
    reasons.add(new Reason("compactness " + fmt(netCompact) + " (net enclosed area)",
      W_COMPACTNESS, netCompact, compactContrib));
    total += compactContrib;

    // 6. cost/m within profile-typical band.
    double[] band = costMBand(profileName);
    double costM = m.getAverageCostPerMeter();
    double costMScore;
    if (costM <= band[0]) costMScore = 1.0;
    else if (costM >= band[1]) costMScore = 0.0;
    else costMScore = (band[1] - costM) / (band[1] - band[0]);
    double wCostM = costMWeight(profileName);
    double costMContrib = wCostM * costMScore;
    reasons.add(new Reason("cost/m " + fmt(costM)
      + (wCostM > 0 ? " (preferred band [" + fmt(band[0]) + ", " + fmt(band[1]) + "])"
                    : " (not ranked for this profile)"),
      wCostM, costM, costMContrib));
    total += costMContrib;

    // 6b. Road character — profile-aware highway desirability from the route's tags (not its cost,
    //     which would be circular for a residential-penalising profile). Captures the rider-eye
    //     "do I want to ride this" dimension that geometry terms miss.
    if (W_CHARACTER > 0) {
      double appeal = RoadCharacterScore.compute(track, profileName).appeal;
      double charContrib = W_CHARACTER * appeal;
      reasons.add(new Reason("road character " + fmt(appeal), W_CHARACTER, appeal, charContrib));
      total += charContrib;
    }

    // 7. Direction delta — weak. Score 1.0 at delta 0°, 0.0 at delta 180°.
    //    Direction is intentionally a small weight so it cannot dominate
    //    other factors.
    double dirScore = 1.0 - Math.min(1.0, m.getDirectionDeltaDegrees() / 180.0);
    double dirContrib = W_DIRECTION * dirScore;
    reasons.add(new Reason("direction delta " + fmt(m.getDirectionDeltaDegrees()) + "°",
      W_DIRECTION, m.getDirectionDeltaDegrees(), dirContrib));
    total += dirContrib;

    // 8. Shape disclosure penalty: cyclist sees the route shape; a clean
    //    STRICT_LOOP ranks above a LOLLIPOP above a OUT_AND_BACK
    //    all else equal. In strict mode INVALID_RETRACE is gate-rejected above
    //    and never reaches here; in best-effort mode it can — it carries no
    //    explicit penalty, but its ~50%+ reuse already zeroes the reuse term.
    if (qualityGate != null) {
      RouteShape shape = qualityGate.getShape();
      double penalty = 0;
      if (shape == RouteShape.LOLLIPOP) penalty = SHAPE_PENALTY_LOLLIPOP;
      else if (shape == RouteShape.OUT_AND_BACK) penalty = SHAPE_PENALTY_OUT_AND_BACK;
      if (penalty > 0) {
        reasons.add(new Reason("shape " + shape + " penalty",
          -penalty, 0, -penalty));
        total -= penalty;
      }
    }

    // 9. Self-intersection penalty. A route-shape defect the cyclist sees that
    //    no term above captures (reuse = co-linear retrace, not transverse
    //    crossings). Read from the shared analysis (gate-stamped or computed
    //    once above). Lets AUTO pick the cleaner of two comparable candidates;
    //    measured to take shipped crossings 12.8%→3.7% (see constant).
    int selfIntersections = analysis.selfIntersections;
    double selfIntPenalty = 0;
    if (selfIntersections > 0) {
      selfIntPenalty = SHAPE_PENALTY_PER_SELF_INTERSECTION * selfIntersections;
      reasons.add(new Reason("self-intersections " + selfIntersections,
        -selfIntPenalty, selfIntersections, -selfIntPenalty));
      total -= selfIntPenalty;
    }

    // 9b. Lasso surcharge: crossings whose enclosed sub-loop is short are
    //     small detour loops the cyclist rides around. Size-faded severity
    //     (see SHAPE_PENALTY_LASSO_EXTRA): a tiny circle pays ~full, a loop
    //     near the 4km cap pays almost nothing. Guarded on #9 so the
    //     O(n²-capped) classification scan only runs on tracks that have
    //     crossings at all.
    double lassoPenalty = 0;
    if (selfIntersections > 0 && track.nodes != null) {
      double lassoSeverity = 0;
      for (double[] x : analysis.crossingPoints) {
        double enclosed = x[2];
        if (enclosed <= LoopQualityMetrics.SMALL_LOOP_MAX_ARC_METERS) {
          lassoSeverity += 1.0 - enclosed / LoopQualityMetrics.SMALL_LOOP_MAX_ARC_METERS;
        }
      }
      if (lassoSeverity > 0) {
        lassoPenalty = SHAPE_PENALTY_LASSO_EXTRA * lassoSeverity;
        reasons.add(new Reason("lasso crossings sev " + fmt(lassoSeverity),
          -lassoPenalty, lassoSeverity, -lassoPenalty));
        total -= lassoPenalty;
      }
    }

    // 10. Near-revisit / teardrop penalty. The shape defect #9 misses: a "small loop
    //     back to the same point" that never transversely crosses (countSelfIntersections=0).
    //     Severity = Σ min(1, arc/NORM) over teardrop spans, so a big excursion outweighs a
    //     minor wiggle. Ranking-only like #9 (added back into qualityScore below).
    double teardropSeverity = teardropSeverity(track);
    double teardropPenalty = 0;
    if (teardropSeverity > 0) {
      teardropPenalty = SHAPE_PENALTY_PER_TEARDROP * teardropSeverity;
      reasons.add(new Reason("near-revisit teardrop sev " + fmt(teardropSeverity),
        -teardropPenalty, teardropSeverity, -teardropPenalty));
      total -= teardropPenalty;
    }

    // 11. Clover / petal penalty. Several out-and-backs sharing a centre: no
    //     transverse crossing (#9 blind), real enclosed area (#5 blind), each
    //     petal too long for the teardrop window (#10 blind). FAST already
    //     retries on this shape; this is the same eye for AUTO's ranking.
    double petalAmp = LoopQualityMetrics.petalAmplitude(track.nodes, track.distance);
    double petalPenalty = 0;
    if (petalAmp > PETAL_FREE_AMPLITUDE) {
      double sev = Math.min(1.0, (petalAmp - PETAL_FREE_AMPLITUDE)
        / (PETAL_FULL_AMPLITUDE - PETAL_FREE_AMPLITUDE));
      petalPenalty = SHAPE_PENALTY_PETAL * sev;
      reasons.add(new Reason("petal amplitude " + fmt(petalAmp)
        + " (clover; free below " + fmt(PETAL_FREE_AMPLITUDE) + ")",
        -petalPenalty, petalAmp, -petalPenalty));
      total -= petalPenalty;
    }

    // 12. Far-field dwell penalty. How much of the ride is actually spent away
    //     from home — the bowtie/orbit defect. Floor mirrors FAST's own.
    double dwell = LoopQualityMetrics.farFieldDwell(track.nodes);
    double dwellPenalty = 0;
    if (dwell < DWELL_FLOOR) {
      double sev = Math.min(1.0, (DWELL_FLOOR - dwell) / (DWELL_FLOOR - DWELL_FULL_PENALTY_AT));
      dwellPenalty = SHAPE_PENALTY_DWELL * sev;
      reasons.add(new Reason("far-field dwell " + fmt(dwell)
        + " (floor " + fmt(DWELL_FLOOR) + ")", -dwellPenalty, dwell, -dwellPenalty));
      total -= dwellPenalty;
    }

    // 13. Crumple penalty. #5 can only withhold credit; a loop enclosing
    //     ~nothing needs to be pushed DOWN, or it reaches the clear-accept bar
    //     on the strength of its other, perfectly satisfied dimensions.
    double crumplePenalty = 0;
    if (netCompact < CRUMPLE_FLOOR) {
      double sev = Math.min(1.0, (CRUMPLE_FLOOR - netCompact) / CRUMPLE_FLOOR);
      crumplePenalty = SHAPE_PENALTY_CRUMPLE * sev;
      reasons.add(new Reason("crumple: encloses " + fmt(netCompact)
        + " (floor " + fmt(CRUMPLE_FLOOR) + ")", -crumplePenalty, netCompact, -crumplePenalty));
      total -= crumplePenalty;
    }

    // Ranking score includes every shape penalty; qualityScore excludes them ALL
    // (they are ranking signals to pick the cleaner alternative — a terrain-forced
    // defect shipping as best-available must not trip the soft floor).
    double clamped = Math.max(0.0, Math.min(1.0, total));
    double qualityClamped = Math.max(0.0, Math.min(1.0, total + selfIntPenalty + lassoPenalty
      + teardropPenalty + petalPenalty + dwellPenalty + crumplePenalty));
    return new Verdict(clamped, qualityClamped, reasons);
  }

  /**
   * Sum of teardrop severities (Σ min(1, arc / {@link #NEAR_REVISIT_NORM_M})) over the track's
   * near-revisit spans, excluding the loop's own start≈end closure. Drives the shape penalty.
   * See {@link LoopQualityMetrics#nearRevisitSpans}.
   */
  static double teardropSeverity(OsmTrack track) {
    if (track == null || track.nodes == null) return 0;
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    if (n < 4) return 0;
    double[] cum = LoopGeometry.cumulativeDistances(nodes);
    double total = cum[n - 1];
    double sev = 0;
    for (int[] s : LoopQualityMetrics.nearRevisitSpans(nodes, cum,
        NEAR_REVISIT_EPS_M, NEAR_REVISIT_MIN_ARC_M, NEAR_REVISIT_MAX_ARC_M)) {
      double arc = cum[s[1]] - cum[s[0]];
      if (total > 0 && arc > CLOSURE_EXCLUSION_FRACTION * total) continue; // loop closure, not a teardrop
      // Position weight: a teardrop in the ride's first or last kilometres is
      // what the rider sees while still fresh (and what a screenshot shows).
      // Measured tile-clean A/B (2026-07-20, 548 AUTO cells): moves 10 routes,
      // all inside the planners' own top-K selection (0 changed the AUTO
      // winner); removes 5.1 + 5.1 + 3.2 km of ridden-twice stretches in the
      // three biggest cells, aggregate double-riding −47%; one regression in a
      // chronically terrain-walled Garmisch cell that is poor either way.
      // (An earlier same-day A/B read as a 66-cell wash and led to a brief
      // revert — that spread was a mid-experiment E5_N45 tile refresh, not
      // this weight. Full history: .agents/route-comparison-20260720/.)
      // Ranking-only, like the whole teardrop term: qualityScore excludes it,
      // so it steers selection and can never fail a cell.
      double mid = (cum[s[0]] + cum[s[1]]) / 2.0;
      double edgeFrac = total > 0 ? Math.min(mid, total - mid) / total : 1.0;
      double posWeight = edgeFrac <= TEARDROP_START_ZONE_FRACTION
        ? TEARDROP_START_ZONE_WEIGHT : 1.0;
      sev += posWeight * Math.min(1.0, arc / NEAR_REVISIT_NORM_M);
    }
    return sev;
  }

  /**
   * Clover penalty — {@link LoopQualityMetrics#petalAmplitude} above
   * {@link #PETAL_FREE_AMPLITUDE}, ramped to full at {@link #PETAL_FULL_AMPLITUDE}.
   *
   * <p>Prices the defect NO other term sees. A clover's petals meet at a shared junction
   * instead of crossing, so {@code countSelfIntersections} correctly reports zero; its
   * enclosed area stays well above the thread floor, so {@link #W_COMPACTNESS} passes it;
   * and each petal is short enough to duck the teardrop arc window. FAST's retry cascade
   * has treated {@code petalAmplitude >= 0.30} as a failure shape since the shape work —
   * this term gives AUTO's ranking the same eye, so a clean alternative can win on it.
   *
   * <p>Magnitude sits just under {@link #SHAPE_PENALTY_OUT_AND_BACK}: a full clover is
   * about as unwelcome as a disclosed out-and-back. Ranking-only, excluded from
   * {@link Verdict#qualityScore()}.
   */
  static final double SHAPE_PENALTY_PETAL = 0.12;
  /** Petal amplitude that starts costing: FAST's clover trigger; corpus median is 0.16. */
  static final double PETAL_FREE_AMPLITUDE = 0.30;
  /** Petal amplitude at full penalty; measured clover maximum on the 548-cell matrix is 0.56. */
  static final double PETAL_FULL_AMPLITUDE = 0.60;

  /**
   * Home-hugging penalty — {@link LoopQualityMetrics#farFieldDwell} below
   * {@link #DWELL_FLOOR}, ramped to full at {@link #DWELL_FULL_PENALTY_AT}.
   *
   * <p>The rider's "am I actually going somewhere" dimension: the share of the ride spent
   * far from the start. A circle through the start dwells ~0.59 by construction, a plain
   * out-and-back ~0.40, a clover ~0.30. The floor mirrors {@code FastStrategy.MIN_FAST_DWELL}
   * so both tiers call the same shape bad.
   *
   * <p>Deliberately smaller than {@link #SHAPE_PENALTY_PETAL}: dwell overlaps the petal and
   * out-and-back signals, and stacking two full-weight penalties on one defect would let a
   * shape term outvote reuse. An out-and-back's ~0.40 clears the floor, so this never
   * double-charges {@link #SHAPE_PENALTY_OUT_AND_BACK}. Ranking-only.
   */
  static final double SHAPE_PENALTY_DWELL = 0.10;
  /** Far-field dwell below which the loop reads as orbiting its own start. */
  static final double DWELL_FLOOR = 0.35;
  /** Far-field dwell at full penalty. */
  static final double DWELL_FULL_PENALTY_AT = 0.15;

  /**
   * Crumple penalty — net {@link LoopQualityMetrics#enclosedAreaCompactness} below
   * {@link #CRUMPLE_FLOOR}, ramped to full at zero enclosed area.
   *
   * <p>Why a penalty when {@link #W_COMPACTNESS} already scores compactness: that term can
   * only WITHHOLD its 0.10, which is not enough to keep a pathological shape off the 0.85
   * clear-accept bar when every other dimension is satisfied. Vosges 75 km mtb shipped a
   * serpentine crumple (net compactness 0.033) that scored 0.850 with clean reuse, closure,
   * continuity and excellent trails — it was held below the bar only by the mtb cost band
   * being dead, and reviving that band (measured: +0.029) pushed it over, silencing the
   * weak-winner second opinion that had fixed the cell. A loop that rides 73 km around
   * nothing is a defect, and is priced as one.
   *
   * <p>Floor calibrated on the 12-cell three-way corpus: shipped loops run 0.19-0.58 net
   * compactness with a median near 0.32; everything below 0.10 (Annecy 0.07, Finale 0.00-0.11,
   * Garmisch 0.01-0.04) is a route no rider would follow. Ranking-only.
   */
  static final double SHAPE_PENALTY_CRUMPLE = 0.12;
  /** Net enclosed-area compactness below which a loop reads as a crumple/thread. */
  static final double CRUMPLE_FLOOR = 0.10;

  /** Arc fraction from either end of the loop that counts as the start/finish zone. */
  static final double TEARDROP_START_ZONE_FRACTION = 0.10;
  /** Severity multiplier for a teardrop inside the start/finish zone. */
  static final double TEARDROP_START_ZONE_WEIGHT = 2.0;

  private static String fmt(double v) {
    return String.format(Locale.US, "%.2f", v);
  }
}
