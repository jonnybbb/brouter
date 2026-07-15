package btools.router.roundtrip;

import btools.router.OsmTrack;

/**
 * Internal candidate-comparison record for AUTO mode round-trip routing.
 *
 * <p>When AUTO runs its candidate algorithms in sequence (ISO_GREEDY first,
 * then GREEDY if ISO_GREEDY was weak, then the legacy WAYPOINT/probe and
 * ISOCHRONE paths only as last-resort fallbacks), each attempt produces one
 * {@link RoundTripCandidateResult}. The comparison loop in
 * {@code RoutingEngine.runAutoCandidateCompetition} examines all accepted
 * candidates and picks the highest {@link RouteChoiceScore.Verdict#score()}.
 *
 * <p>Field semantics:
 * <ul>
 *   <li>{@link #algorithm} — which algorithm produced this candidate.</li>
 *   <li>{@link #track} — the actually-routed {@link OsmTrack}, or null if
 *       routing failed.</li>
 *   <li>{@link #gateVerdict} — the {@link RoundTripQualityGate} hard-safety
 *       verdict. A candidate with {@code gateVerdict.isAccepted() == false}
 *       is ineligible regardless of its score.</li>
 *   <li>{@link #score} — the {@link RouteChoiceScore.Verdict} for ranking;
 *       null if the candidate failed before scoring was attempted.</li>
 *   <li>{@link #runtimeMillis} — wall-clock of this candidate's attempt.</li>
 *   <li>{@link #planner} — the child planner's full {@link RoundTripResult};
 *       all issue-#26 winner-attribution telemetry reads through it.</li>
 *   <li>{@link #errorMessage} — set when the candidate failed (engine error,
 *       gate rejection, planner aborted).</li>
 * </ul>
 *
 * <p>Internal-only; do not expose on the public API.
 */
public final class RoundTripCandidateResult {
  public final RoundTripAlgorithm algorithm;
  public OsmTrack track;
  public RoundTripQualityResult gateVerdict;
  public RouteChoiceScore.Verdict score;
  public long runtimeMillis;
  /**
   * The child planner's result, or {@code null} when the candidate never
   * produced one (legacy WAYPOINT/ISOCHRONE paths, engine error). The
   * attribution accessors below read THROUGH this reference — the former
   * field-by-field mirror invited copy omissions whose sentinel defaults
   * (NaN/-1/0) silently flipped AUTO's plain-GREEDY absorption decision.
   */
  public RoundTripResult planner;
  public String errorMessage;

  public RoundTripCandidateResult(RoundTripAlgorithm algorithm) {
    this.algorithm = algorithm;
  }

  public boolean accepted() {
    return errorMessage == null && track != null
      && gateVerdict != null && gateVerdict.isAccepted();
  }

  public double scoreValue() {
    return (score == null) ? 0.0 : score.score();
  }

  int routedIsoCandidates() {
    return planner == null ? 0 : planner.getRoutedIsoCandidates();
  }

  int routedNonIsoCandidates() {
    return planner == null ? 0 : planner.getRoutedNonIsoCandidates();
  }

  int acceptedQuotaInjectedLegs() {
    return planner == null ? 0 : planner.getAcceptedQuotaInjectedLegs();
  }

  /** Final iso-pool health score; {@code NaN} = no iso pool. */
  double isoPoolHealthScore() {
    return planner == null ? Double.NaN : planner.getIsoPoolHealthScore();
  }

  /** Step at which the pool's influence was reduced; {@code -1} = never. */
  int poolDemotedAtStep() {
    return planner == null ? -1 : planner.getPoolDemotedAtStep();
  }

  public boolean internalGraphNativeCompared() {
    return planner != null && planner.isInternalGraphNativeCompared();
  }

  /** The child's explicit graph-native-only start decision (issue #26). */
  public boolean graphNativeOnlyStart() {
    return planner != null && planner.isGraphNativeOnlyStart();
  }

  /**
   * Keep-when-forced marker from the child planner
   * ({@link RoundTripResult#isForcedCorridorAccepted()}): the loop is a rideable
   * same-way-back corridor and the planner proved no clean alternative exists.
   * The parent's re-gate must honor it (allow same-way-back) exactly like the
   * direct routing path does — otherwise AUTO rejects a route the child engine
   * accepted by design.
   */
  public boolean forcedCorridorAccepted() {
    return planner != null && planner.isForcedCorridorAccepted();
  }

  @Override
  public String toString() {
    if (errorMessage != null) {
      return algorithm + ": error \"" + errorMessage + "\"";
    }
    if (track == null) {
      return algorithm + ": no track";
    }
    return String.format(java.util.Locale.US,
      "%s: accepted=%s, track=%dm, gateShape=%s, score=%.3f, runtime=%dms, isoRouted=%d, nonIsoRouted=%d",
      algorithm, accepted(), track.distance,
      gateVerdict == null ? "?" : gateVerdict.getShape(),
      scoreValue(), runtimeMillis, routedIsoCandidates(), routedNonIsoCandidates())
      // Winner-attribution suffix, only for candidates that ran an iso pool —
      // this is what the "plain GREEDY fallback wins are gone" acceptance
      // check greps from AUTO competition logs.
      + (Double.isNaN(isoPoolHealthScore()) ? "" : String.format(java.util.Locale.US,
        ", quotaAccepted=%d, poolHealth=%.2f, demotedAtStep=%d",
        acceptedQuotaInjectedLegs(), isoPoolHealthScore(), poolDemotedAtStep()))
      + (internalGraphNativeCompared() ? ", internalGraphNativeCompared=true" : "");
  }
}
