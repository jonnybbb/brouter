package btools.router;

import java.util.ArrayList;
import java.util.List;

/**
 * QUALITY-tier candidate provider that blends an {@link IsochroneCandidateProvider}
 * with a {@link RoundTripCandidateProvider.RadialCandidateProvider} fallback ring.
 *
 * <p>Pure ISO_GREEDY (iso candidates only) regressed in rural/coastal cells where
 * the bounded isochrone's road-native pool happens to be sparse or pulled toward
 * one corridor — the planner has no "geometric safety candidates" to fall back
 * on when iso doesn't propose anything useful in a particular angular wedge.
 * Blending fixes that: iso candidates win where roads actually exist; the radial
 * ring guarantees angular coverage at the planner's requested air-radius even
 * when iso has nothing for that wedge.
 *
 * <p>For each step we ask both providers, then concatenate iso results first
 * (so the planner's score-sort favours road-native picks at the same heuristic
 * tier) followed by the radial ring.
 */
final class BlendedCandidateProvider implements RoundTripCandidateProvider {

  private final IsochroneCandidateProvider iso;
  private final RadialCandidateProvider radial;

  BlendedCandidateProvider(IsochroneCandidateProvider iso) {
    this.iso = iso;
    this.radial = new RadialCandidateProvider();
  }

  @Override
  public List<CandidatePoint> candidatesForStep(
    int fromIlon, int fromIlat, double airRadius,
    int step, int totalSteps,
    int startIlon, int startIlat,
    double startDirection) {
    List<CandidatePoint> isoPicks = iso.candidatesForStep(
      fromIlon, fromIlat, airRadius, step, totalSteps,
      startIlon, startIlat, startDirection);
    List<CandidatePoint> radialPicks = radial.candidatesForStep(
      fromIlon, fromIlat, airRadius, step, totalSteps,
      startIlon, startIlat, startDirection);
    List<CandidatePoint> blended = new ArrayList<>(isoPicks.size() + radialPicks.size());
    blended.addAll(isoPicks);
    blended.addAll(radialPicks);
    return blended;
  }
}
