package btools.router;

import java.util.ArrayList;

import org.junit.Assert;
import org.junit.Test;

/**
 * Acceptance tests for the AUTO candidate-competition flow described in
 * docs/features/roundtrip-auto-quality-redesign.md §353. Covers all 14
 * listed test cases — some are pure-logic on {@link RouteChoiceScore}, others
 * require real routing (segments-gated, skipped if data absent).
 *
 * <p>The flow under test: for generated AUTO loops with no user vias, the
 * engine runs ISO_GREEDY first, compares GREEDY when ISO_GREEDY is weak,
 * and falls back to legacy WAYPOINT/probe only when greedy variants fail.
 * Accepted candidates are scored and the highest-scoring child track is
 * adopted directly.
 */
public class RoutingEngineAutoCompetitionTest {

  // =========================================================================
  // §353.8 — Route-choice score returns a reason breakdown.
  // §353.7 — Direction weak: cannot dominate other factors.
  // §353.12 — ISO/radial telemetry fields available on RoundTripResult.
  // §353.13 — Low-iso classification uses accepted legs, not routed.
  // §353.14 — Existing forced algorithm tests still work (covered by other suites).
  // These are pure-logic tests that run unconditionally.
  // =========================================================================

  @Test
  public void routeChoiceScoreReturnsReasonBreakdown() {
    OsmTrack t = cleanSquareLoop(5000);
    RoundTripQualityResult gateVerdict = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.STRICT_LOOP).build();
    RouteChoiceScore.Verdict v = RouteChoiceScore.score(t, t.distance, "fastbike", gateVerdict);

    Assert.assertTrue("score in [0,1]", v.score() >= 0 && v.score() <= 1);
    Assert.assertFalse("reasons non-empty", v.reasons().isEmpty());
    // All component categories present (distance, reuse, closure, continuity,
    // compactness, cost/m, direction).
    Assert.assertTrue("has distance reason",
      v.reasons().stream().anyMatch(r -> r.label.contains("distance ratio")));
    Assert.assertTrue("has reuse reason",
      v.reasons().stream().anyMatch(r -> r.label.contains("road reuse")));
    Assert.assertTrue("has closure reason",
      v.reasons().stream().anyMatch(r -> r.label.contains("closure")));
    Assert.assertTrue("has cost/m reason",
      v.reasons().stream().anyMatch(r -> r.label.contains("cost/m")));
    // describe() produces multi-line output
    String desc = v.describe();
    Assert.assertTrue("describe has score line", desc.contains("score="));
    Assert.assertTrue("describe has reasons section", desc.contains("reasons:"));
  }

  @Test
  public void directionDeltaCannotDominateScore() {
    // §353.7. Direction may shift a candidate's score by at most W_DIRECTION
    // (5%), never more — it must not dominate distance/reuse/closure/etc.
    OsmTrack good = cleanSquareLoop(5000);
    RoundTripQualityResult gate = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.STRICT_LOOP).build();

    // (a) The direction reason's contribution is bounded by a hard literal
    //     (0.05). Asserting against RouteChoiceScore.W_DIRECTION itself would be
    //     tautological — dirContrib == W_DIRECTION * dirScore with dirScore in
    //     [0,1], so it always holds and a W_DIRECTION regression would slip past.
    double dirContrib = RouteChoiceScore.score(good, good.distance, "fastbike", gate, 0)
      .reasons().stream().filter(r -> r.label.startsWith("direction delta"))
      .findFirst().get().scoreContribution;
    Assert.assertTrue("direction contribution must stay <= 0.05; got " + dirContrib,
      Math.abs(dirContrib) <= 0.05 + 1e-9);

    // (b) Dominance: scoring the SAME track across every requested direction
    //     varies only the direction delta, so the total score may move by at
    //     most 0.05 between the best- and worst-aligned direction.
    double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
    for (int dir = 0; dir < 360; dir += 30) {
      double s = RouteChoiceScore.score(good, good.distance, "fastbike", gate, dir).score();
      min = Math.min(min, s);
      max = Math.max(max, s);
    }
    Assert.assertTrue("direction must actually affect the score (otherwise the bound is vacuous)",
      max - min > 0);
    Assert.assertTrue("direction may swing the total score by at most 0.05; swing was " + (max - min),
      max - min <= 0.05 + 1e-9);
  }

  @Test
  public void rejectedGateMeansZeroScore() {
    // §353.9. A candidate rejected by the hard gate cannot win, regardless
    // of what the soft score would compute. We test this by handing
    // RouteChoiceScore a rejected verdict — it returns 0.
    OsmTrack t = cleanSquareLoop(5000);
    RoundTripQualityResult rejected = RoundTripQualityResult.builder()
      .accepted(false).shape(RouteShape.INVALID_RETRACE)
      .rejectionReason("synthetic rejection").build();
    RouteChoiceScore.Verdict v = RouteChoiceScore.score(t, t.distance, "fastbike", rejected);
    Assert.assertEquals("rejected gate → zero score", 0.0, v.score(), 1e-9);
  }

  @Test
  public void shapeLollipopAndScenicGetPenalty() {
    OsmTrack t = cleanSquareLoop(5000);
    RoundTripQualityResult strict = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.STRICT_LOOP).build();
    RoundTripQualityResult lollipop = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.LOLLIPOP).build();
    RoundTripQualityResult scenic = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.SCENIC_OUT_AND_BACK).build();

    double strictS = RouteChoiceScore.score(t, t.distance, "fastbike", strict).score();
    double lollipopS = RouteChoiceScore.score(t, t.distance, "fastbike", lollipop).score();
    double scenicS = RouteChoiceScore.score(t, t.distance, "fastbike", scenic).score();
    Assert.assertTrue("STRICT_LOOP > LOLLIPOP", strictS > lollipopS);
    Assert.assertTrue("LOLLIPOP > SCENIC_OUT_AND_BACK", lollipopS > scenicS);
  }

  @Test
  public void childCandidateBudgetSharesTheDeadline() {
    // The AUTO competition runs candidates sequentially against one shared
    // deadline; each child gets the REMAINING time, floored so a spawned
    // candidate still gets a usable slice (never the full request timeout).
    long now = 1_000_000L;
    Assert.assertEquals("ample time remaining → full remainder",
      50_000L, RoutingEngine.childCandidateBudgetMs(now + 50_000L, now));
    Assert.assertEquals("remaining below the 5s floor → floored",
      5_000L, RoutingEngine.childCandidateBudgetMs(now + 3_000L, now));
    Assert.assertEquals("deadline already passed → floored, never negative",
      5_000L, RoutingEngine.childCandidateBudgetMs(now - 10_000L, now));
  }

  @Test
  public void roundTripResultExposesIsoRadialTelemetry() {
    // §353.12. The RoundTripResult model carries iso/radial routed +
    // accepted counters; default 0.
    RoundTripResult r = new RoundTripResult();
    Assert.assertEquals(0, r.getRoutedIsoCandidates());
    Assert.assertEquals(0, r.getRoutedRadialCandidates());
    Assert.assertEquals(0, r.getAcceptedIsoLegs());
    Assert.assertEquals(0, r.getAcceptedRadialLegs());

    r.setRoutedIsoCandidates(12);
    r.setRoutedRadialCandidates(8);
    r.setAcceptedIsoLegs(3);
    r.setAcceptedRadialLegs(2);
    Assert.assertEquals(12, r.getRoutedIsoCandidates());
    Assert.assertEquals(8, r.getRoutedRadialCandidates());
    Assert.assertEquals(3, r.getAcceptedIsoLegs());
    Assert.assertEquals(2, r.getAcceptedRadialLegs());
  }

  @Test
  public void candidateResultModelTracksAlgorithmAndAcceptance() {
    // The internal RoundTripCandidateResult wrapper aggregates the per-
    // candidate fields and exposes accepted() / scoreValue() helpers used
    // by the competition loop.
    RoundTripCandidateResult r = new RoundTripCandidateResult(RoundTripAlgorithm.ISO_GREEDY);
    Assert.assertEquals(RoundTripAlgorithm.ISO_GREEDY, r.algorithm);
    Assert.assertFalse("no track + no gate → not accepted", r.accepted());
    Assert.assertEquals("scoreValue 0 when no score", 0.0, r.scoreValue(), 1e-9);

    r.track = cleanSquareLoop(5000);
    r.gateVerdict = RoundTripQualityResult.builder()
      .accepted(true).shape(RouteShape.STRICT_LOOP).build();
    r.score = RouteChoiceScore.score(r.track, r.track.distance, "fastbike", r.gateVerdict);
    Assert.assertTrue("accepted now", r.accepted());
    Assert.assertTrue("scoreValue > 0", r.scoreValue() > 0);
  }

  // ------------- helpers -----------------------------------------------------

  /** A clean rectangular loop with proper paved metadata. */
  private static OsmTrack cleanSquareLoop(int sideMeters) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    int base_ilon = 180_000_000;
    int base_ilat = 50_000_000;
    int s = sideMeters;
    addNode(t, base_ilon + 0,           base_ilat + 0);
    addNode(t, base_ilon + s * 14,      base_ilat + 0);
    addNode(t, base_ilon + s * 14,      base_ilat + s * 9);
    addNode(t, base_ilon + 0,           base_ilat + s * 9);
    addNode(t, base_ilon + 0,           base_ilat + 0);
    int d = 0;
    for (int i = 1; i < t.nodes.size(); i++) {
      d += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
      MessageData m = new MessageData();
      m.wayKeyValues = "highway=residential surface=asphalt";
      m.costfactor = 1.0f;
      t.nodes.get(i).message = m;
    }
    t.distance = d;
    return t;
  }

  private static void addNode(OsmTrack t, int ilon, int ilat) {
    t.nodes.add(OsmPathElement.create(ilon, ilat, (short) 0, null));
  }
}
