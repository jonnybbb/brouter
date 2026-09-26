package btools.router.roundtrip;

import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import btools.router.OsmPathElement;
import btools.router.OsmTrack;

public class RouteChoiceScoreTest {

  // ~8.72E, ~50.0N in BRouter internal coords
  private static final int BASE_ILON = 188720000;
  private static final int BASE_ILAT = 140000000;

  private static OsmPathElement pt(double xMeters, double yMeters) {
    double latScale = Math.cos(Math.toRadians(BASE_ILAT / 1e6 - 90.0));
    double perMx = 1e6 / (111320.0 * latScale);
    double perMy = 1e6 / 110540.0;
    return OsmPathElement.create(BASE_ILON + (int) Math.round(xMeters * perMx),
      BASE_ILAT + (int) Math.round(yMeters * perMy), (short) 0, null);
  }

  /**
   * A ~20 km circular loop with one 1.5 km out-and-back spur, inserted at the
   * given fraction of the perimeter. The spur is a near-revisit teardrop
   * (arc gap ~3 km, return within metres of the entry).
   */
  private static OsmTrack loopWithSpur(double spurAtFraction) {
    double r = 20000.0 / (2 * Math.PI);
    List<OsmPathElement> nodes = new ArrayList<>();
    int steps = 400;
    int spurStep = (int) (steps * spurAtFraction);
    for (int i = 0; i <= steps; i++) {
      double th = 2 * Math.PI * i / steps;
      double x = r * (1 + Math.cos(th + Math.PI));
      double y = r * Math.sin(th + Math.PI);
      nodes.add(pt(x, y));
      if (i == spurStep) {
        // radial out-and-back spur, 1.5 km each way, 15 points out and back
        double ux = Math.cos(th + Math.PI);
        double uy = Math.sin(th + Math.PI);
        for (int k = 1; k <= 15; k++) {
          nodes.add(pt(x + ux * 100 * k, y + uy * 100 * k));
        }
        for (int k = 14; k >= 0; k--) {
          nodes.add(pt(x + ux * 100 * k, y + uy * 100 * k));
        }
      }
    }
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    return t;
  }

  @Test
  public void teardropNearStartIsPenalizedHarderThanMidLoop() {
    // The x2 start-zone weight was convicted, exonerated and restored on the
    // same day (2026-07-20): an A/B first read as a 66-cell wash, which turned
    // out to be a mid-experiment E5_N45 tile refresh. The tile-clean A/B shows
    // 10 moved routes, aggregate double-riding -47%, three cells losing
    // 5.1/5.1/3.2 km of ridden-twice stretches. Full history in
    // .agents/route-comparison-20260720/.
    double nearStart = RouteChoiceScore.teardropSeverity(loopWithSpur(0.03));
    double midLoop = RouteChoiceScore.teardropSeverity(loopWithSpur(0.50));
    Assert.assertTrue("both spurs must register (start=" + nearStart + " mid=" + midLoop + ")",
      nearStart > 0 && midLoop > 0);
    Assert.assertEquals("same spur must weigh ~2x in the start zone",
      RouteChoiceScore.TEARDROP_START_ZONE_WEIGHT, nearStart / midLoop, 0.15);
  }

  /**
   * A V-shaped "thread": two 10 km out-and-back arms 120° apart, return legs
   * offset 60 m. Rides ~40 km but encloses almost nothing (~1.2 km² in two
   * thin strips) while its convex hull is a fat 43 km² triangle — the exact
   * blind case the compactness metric's own javadoc documents.
   */
  private static OsmTrack threadLoop() {
    List<OsmPathElement> nodes = new ArrayList<>();
    double[][] arms = {{1, 0}, {-0.5, Math.sqrt(3) / 2}};
    for (double[] u : arms) {
      double px = -u[1], py = u[0]; // left normal for the 60 m offset
      for (int i = 0; i <= 100; i++) {
        nodes.add(pt(u[0] * 100.0 * i, u[1] * 100.0 * i));
      }
      for (int i = 100; i >= 0; i--) {
        nodes.add(pt(u[0] * 100.0 * i + px * 60, u[1] * 100.0 * i + py * 60));
      }
    }
    nodes.add(pt(0, 0));
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    t.distance = trackLengthMeters(nodes);
    return t;
  }

  private static OsmTrack ringLoop() {
    double r = 20000.0 / (2 * Math.PI);
    List<OsmPathElement> nodes = new ArrayList<>();
    for (int i = 0; i <= 400; i++) {
      double th = 2 * Math.PI * i / 400;
      nodes.add(pt(r * (1 + Math.cos(th + Math.PI)), r * Math.sin(th + Math.PI)));
    }
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    t.distance = trackLengthMeters(nodes);
    return t;
  }

  private static int trackLengthMeters(List<OsmPathElement> nodes) {
    int d = 0;
    for (int i = 1; i < nodes.size(); i++) {
      d += nodes.get(i - 1).calcDistance(nodes.get(i));
    }
    return d;
  }

  private static RouteChoiceScore.Reason compactnessReason(OsmTrack t) {
    RouteChoiceScore.Verdict v = RouteChoiceScore.score(t, t.distance, "fastbike", null);
    for (RouteChoiceScore.Reason r : v.reasons()) {
      if (r.label.startsWith("compactness")) {
        return r;
      }
    }
    throw new AssertionError("no compactness reason in " + v.describe());
  }

  @Test
  public void threadLoopGetsNoCompactnessCredit() {
    // Spec: the compactness term measures how much area the ride ENCLOSES.
    // A thread that rides 40 km around ~nothing deserves ~none of the 0.10
    // weight — whatever its convex hull spans.
    RouteChoiceScore.Reason r = compactnessReason(threadLoop());
    Assert.assertTrue("thread compactness credit must be ~zero, got " + r,
      r.scoreContribution <= 0.005);
  }

  @Test
  public void cleanRingKeepsCompactnessCredit() {
    RouteChoiceScore.Reason r = compactnessReason(ringLoop());
    Assert.assertTrue("clean ring must keep most compactness credit, got " + r,
      r.scoreContribution >= 0.07);
  }

  /** N-petal clover through the start: r(theta) = R*(1+cos(lobes*theta))/2. */
  private static OsmTrack cloverLoop(double radiusM, int lobes) {
    List<OsmPathElement> nodes = new ArrayList<>();
    int steps = 720;
    for (int i = 0; i <= steps; i++) {
      double th = 2 * Math.PI * i / steps;
      double r = radiusM * (1 + Math.cos(lobes * th)) / 2;
      nodes.add(pt(r * Math.cos(th), r * Math.sin(th)));
    }
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    t.distance = trackLengthMeters(nodes);
    return t;
  }

  private static RouteChoiceScore.Reason reasonStartingWith(OsmTrack t, String prefix) {
    RouteChoiceScore.Verdict v = RouteChoiceScore.score(t, t.distance, "fastbike", null);
    for (RouteChoiceScore.Reason r : v.reasons()) {
      if (r.label.startsWith(prefix)) {
        return r;
      }
    }
    return null;
  }

  @Test
  public void cloverPaysAPetalPenalty() {
    // Spec: a clover is several out-and-backs pretending to be a loop. It
    // crosses itself nowhere (petals meet at a shared centre) and encloses
    // real area, so neither the self-intersection term nor compactness sees
    // it. FAST's retry cascade already treats petalAmplitude >= 0.30 as a
    // failure shape; the scorer must price the same defect so AUTO's ranking
    // can prefer a clean alternative.
    RouteChoiceScore.Reason r = reasonStartingWith(cloverLoop(5000, 4), "petal");
    Assert.assertNotNull("clover must carry a petal penalty reason", r);
    Assert.assertTrue("petal penalty must be negative, got " + r, r.scoreContribution < 0);
  }

  @Test
  public void cleanRingPaysNoPetalPenalty() {
    Assert.assertNull("a round loop must not be charged for petals",
      reasonStartingWith(ringLoop(), "petal"));
  }

  /**
   * The real low-dwell shape: a lot of local wandering near the start plus one
   * long spike that sets the loop's reach. A 3 km blob (~19 km of arc, all of it
   * inside the far-field bar) with a 10 km out-and-back spike — the rider covers
   * distance but is almost never actually away from home. This is the measured
   * Annecy-FAST (0.27) and Finale-FAST (0.25) class, not a clover: a synthetic
   * clover dwells 0.42-0.53 because its far petals really are far from the start.
   */
  private static OsmTrack spikeAndLocalWander() {
    List<OsmPathElement> nodes = new ArrayList<>();
    for (int k = 0; k <= 100; k++) {
      nodes.add(pt(k * 100.0, 0));            // 10 km out
    }
    for (int k = 100; k >= 0; k--) {
      nodes.add(pt(k * 100.0, 60));           // and back, 60 m offset
    }
    for (int i = 0; i <= 360; i++) {          // 3 km blob around the start
      double th = 2 * Math.PI * i / 360;
      nodes.add(pt(3000 * (Math.cos(th) - 1), 3000 * Math.sin(th)));
    }
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    t.distance = trackLengthMeters(nodes);
    return t;
  }

  @Test
  public void loopThatNeverLeavesHomePaysADwellPenalty() {
    // The rider's "am I actually going somewhere" dimension. Floor mirrors
    // FAST's MIN_FAST_DWELL so both tiers call the same shape bad.
    RouteChoiceScore.Reason r = reasonStartingWith(spikeAndLocalWander(), "far-field dwell");
    Assert.assertNotNull("a home-hugging loop must carry a far-field dwell penalty", r);
    Assert.assertTrue("dwell penalty must be negative, got " + r, r.scoreContribution < 0);
  }

  @Test
  public void cloverIsNotChargedTwiceForDwell() {
    // Guard against stacking: a synthetic clover measures dwell 0.42-0.53 — its
    // petals genuinely reach far from the start — so it pays the petal penalty
    // and nothing more. Only a shape that really stays home pays dwell.
    Assert.assertNull("clover must pay petals, not dwell",
      reasonStartingWith(cloverLoop(5000, 4), "far-field dwell"));
  }

  @Test
  public void cleanRingPaysNoDwellPenalty() {
    // A circle through the start dwells ~0.59, comfortably above the floor.
    Assert.assertNull("a round loop must not be charged for dwell",
      reasonStartingWith(ringLoop(), "far-field dwell"));
  }

  @Test
  public void loopEnclosingNothingPaysACrumplePenalty() {
    // The compactness CREDIT term (#5, weight 0.10) can only withhold 0.10 from
    // a loop that rides 40 km around nothing — not enough to stop it reaching
    // the 0.85 clear-accept bar when every other dimension is satisfied. That
    // is how Vosges 75 km mtb shipped a serpentine crumple (net compactness
    // 0.033) for months: it was held below the bar only by the mtb cost term
    // being dead, and reviving that band pushed it back over. A shape this
    // pathological has to be priced as a defect, not as missing credit.
    RouteChoiceScore.Reason r = reasonStartingWith(threadLoop(), "crumple");
    Assert.assertNotNull("a loop enclosing ~nothing must carry a crumple penalty", r);
    Assert.assertTrue("crumple penalty must be negative, got " + r, r.scoreContribution < 0);
  }

  @Test
  public void cleanRingPaysNoCrumplePenalty() {
    Assert.assertNull("a round loop must not be charged for crumpling",
      reasonStartingWith(ringLoop(), "crumple"));
  }

  @Test
  public void petalAndDwellAreRankingOnlyLikeTheOtherShapePenalties() {
    // Same contract as self-intersections/lasso/teardrop: they steer the CHOICE
    // between candidates but must never trip the soft quality floor, or a
    // terrain-forced clover shipping as best-available would fail its cell.
    OsmTrack clover = cloverLoop(5000, 4);
    RouteChoiceScore.Verdict v =
      RouteChoiceScore.score(clover, clover.distance, "fastbike", null);
    Assert.assertTrue("qualityScore must exclude the new shape penalties ("
        + v.qualityScore() + " vs " + v.score() + ")", v.qualityScore() > v.score());
  }

  @Test
  public void costPerMetreDoesNotRankMtbLoops() {
    // Cost/m measures how EASY the ground is. For a paved profile that is a
    // quality signal; for mtb it is the opposite of what the rider wants, so
    // the term carries zero weight there. Measured on the 588-cell AUTO matrix:
    // a live band moved 15 mtb cells, length |dev| 8.2%->7.0% but compactness
    // 0.342->0.286 — 1.2 points of length bought with 16% of loop shape.
    Assert.assertEquals("mtb: cost/m must not rank",
      0.0, RouteChoiceScore.costMWeight("mtb"), 1e-9);
    Assert.assertEquals("mtb variants too",
      0.0, RouteChoiceScore.costMWeight("rove-mtb"), 1e-9);
    for (String paved : new String[]{"fastbike", "gravel", "trekking", "road", null}) {
      Assert.assertEquals("cost/m still ranks " + paved,
        RouteChoiceScore.W_COSTM, RouteChoiceScore.costMWeight(paved), 1e-9);
    }
  }

  @Test
  public void mtbVerdictCarriesNoCostContribution() {
    OsmTrack t = ringLoop();
    RouteChoiceScore.Verdict v = RouteChoiceScore.score(t, t.distance, "mtb", null);
    for (RouteChoiceScore.Reason r : v.reasons()) {
      if (r.label.startsWith("cost/m")) {
        Assert.assertEquals("mtb cost/m must contribute nothing, got " + r,
          0.0, r.scoreContribution, 1e-9);
        return;
      }
    }
    throw new AssertionError("cost/m reason must still be reported for transparency");
  }

  @Test
  public void mtbCostBandCoversTheTerrainMtbActuallyRides() {
    // Measured over 64 mtb loop cells (4 regions x 2 starts x 4 headings x
    // 2 tiers, .agents/route-comparison-20260720/startaudit): cost/m ranges
    // 7.98 to 15.18, median 10.20. The old [4, 9] band scored 55 of those 64
    // at exactly zero — a dead term that could not rank one mtb loop above
    // another. The band must span the real distribution instead.
    double[] band = RouteChoiceScore.costMBand("mtb");
    Assert.assertTrue("mtb band low must sit at the good end of the real range, got "
      + band[0], band[0] >= 7.5 && band[0] <= 8.5);
    Assert.assertTrue("mtb band high must sit above the 90th percentile (12.04), got "
      + band[1], band[1] >= 13.0);
    // The band is retained for LOGGING only — costMWeight("mtb") is 0, so it
    // ranks nothing. Kept truthful to the measured range so the log line is
    // informative rather than misleading.
  }

  @Test
  public void cleanLoopHasZeroTeardropSeverity() {
    double r = 20000.0 / (2 * Math.PI);
    List<OsmPathElement> nodes = new ArrayList<>();
    for (int i = 0; i <= 400; i++) {
      double th = 2 * Math.PI * i / 400;
      nodes.add(pt(r * (1 + Math.cos(th + Math.PI)), r * Math.sin(th + Math.PI)));
    }
    OsmTrack t = new OsmTrack();
    t.nodes = nodes;
    Assert.assertEquals(0.0, RouteChoiceScore.teardropSeverity(t), 1e-9);
  }
}
