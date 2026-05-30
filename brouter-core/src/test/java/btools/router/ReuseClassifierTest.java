package btools.router;

import java.util.ArrayList;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Unit tests for the semantic reuse classifier — covers every case the
 * spec ("Same-Way-Back Quality Gate") names:
 *
 * <ul>
 *   <li>short start/end shared stem is accepted</li>
 *   <li>long start/end retrace is rejected unless classified as terminal
 *       spur or same-way-back</li>
 *   <li>long mid-route retrace is rejected</li>
 *   <li>clean loop remains STRICT_LOOP</li>
 *   <li>lollipop route with terminal spur is accepted as LOLLIPOP</li>
 *   <li>pure out-and-back is rejected by default</li>
 *   <li>pure out-and-back is accepted when allowSamewayback=true</li>
 *   <li>scenic out-and-back is not returned as STRICT_LOOP</li>
 *   <li>profile-hostile scenic spur is still rejected for fastbike</li>
 * </ul>
 *
 * <p>Plus synthetic fixtures for the cape/dead-end-corridor case, the
 * mountain-pass-like spur, and an accidental U-turn in the middle of an
 * otherwise loop-like route.
 */
public class ReuseClassifierTest {

  // ============ Helpers ====================================================

  /**
   * Node coordinate scale. CheapRuler interprets ilon/ilat as integer
   * degrees-times-1e6. At ~50° latitude, 1m east ≈ 14 ilon units, 1m
   * north ≈ 9 ilat units. We pick a base point well inside the supported
   * range and use the (14, 9) scale so the test geometry is meters-accurate.
   */
  private static final int BASE_ILON = 180_000_000;
  private static final int BASE_ILAT = 50_000_000;
  private static final int ILON_PER_M = 14;
  private static final int ILAT_PER_M = 9;

  private static OsmPathElement node(int xMeters, int yMeters) {
    return OsmPathElement.create(
      BASE_ILON + xMeters * ILON_PER_M,
      BASE_ILAT + yMeters * ILAT_PER_M,
      (short) 0, null);
  }

  /**
   * Build a track from (x, y) meter offsets; computes track.distance from
   * edges and stamps each edge with clean residential metadata so the
   * paved-profile hostility gate (which is part of {@link RoundTripQualityGate#evaluate})
   * accepts the route. The classifier itself doesn't care about
   * MessageData, but the gate-level integration tests run the full pipeline.
   */
  private static OsmTrack track(int[][] coords) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    for (int[] xy : coords) t.nodes.add(node(xy[0], xy[1]));
    int d = 0;
    for (int i = 1; i < t.nodes.size(); i++) d += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
    t.distance = d;
    // Stamp clean paved tags so RoundTripQualityGate.evaluate doesn't
    // reject these synthetic tracks for "missing metadata" before the
    // semantic classifier even runs.
    for (int i = 1; i < t.nodes.size(); i++) {
      MessageData m = new MessageData();
      m.wayKeyValues = "highway=residential surface=asphalt";
      m.costfactor = 1.0f;
      t.nodes.get(i).message = m;
    }
    return t;
  }

  /** A clean rectangular loop with no retracing. */
  private static OsmTrack cleanLoop(int sideMeters) {
    return track(new int[][]{
      {0, 0},
      {sideMeters, 0},
      {sideMeters, sideMeters},
      {0, sideMeters},
      {0, 0}
    });
  }

  /**
   * Lollipop: A → P (stem-out) → Q → R → P (loop body) → A (stem-back).
   * Stem of {@code stemMeters} on each side; loop body roughly a triangle.
   */
  private static OsmTrack lollipop(int stemMeters, int loopSideMeters) {
    // P at (stem, 0). Triangle: P → (stem+loop, 0) → (stem+loop/2, loop) → P.
    int pX = stemMeters;
    return track(new int[][]{
      {0, 0},                                  // A
      {pX, 0},                                 // P (hub)
      {pX + loopSideMeters, 0},                // loop corner 1
      {pX + loopSideMeters / 2, loopSideMeters}, // loop corner 2
      {pX, 0},                                 // back at P
      {0, 0}                                   // back at A
    });
  }

  /** Pure out-and-back to a single far node. */
  private static OsmTrack outAndBack(int spurMeters) {
    return track(new int[][]{
      {0, 0},
      {spurMeters / 3, 0},
      {2 * spurMeters / 3, 0},
      {spurMeters, 0},
      {2 * spurMeters / 3, 0},
      {spurMeters / 3, 0},
      {0, 0}
    });
  }

  /**
   * Loop with an accidental U-turn in the middle: square loop, but after
   * reaching the far corner the route goes back to the previous corner
   * and forward again before continuing.
   */
  private static OsmTrack loopWithMidUTurn(int sideMeters) {
    return track(new int[][]{
      {0, 0},
      {sideMeters, 0},        // corner 2
      {sideMeters, sideMeters / 2}, // mid right edge
      {sideMeters, 0},        // U-turn back
      {sideMeters, sideMeters},     // corner 3
      {0, sideMeters},        // corner 4
      {0, 0}                  // back to start
    });
  }

  // ============ Tests ======================================================

  @Test
  public void cleanLoopStaysStrictLoop() {
    OsmTrack t = cleanLoop(5000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("clean loop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.STRICT_LOOP, r.getShape());
    assertEquals("no reuse", 0.0, r.getTotalReuseRatio(), 0.001);
    assertEquals("no stem", 0, r.getTerminalStemReuseMeters());
    assertEquals("no spur", 0, r.getScenicSpurReuseMeters());
  }

  @Test
  public void shortSharedStemIsAccepted() {
    // 500m stem + 4km-side loop body → total ~14km. The stem cap is
    // min(1500m, 5% of 14km = 700m) = 700m, so the 500m stem is comfortably
    // inside the cap → STRICT_LOOP with a stem disclosure.
    OsmTrack t = lollipop(500, 4000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("short-stem lollipop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.STRICT_LOOP, r.getShape());
    assertTrue("stem disclosure surfaced",
      r.getDisclosures().stream().anyMatch(d -> d.contains("shared stem")));
  }

  @Test
  public void longStartEndRetraceRejectedAsOutAndBackByDefault() {
    // 10km out-and-back: pure same-way-back, no loop body at all. Must be
    // rejected when allowSamewayback=false (the default).
    OsmTrack t = outAndBack(10000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertFalse("pure out-and-back rejected by default: " + r, r.isAccepted());
    assertEquals(RouteShape.SCENIC_OUT_AND_BACK, r.getShape());
    assertNotNull(r.getRejectionReason());
  }

  @Test
  public void longStartEndRetraceAcceptedAsScenicWhenAllowed() {
    // Same out-and-back, with allowSamewayback=true → accepted as
    // SCENIC_OUT_AND_BACK with a disclosure.
    OsmTrack t = outAndBack(10000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, true);
    assertTrue("out-and-back accepted with allowSamewayback: " + r, r.isAccepted());
    assertEquals(RouteShape.SCENIC_OUT_AND_BACK, r.getShape());
    assertTrue("out-and-back disclosure surfaced",
      r.getDisclosures().stream().anyMatch(d -> d.toLowerCase().contains("out-and-back")));
  }

  @Test
  public void longMidRouteRetraceRejected() {
    // Square loop where the cyclist u-turns back to the previous corner
    // mid-loop. The reused edge sits in the middle of the route; this
    // pattern is exactly what the gate must catch.
    OsmTrack t = loopWithMidUTurn(5000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertFalse("mid-route U-turn rejected: " + r, r.isAccepted());
    assertEquals(RouteShape.INVALID_RETRACE, r.getShape());
    assertTrue("rejection reason mentions retrace/backtrack: " + r.getRejectionReason(),
      r.getRejectionReason().toLowerCase().contains("retrace")
        || r.getRejectionReason().toLowerCase().contains("backtrack"));
  }

  @Test
  public void lollipopAcceptedWithDisclosure() {
    // Big lollipop: 3km stem + 12km loop body. The stem (3km) exceeds the
    // absolute stem cap (1500m), so it's classified as a TERMINAL_SPUR.
    // The non-retraced loop body is ~12km of a ~18km total → > 35%.
    OsmTrack t = lollipop(3000, 4000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("lollipop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.LOLLIPOP, r.getShape());
    assertTrue("scenic spur disclosure surfaced",
      r.getDisclosures().stream().anyMatch(d -> d.contains("scenic spur")));
    assertTrue("spur length recorded", r.getScenicSpurReuseMeters() > 0);
  }

  @Test
  public void startTouchingDominantSpurStillClassifiesAsLollipop() {
    // Regression: a lollipop whose DOMINANT retrace touches the START (not the
    // end) used to be wrongly rejected as SCENIC_OUT_AND_BACK because
    // isStructuralLollipop's touchesStart branch skipped the hub node by one,
    // counting the start hub once (closing visit) instead of twice.
    //
    // Layout A,Q,A,P,c1,c2,P,A: a 6km start spur (A→Q→A, the longer/dominant
    // retrace, within the 8% boundary band of this ~200km loop), a 5km stem
    // (A→P / P→A), and a large non-retraced loop body (P→c1→c2→P). The hub is
    // the start node A — visited at the end of the start spur AND again when
    // the loop closes — so the unique suffix revisits it: a genuine LOLLIPOP.
    OsmTrack t = track(new int[][]{
      {0, 0},            // A
      {6000, 0},         // Q  (start spur out)
      {0, 0},            // A  (start spur back — hub)
      {0, 5000},         // P  (stem out)
      {50000, 5000},     // c1 (loop body)
      {25000, 55000},    // c2 (loop body)
      {0, 5000},         // P  (stem back)
      {0, 0}             // A  (close — hub revisited)
    });
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertEquals(RouteShape.LOLLIPOP, r.getShape());
    assertTrue("start-touching-dominant lollipop accepted: " + r, r.isAccepted());
  }

  @Test
  public void scenicOutAndBackNotReturnedAsStrictLoop() {
    // Even when allowed, an out-and-back is never returned as STRICT_LOOP.
    OsmTrack t = outAndBack(8000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, true);
    assertTrue("accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.SCENIC_OUT_AND_BACK, r.getShape());
  }

  @Test
  public void shortStemBelowCapKeepsLoopStrict() {
    // Very short stem (500m) on a 20km loop. Within the stem cap → still
    // classified as STRICT_LOOP (we don't downgrade clean loops just because
    // they happen to share 500m of access road).
    OsmTrack t = lollipop(500, 4500);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("short-stem loop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.STRICT_LOOP, r.getShape());
  }

  @Test
  public void capeStyleSpurAcceptedAsLollipopOrScenic() {
    // Cape/dead-end corridor: route goes 4km out to a tip, then back.
    // The "loop body" is essentially zero — pure spur. With allowSamewayback
    // true, the gate accepts it as SCENIC_OUT_AND_BACK and discloses.
    OsmTrack t = outAndBack(4000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, true);
    assertTrue("cape spur accepted with allowSamewayback: " + r, r.isAccepted());
    assertEquals(RouteShape.SCENIC_OUT_AND_BACK, r.getShape());
  }

  @Test
  public void longStemBigLoopLollipopStillAccepted() {
    // 5km stem + 8km loop side → ~25km total, with the FARTHEST-from-start
    // point on the loop body (not on the stem tip). This is the realistic
    // "long stem + bigger loop" lollipop that earlier farthest-point gating
    // mis-rejected. The structural test (stem-back pattern + closed loop
    // body) must accept it.
    OsmTrack t = lollipop(5000, 8000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("long-stem lollipop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.LOLLIPOP, r.getShape());
  }

  @Test
  public void mountainPassSpurAcceptedAsLollipop() {
    // Mountain-pass-like: short stem out (1km), big loop body around the
    // pass area, back along the stem. Stem exceeds absolute cap so it
    // shows up as a spur; loop body is meaningful → LOLLIPOP.
    OsmTrack t = lollipop(1800, 5000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertTrue("mountain-pass lollipop accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.LOLLIPOP, r.getShape());
  }

  @Test
  public void accidentalUTurnRejectedEvenInLoopLikeRoute() {
    OsmTrack t = loopWithMidUTurn(3000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertFalse("U-turn rejected: " + r, r.isAccepted());
    assertEquals(RouteShape.INVALID_RETRACE, r.getShape());
  }

  @Test
  public void zigzagWithRepeatedEdgeRejectedAsAccidental() {
    // Hand-built zigzag: an edge that the route walks 4 times. Even though
    // the contiguous reuse touches the route end, the visit ordinal > 2 is
    // structural evidence of accidental backtracking — never a clean spur.
    OsmTrack t = track(new int[][]{
      {0, 0},
      {1000, 0},
      {2000, 0},
      {1000, 0},     // 1st reuse of edge 1-2
      {2000, 0},     // 2nd reuse of edge 1-2
      {1000, 0},     // 3rd reuse of edge 1-2  → visit ordinal = 4
      {0, 0}         // closing
    });
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertFalse("zigzag (4x edge) rejected: " + r, r.isAccepted());
    assertEquals(RouteShape.INVALID_RETRACE, r.getShape());
  }

  @Test
  public void rejectionResultsCarryShapeAndReason() {
    OsmTrack t = loopWithMidUTurn(3000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    assertFalse(r.isAccepted());
    assertNotNull("shape always set", r.getShape());
    assertNotNull("rejection reason always set on reject", r.getRejectionReason());
  }

  @Test
  public void scenicOutAndBackDoesNotPretendToBeLoop() {
    // Even with allowSamewayback=true, the result's shape must be
    // SCENIC_OUT_AND_BACK — never STRICT_LOOP. A consumer that asks
    // "is this a loop?" must get the correct answer.
    OsmTrack t = outAndBack(6000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, true);
    assertEquals(RouteShape.SCENIC_OUT_AND_BACK, r.getShape());
    org.junit.Assert.assertNotSame("out-and-back is not STRICT_LOOP",
      RouteShape.STRICT_LOOP, r.getShape());
  }

  @Test
  public void stemReuseCapScalesWithRequestedDistance() {
    // 5km loop → cap is 5% = 250m (under the 1500m absolute cap).
    assertEquals(250, ReuseClassifier.stemReuseCap(5000));
    // 50km loop → cap is 1500m (absolute cap wins over 5%).
    assertEquals(1500, ReuseClassifier.stemReuseCap(50000));
    // Zero requested → absolute cap (defensive).
    assertEquals(1500, ReuseClassifier.stemReuseCap(0));
  }

  @Test
  public void unclassifiedCapScalesWithRequestedDistance() {
    // Pure 8% × distance, no upper clamp.
    assertEquals(800, ReuseClassifier.unclassifiedContiguousCap(10000));
    assertEquals(2400, ReuseClassifier.unclassifiedContiguousCap(30000));
    assertEquals(4000, ReuseClassifier.unclassifiedContiguousCap(50000));
    assertEquals(8000, ReuseClassifier.unclassifiedContiguousCap(100000));
    // Legacy fallback for degenerate test fixtures with no requested distance.
    assertEquals(2000, ReuseClassifier.unclassifiedContiguousCap(0));
  }

  // ---- Phase 1.4 — long-loop retrace tolerance --------------------------

  @Test
  public void longLoopAcceptsSingleRetraceWithin8Percent() {
    // 100 km requested, single ~5 km mid-route retrace (5% of requested) →
    // under the 8% cap → accepted. Pre-Phase 1.4 the absolute 2000m clamp
    // would have rejected this.
    int requested = 100000;
    OsmTrack t = synthLongLoopWithMidRetrace(requested, 5000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, requested, false);
    assertTrue("100km request with 5km mid retrace (5%) accepted under Phase 1.4: " + r,
      r.isAccepted());
  }

  @Test
  public void longLoopRejectsSingleRetraceOver8Percent() {
    // 100 km requested, single ~10 km mid-route retrace (10% of requested) →
    // over the 8% cap → rejected. The fraction rule still does the work.
    int requested = 100000;
    OsmTrack t = synthLongLoopWithMidRetrace(requested, 10000);
    RoundTripQualityResult r = ReuseClassifier.classify(t, requested, false);
    assertFalse("10% mid-route retrace must reject: " + r, r.isAccepted());
    assertEquals(RouteShape.INVALID_RETRACE, r.getShape());
  }

  @Test
  public void mediumLoopRetraceUsesNewFractionalCap() {
    // 30 km requested: 8% × 30km = 2400m cap. A 1.6km mid-route retrace is
    // under 2400m and is accepted. (Pre-Phase 1.4 the cap was min(2000, 8%) =
    // 2000m; 1.6km was also under that; behaviour at this distance is
    // unchanged — locking the new formula.)
    int requested = 30000;
    OsmTrack t = synthLongLoopWithMidRetrace(requested, 1600);
    RoundTripQualityResult r = ReuseClassifier.classify(t, requested, false);
    assertTrue("30km loop with 1.6km mid retrace accepted: " + r, r.isAccepted());
  }

  /**
   * Build a synthetic triangle loop of {@code totalMeters} total length
   * containing one mid-route out-and-back detour. The detour goes off
   * the triangle side, then returns along the same edges — so the
   * BACK-leg of the detour counts as reuse. Setting {@code reusedMeters
   * = N} means the back-leg edges sum to ~N meters; the out-leg is
   * already there as part of the unique traversal.
   */
  private static OsmTrack synthLongLoopWithMidRetrace(int totalMeters, int reusedMeters) {
    int sideMeters = totalMeters / 3;
    // Detour radius: the cyclist goes mid→det (out) then det→mid (back),
    // making the back-edge length equal to reusedMeters. Both legs of the
    // detour are the same length, so detour radius = reusedMeters.
    int detourRadius = reusedMeters;
    int A_x = 0,                  A_y = 0;
    int B_x = sideMeters,         B_y = 0;
    int C_x = sideMeters / 2,     C_y = (int) (sideMeters * 0.866);
    int mid_x = (B_x + C_x) / 2;
    int mid_y = (B_y + C_y) / 2;
    // Detour direction: perpendicular to B→C so the detour edges don't
    // overlap the triangle side (otherwise the on-side reuse confuses
    // the classifier). Use the perpendicular unit vector.
    double bcDx = (C_x - B_x);
    double bcDy = (C_y - B_y);
    double bcLen = Math.sqrt(bcDx * bcDx + bcDy * bcDy);
    // Perpendicular: rotate (dx, dy) by 90°: (-dy, dx).
    double perpDx = -bcDy / bcLen;
    double perpDy = bcDx / bcLen;
    int det_x = mid_x + (int) (perpDx * detourRadius);
    int det_y = mid_y + (int) (perpDy * detourRadius);

    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    t.nodes.add(node(A_x, A_y));
    t.nodes.add(node(B_x, B_y));
    t.nodes.add(node(mid_x, mid_y));
    t.nodes.add(node(det_x, det_y));
    t.nodes.add(node(mid_x, mid_y)); // detour back — reuses mid→det edge
    t.nodes.add(node(C_x, C_y));
    t.nodes.add(node(A_x, A_y)); // close

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

  // ============ Gate-level integration tests ===============================
  // These verify the gate's evaluate() method composes correctly: hard
  // pre-checks + classifier. The classifier tests above cover the semantic
  // logic in isolation.

  @Test
  public void gateRejectsProfileHostileScenicSpur() {
    // Synthetic scenic-spur route but ALL edges carry highway=path. Even
    // though structurally it's a SCENIC_OUT_AND_BACK that allowSamewayback
    // would permit, fastbike must reject it as profile-hostile.
    OsmTrack t = outAndBack(6000);
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=path";
    m.costfactor = 5.0f;
    for (int i = 1; i < t.nodes.size(); i++) {
      MessageData copy = new MessageData();
      copy.wayKeyValues = m.wayKeyValues;
      copy.costfactor = m.costfactor;
      t.nodes.get(i).message = copy;
    }
    RoundTripQualityResult r = RoundTripQualityGate.evaluate(
      t, t.distance, "fastbike", /*allowSamewayback*/ true);
    assertFalse("profile-hostile spur rejected for fastbike even when "
      + "allowSamewayback=true: " + r, r.isAccepted());
    assertTrue("rejection mentions profile-hostile/path: " + r.getRejectionReason(),
      r.getRejectionReason().toLowerCase().contains("hostile")
        || r.getRejectionReason().toLowerCase().contains("path"));
  }

  @Test
  public void gateAcceptsLollipopOnGravel() {
    // The same lollipop on gravel: paved-hostility check is skipped for
    // non-paved profiles, so the lollipop is accepted.
    OsmTrack t = lollipop(2500, 5000);
    RoundTripQualityResult r = RoundTripQualityGate.evaluate(
      t, t.distance, "gravel", /*allowSamewayback*/ false);
    assertTrue("lollipop on gravel accepted: " + r, r.isAccepted());
    assertEquals(RouteShape.LOLLIPOP, r.getShape());
  }

  @Test
  public void gateLegacyValidateReturnsNullOnAccept() {
    OsmTrack t = cleanLoop(5000);
    String reason = RoundTripQualityGate.validate(t, t.distance, "fastbike");
    assertNull("legacy validate returns null on accept", reason);
  }

  @Test
  public void gateLegacyValidateReturnsReasonOnReject() {
    OsmTrack t = loopWithMidUTurn(3000);
    String reason = RoundTripQualityGate.validate(t, t.distance, "fastbike");
    assertNotNull("legacy validate returns reason on reject", reason);
  }
}
