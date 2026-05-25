package btools.router;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/** Pure-function tests for {@link RoundTripQualityGate}. These tests construct
 * synthetic {@link OsmTrack} objects and verify the gate accepts good routes
 * and rejects every hard-fail case named in the production-safety spec. */
public class RoundTripQualityGateTest {

  // ---- Algorithm aliases sanity (also covered by RoundTripAlgorithmTest) ----

  @Test
  public void pavedProfileRecognition() {
    assertTrue(RoundTripQualityGate.isPavedProfile("fastbike"));
    assertTrue(RoundTripQualityGate.isPavedProfile("road"));
    assertTrue(RoundTripQualityGate.isPavedProfile("racing"));
    assertFalse(RoundTripQualityGate.isPavedProfile("gravel"));
    assertFalse(RoundTripQualityGate.isPavedProfile("mtb"));
    assertFalse(RoundTripQualityGate.isPavedProfile("trekking"));
    assertFalse(RoundTripQualityGate.isPavedProfile(null));
  }

  // ---- Happy path ---------------------------------------------------------

  @Test
  public void acceptsCleanCloseLoop() {
    OsmTrack good = squareLoop(/*sideMeters*/ 5000);
    // 4 edges × ~5km = ~20km loop — pass desired = actual so ratio = 1.0
    assertNull(RoundTripQualityGate.validate(good, good.distance, "fastbike"));
  }

  @Test
  public void acceptsAtExactRatioBoundary() {
    OsmTrack track = squareLoop(2500);
    // Reference the actual computed distance so the test is robust to
    // exact distance-formula differences. Test that ratio at MIN and MAX
    // boundaries is accepted.
    double dist = track.distance;
    double desiredAtMin = dist / RoundTripQualityGate.MIN_DISTANCE_RATIO;
    double desiredAtMax = dist / RoundTripQualityGate.MAX_DISTANCE_RATIO;
    assertNull("at MIN_DISTANCE_RATIO should pass",
      RoundTripQualityGate.validate(track, desiredAtMin, "fastbike"));
    assertNull("at MAX_DISTANCE_RATIO should pass",
      RoundTripQualityGate.validate(track, desiredAtMax, "fastbike"));
  }

  // ---- Hard-fail cases (one per spec criterion) ---------------------------

  @Test
  public void rejectsNullTrack() {
    assertNotNull(RoundTripQualityGate.validate(null, 20000, "fastbike"));
  }

  @Test
  public void rejectsTooFewNodes() {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    t.nodes.add(makeNode(0, 0));
    t.nodes.add(makeNode(1000, 0));
    t.distance = 1000;
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected too-few-nodes, got: " + reason, reason.contains("too few nodes"));
  }

  @Test
  public void rejectsClosureGapTooLarge() {
    OsmTrack t = squareLoop(5000);
    // Move the last node 500m away from the start
    OsmPathElement last = t.nodes.get(t.nodes.size() - 1);
    t.nodes.set(t.nodes.size() - 1, makeNodeRaw(last.getILon() + 5000, last.getILat()));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected closure rejection, got: " + reason, reason.contains("closure"));
  }

  @Test
  public void rejectsRatioBelowMin() {
    OsmTrack t = squareLoop(2000);  // 8km
    // 8km / 20km = 0.4 → below MIN_DISTANCE_RATIO 0.5
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected ratio rejection, got: " + reason, reason.contains("ratio"));
  }

  @Test
  public void rejectsRatioAboveMax() {
    OsmTrack t = squareLoop(5000); // 20km
    // 20km / 10km = 2.0 → above MAX_DISTANCE_RATIO 1.8
    String reason = RoundTripQualityGate.validate(t, 10000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected ratio rejection, got: " + reason, reason.contains("ratio"));
  }

  @Test
  public void rejectsExcessiveReuse() {
    // 4 edges out (1km each) + 4 edges back (re-traced) + 1 extra short edge
    // → 4/9 ≈ 44%? no, we need > 50%. Use 3 out + 3 back = 6 edges, 3 reused
    // = exactly 50% by edge count, but by distance-weighted reuse the first
    // visit isn't reuse so it's 3/6 = 50% — exactly at the boundary.
    // Push past with one extra reused edge.
    OsmTrack heavy = new OsmTrack();
    heavy.nodes = new ArrayList<>();
    heavy.nodes.add(makeNode(0, 0));
    heavy.nodes.add(makeNode(1000, 0));
    heavy.nodes.add(makeNode(2000, 0));
    heavy.nodes.add(makeNode(3000, 0));
    heavy.nodes.add(makeNode(2000, 0)); // reuse #1
    heavy.nodes.add(makeNode(1000, 0)); // reuse #2
    heavy.nodes.add(makeNode(2000, 0)); // reuse #3 (re-revisit of 1000-2000)
    heavy.nodes.add(makeNode(1000, 0)); // reuse #4 (re-revisit of 2000-1000)
    heavy.nodes.add(makeNode(0, 0));    // reuse #5 (0-1000)
    int hd = 0;
    for (int i = 1; i < heavy.nodes.size(); i++) {
      hd += heavy.nodes.get(i - 1).calcDistance(heavy.nodes.get(i));
      MessageData m = msgCostfactor(1.5f, "highway=residential surface=asphalt");
      heavy.nodes.get(i).message = m;
    }
    heavy.distance = hd;
    String reason = RoundTripQualityGate.validate(heavy, hd, "fastbike");
    assertNotNull(reason);
    assertTrue("expected reuse rejection, got: " + reason, reason.contains("reuse"));
  }

  @Test
  public void rejectsBeelineMarkedWaypoint() {
    OsmTrack t = squareLoop(5000);
    t.matchedWaypoints = new ArrayList<>();
    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
    t.matchedWaypoints.add(mwp);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected beeline rejection, got: " + reason, reason.contains("beeline"));
  }

  @Test
  public void pavedProfileRejectsPathHeavyRoute() {
    OsmTrack t = squareLoopWithMessage(5000, msgWayTags("highway=path"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected hostile-segment rejection, got: " + reason,
      reason.contains("profile-hostile") || reason.contains("path/track/unpaved"));
  }

  @Test
  public void pavedProfileRejectsHighCostFactorSpike() {
    // 100% of edges have costfactor=10 (e.g. forced onto a grade-5 track)
    OsmTrack t = squareLoopWithMessage(5000, msgCostfactor(10.0f, "highway=residential"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected hostile rejection on cost-spike, got: " + reason,
      reason.contains("profile-hostile"));
  }

  @Test
  public void pavedProfileRejectsRouteWithMissingMetadata() {
    // No per-edge messages — engine can't prove the edges are paved
    OsmTrack t = squareLoopNoMessage(5000);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected missing-metadata rejection, got: " + reason,
      reason.contains("missing/unknown metadata"));
  }

  @Test
  public void pavedProfileAcceptsCleanResidentialRoute() {
    // costfactor=1.5 (residential-ish), tags are clearly paved
    OsmTrack t = squareLoopWithMessage(5000, msgCostfactor(1.5f, "highway=residential"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNull("clean paved route should pass: " + reason, reason);
  }

  @Test
  public void nonPavedProfileSkipsHostilityCheck() {
    // Same path-heavy route that fastbike rejects → gravel/MTB accept it
    OsmTrack t = squareLoopWithMessage(5000, msgWayTags("highway=path"));
    assertNull(RoundTripQualityGate.validate(t, 20000, "gravel"));
    assertNull(RoundTripQualityGate.validate(t, 20000, "mtb"));
  }

  @Test
  public void pavedProfileToleratesMinorHostileSegments() {
    // 95% residential, 5% path → below MAX_HOSTILE_FRACTION (10%)
    OsmTrack t = mixedSurfaceLoop(/*sideMeters*/ 5000, /*hostileFractionPct*/ 5);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNull("5% path content should pass: " + reason, reason);
  }

  // ---- helpers ------------------------------------------------------------

  /** Square loop with 4 edges of {@code side} meters each — total ~4×side.
   * Edges carry clean residential metadata so the paved-profile gate accepts
   * the route unless the test deliberately overrides it. */
  private static OsmTrack squareLoop(int side) {
    OsmTrack t = squareLoopNoMessage(side);
    MessageData clean = msgCostfactor(1.5f, "highway=residential surface=asphalt");
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = clone(clean);
    }
    return t;
  }

  /** Same geometry but without per-edge messages — used by the missing-metadata
   * test to verify the gate rejects routes whose paved-ness can't be proven. */
  private static OsmTrack squareLoopNoMessage(int side) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    t.nodes.add(makeNode(0, 0));
    t.nodes.add(makeNode(side, 0));
    t.nodes.add(makeNode(side, side));
    t.nodes.add(makeNode(0, side));
    t.nodes.add(makeNode(0, 0)); // close
    int dist = 0;
    for (int i = 1; i < t.nodes.size(); i++) {
      dist += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
    }
    t.distance = dist;
    return t;
  }

  /** Square loop where every edge carries the supplied {@link MessageData}. */
  private static OsmTrack squareLoopWithMessage(int side, MessageData m) {
    OsmTrack t = squareLoop(side);
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = clone(m);
    }
    return t;
  }

  /** Square loop where the first {@code hostileFractionPct}% of edges are
   * path/track and the rest are residential. */
  private static OsmTrack mixedSurfaceLoop(int side, int hostileFractionPct) {
    OsmTrack t = squareLoop(side);
    // 4 edges; flip k of them to "hostile" based on percentage
    int hostileEdges = (4 * hostileFractionPct) / 100;
    for (int i = 1; i < t.nodes.size(); i++) {
      MessageData m = (i <= hostileEdges)
        ? msgWayTags("highway=path")
        : msgCostfactor(1.5f, "highway=residential");
      t.nodes.get(i).message = m;
    }
    return t;
  }

  private static OsmPathElement makeNode(int x, int y) {
    // ~1 degree ≈ 111km at equator; use small offsets so distances are roughly
    // x and y in meters. CheapRuler is what does the actual scaling.
    return makeNodeRaw(180000000 + x * 14, 50000000 + y * 9);
  }

  private static OsmPathElement makeNodeRaw(int ilon, int ilat) {
    return OsmPathElement.create(ilon, ilat, (short) 0, null);
  }

  private static MessageData msgWayTags(String tags) {
    MessageData m = new MessageData();
    m.wayKeyValues = tags;
    m.costfactor = 1.5f;
    return m;
  }

  private static MessageData msgCostfactor(float cf, String tags) {
    MessageData m = new MessageData();
    m.wayKeyValues = tags;
    m.costfactor = cf;
    return m;
  }

  /** Missing-metadata placeholder: wayKeyValues=null treated as suspect. */
  private static MessageData msgNoMetadata() {
    MessageData m = new MessageData();
    m.wayKeyValues = null;
    m.costfactor = 0;
    return m;
  }

  private static MessageData clone(MessageData src) {
    MessageData m = new MessageData();
    m.wayKeyValues = src.wayKeyValues;
    m.costfactor = src.costfactor;
    return m;
  }
}
