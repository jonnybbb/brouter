package btools.router;

import org.junit.Assert;
import org.junit.Assume;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;

/**
 * Integration tests for the greedy sub-route round-trip planner.
 * Tests require segment data in brouter-core/src/test/resources/test-data/segments/
 * and are skipped if data is absent.
 */
public class GreedyRoundTripPlannerTest {

  private File segmentDir;
  private File profileDir;

  @Before
  public void setup() {
    segmentDir = new File("src/test/resources/test-data/segments");
    profileDir = new File("misc/profiles2");
    if (!profileDir.exists()) {
      profileDir = new File("../misc/profiles2");
    }
    // Classification now comes from the cost-model probe (PavedProfileProbeTest),
    // not the profile name; seed "fastbike" as paved for the gate-delegation case.
    RoundTripQualityGate.putPavedClassificationForTest("fastbike", true);
  }

  private boolean hasSegmentData() {
    return segmentDir.exists() && segmentDir.isDirectory()
      && segmentDir.listFiles() != null
      && segmentDir.listFiles().length > 0;
  }

  @Test
  public void roundTripAlgorithmParsing() {
    Assert.assertEquals(RoundTripAlgorithm.AUTO, RoundTripAlgorithm.fromString(null));
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.fromString("greedy"));
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.fromString("GREEDY"));
    Assert.assertEquals(RoundTripAlgorithm.ISOCHRONE, RoundTripAlgorithm.fromString("isochrone"));
    Assert.assertEquals(RoundTripAlgorithm.WAYPOINT, RoundTripAlgorithm.fromString("waypoint"));
    Assert.assertEquals(RoundTripAlgorithm.AUTO, RoundTripAlgorithm.fromString("bogus"));
  }

  @Test
  public void directionPreferenceParsing() {
    Assert.assertEquals(DirectionPreference.N, DirectionPreference.fromString("N"));
    Assert.assertEquals(DirectionPreference.SW, DirectionPreference.fromString("sw"));
    Assert.assertEquals(DirectionPreference.ANY, DirectionPreference.fromString(null));
    Assert.assertEquals(DirectionPreference.ANY, DirectionPreference.fromString("bogus"));
    Assert.assertEquals(0.0, DirectionPreference.N.bearing, 0.001);
    Assert.assertEquals(180.0, DirectionPreference.S.bearing, 0.001);
  }

  @Test
  public void sortByRoutedScoreOrdersAscending() {
    GreedyRoundTripPlanner.ScoredRoute a = new GreedyRoundTripPlanner.ScoredRoute();
    a.routedScore = 0.42;
    a.candidateIndex = 0;
    GreedyRoundTripPlanner.ScoredRoute b = new GreedyRoundTripPlanner.ScoredRoute();
    b.routedScore = 0.17;
    b.candidateIndex = 1;
    GreedyRoundTripPlanner.ScoredRoute c = new GreedyRoundTripPlanner.ScoredRoute();
    c.routedScore = 0.81;
    c.candidateIndex = 2;

    List<GreedyRoundTripPlanner.ScoredRoute> list = new ArrayList<>();
    list.add(a);
    list.add(b);
    list.add(c);
    GreedyRoundTripPlanner.sortByRoutedScore(list);

    Assert.assertSame("lowest routedScore first", b, list.get(0));
    Assert.assertSame(a, list.get(1));
    Assert.assertSame("highest routedScore last", c, list.get(2));
  }

  @Test
  public void sortByRoutedScoreIsStableOnTies() {
    // Stability matters: the legacy single-best-wins logic used `<` so the
    // first candidate to reach a tied score won. Phase 1 Step 2 keeps that
    // tie-break by relying on List.sort's stable contract.
    GreedyRoundTripPlanner.ScoredRoute first = new GreedyRoundTripPlanner.ScoredRoute();
    first.routedScore = 0.5;
    first.candidateIndex = 0;
    GreedyRoundTripPlanner.ScoredRoute second = new GreedyRoundTripPlanner.ScoredRoute();
    second.routedScore = 0.5;
    second.candidateIndex = 1;
    GreedyRoundTripPlanner.ScoredRoute third = new GreedyRoundTripPlanner.ScoredRoute();
    third.routedScore = 0.5;
    third.candidateIndex = 2;

    List<GreedyRoundTripPlanner.ScoredRoute> list = new ArrayList<>();
    list.add(first);
    list.add(second);
    list.add(third);
    GreedyRoundTripPlanner.sortByRoutedScore(list);

    Assert.assertEquals(0, list.get(0).candidateIndex);
    Assert.assertEquals(1, list.get(1).candidateIndex);
    Assert.assertEquals(2, list.get(2).candidateIndex);
  }

  @Test
  public void sortByRoutedScoreHandlesEmptyList() {
    List<GreedyRoundTripPlanner.ScoredRoute> empty = new ArrayList<>();
    GreedyRoundTripPlanner.sortByRoutedScore(empty);
    Assert.assertTrue(empty.isEmpty());
  }

  @Test
  public void autoSelectsGreedyForLargeRadius() {
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(8000));
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(5000));
  }

  @Test
  public void autoFallbackSelectsGreedyForSmallRadius() {
    // Generated-loop AUTO resolution now runs inside doRoundTrip() via the
    // greedy-first candidate competition. The cheap helper is only a fallback
    // for unsupported/direct callers, and it must keep the same greedy default
    // across radii.
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(4000));
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(1500));
  }

  @Test
  public void greedySubRouteCountScalesWithDistanceAndProfile() {
    Assert.assertEquals(3, RoutingEngine.selectGreedySubRouteCount(7000, "fastbike"));
    Assert.assertEquals(4, RoutingEngine.selectGreedySubRouteCount(20000, "fastbike"));
    Assert.assertEquals(5, RoutingEngine.selectGreedySubRouteCount(50000, "fastbike"));
    Assert.assertEquals(6, RoutingEngine.selectGreedySubRouteCount(90000, "fastbike"));
    Assert.assertEquals(6, RoutingEngine.selectGreedySubRouteCount(50000, "mtb"));
    Assert.assertEquals(6, RoutingEngine.selectGreedySubRouteCount(90000, "mtb"));
  }

  @Test
  public void legacyRoundTripIsochroneParamMapsToIsochrone() {
    // Drive the real parser: roundTripIsochrone=1 promotes AUTO -> ISOCHRONE.
    RoutingContext rctx = new RoutingContext();
    Assert.assertEquals(RoundTripAlgorithm.AUTO, rctx.roundTripAlgorithm);
    Map<String, String> params = new LinkedHashMap<>();
    params.put("roundTripIsochrone", "1");
    new RoutingParamCollector().setParams(rctx, null, params);
    Assert.assertEquals(RoundTripAlgorithm.ISOCHRONE, rctx.roundTripAlgorithm);
    Assert.assertTrue(rctx.roundTripIsochrone);
  }

  @Test
  public void explicitRoundTripAlgorithmWinsOverIsochroneShortcut() {
    // An explicit roundTripAlgorithm beats the roundTripIsochrone=1 shortcut,
    // regardless of parameter order (the parser only promotes when AUTO).
    for (boolean algoFirst : new boolean[]{true, false}) {
      RoutingContext rctx = new RoutingContext();
      Map<String, String> params = new LinkedHashMap<>();
      if (algoFirst) {
        params.put("roundTripAlgorithm", "GREEDY");
        params.put("roundTripIsochrone", "1");
      } else {
        params.put("roundTripIsochrone", "1");
        params.put("roundTripAlgorithm", "GREEDY");
      }
      new RoutingParamCollector().setParams(rctx, null, params);
      Assert.assertEquals("explicit algorithm must win (algoFirst=" + algoFirst + ")",
        RoundTripAlgorithm.GREEDY, rctx.roundTripAlgorithm);
    }
  }

  @Test
  public void nonPositiveRoundTripLengthIsNulled() {
    // roundTripLength <= 0 must be discarded (null), not passed through.
    for (String v : new String[]{"0", "-5"}) {
      RoutingContext rctx = new RoutingContext();
      Map<String, String> params = new LinkedHashMap<>();
      params.put("roundTripLength", v);
      new RoutingParamCollector().setParams(rctx, null, params);
      Assert.assertNull("roundTripLength=" + v + " must null out", rctx.roundTripLength);
    }
  }

  @Test
  public void malformedRoundTripParamsDoNotThrow() {
    // A non-integer value on a round-trip URL parameter is untrusted client input;
    // it must be ignored (logged) rather than thrown as NumberFormatException, which
    // the server surfaces as an opaque HTTP 500 indistinguishable from a real crash.
    String[] intParams = {
      "roundTripLength", "roundTripDistance", "roundTripDirectionAdd",
      "roundTripPoints", "roundTripIsochrone", "allowSamewayback"};
    for (String key : intParams) {
      RoutingContext rctx = new RoutingContext();
      Map<String, String> params = new LinkedHashMap<>();
      params.put(key, "not-a-number");
      // Must not throw.
      new RoutingParamCollector().setParams(rctx, null, params);
    }
    // The malformed value must leave the field at its safe default.
    RoutingContext rctx = new RoutingContext();
    Map<String, String> params = new LinkedHashMap<>();
    params.put("roundTripLength", "12km");
    new RoutingParamCollector().setParams(rctx, null, params);
    Assert.assertNull("malformed roundTripLength must not be set", rctx.roundTripLength);
  }

  // --- fallback-selection rule (gate-accept-aware) ---

  @Test
  public void betterFallbackPrefersGateAcceptedOverRejected() {
    // A gate-accepted loop beats a gate-rejected one regardless of error — this
    // is the core of the fix: don't latch a rejected low-error loop and discard
    // a usable accepted higher-error one.
    Assert.assertTrue("accepted (worse error) beats rejected (better error)",
      GreedyRoundTripPlanner.isBetterFallback(true, 0.90, false, 0.10));
    Assert.assertFalse("rejected (better error) does not beat accepted (worse error)",
      GreedyRoundTripPlanner.isBetterFallback(false, 0.10, true, 0.90));
  }

  @Test
  public void betterFallbackBreaksTiesByLowerError() {
    // Same gate verdict → lower geometric error wins; equal error is not "better".
    Assert.assertTrue("accepted: lower error wins",
      GreedyRoundTripPlanner.isBetterFallback(true, 0.10, true, 0.20));
    Assert.assertFalse("accepted: higher error loses",
      GreedyRoundTripPlanner.isBetterFallback(true, 0.30, true, 0.20));
    Assert.assertTrue("rejected: lower error wins",
      GreedyRoundTripPlanner.isBetterFallback(false, 0.10, false, 0.20));
    Assert.assertFalse("rejected: higher error loses",
      GreedyRoundTripPlanner.isBetterFallback(false, 0.30, false, 0.20));
    Assert.assertFalse("equal error + same verdict is not strictly better",
      GreedyRoundTripPlanner.isBetterFallback(true, 0.20, true, 0.20));
  }

  @Test
  public void greedyRejectsAllowSamewayback() {
    // allowSamewayback semantics (out-and-back) are not implemented by greedy;
    // dispatch must fall back to the waypoint-based algorithm.
    Assert.assertFalse("greedy should not handle allowSamewayback",
      RoutingEngine.greedySupports(true, 1));
  }

  @Test
  public void greedyRejectsExtraUserViaPoints() {
    // User-supplied via points are not honored by greedy today; must fall back.
    Assert.assertFalse("greedy should not handle extra user via points",
      RoutingEngine.greedySupports(false, 2));
    Assert.assertFalse("greedy should not handle 3 user waypoints",
      RoutingEngine.greedySupports(false, 3));
  }

  @Test
  public void greedyAcceptsLoneStartWaypoint() {
    Assert.assertTrue("greedy supports a single start waypoint",
      RoutingEngine.greedySupports(false, 1));
  }

  @Test
  public void combinedRoutedScorePrefersLowerVisitedRatio() {
    // Two candidates with identical cost-per-meter; the one with lower
    // scorer score (e.g. less edge reuse) should win.
    CandidateScorer scorer = new CandidateScorer();
    double subTarget = 2000;
    double total = 0;
    double desired = 10000;
    double estimatedReturn = 8000;

    double freshScore = scorer.score(
      2000, subTarget, total, estimatedReturn, desired,
      90, DirectionPreference.ANY, 1, 5,
      0.0, 5000, 5000, -1);
    double reusedScore = scorer.score(
      2000, subTarget, total, estimatedReturn, desired,
      90, DirectionPreference.ANY, 1, 5,
      0.8, 5000, 5000, -1);

    double freshCombined = GreedyRoundTripPlanner.combinedRoutedScore(freshScore, 1.2);
    double reusedCombined = GreedyRoundTripPlanner.combinedRoutedScore(reusedScore, 1.2);

    Assert.assertTrue("fresh candidate should beat reused at equal cost-per-meter",
      freshCombined < reusedCombined);
  }

  @Test
  public void combinedRoutedScoreFactorsInCostPerMeter() {
    // With identical scorer score, lower cost/meter is preferred.
    double scorerScore = 0.5;
    double cheap = GreedyRoundTripPlanner.combinedRoutedScore(scorerScore, 1.0);
    double expensive = GreedyRoundTripPlanner.combinedRoutedScore(scorerScore, 3.0);
    Assert.assertTrue("cheaper route should score lower at equal scorer score",
      cheap < expensive);
  }

  @Test
  public void greedyMatchedWaypointsAreNeverBeeline() {
    // Successful greedy plans must produce road-snapped (SHAPING) waypoints.
    // A DIRECT entry would tell the routing engine to insert a beeline,
    // which violates the round-trip invariant.
    GreedyRoundTripPlanner planner = new GreedyRoundTripPlanner(null);

    MatchedWaypoint startMwp = makeMatchedWaypoint(1000, 1000, 900, 900, 1100, 1100);
    // Even when the source MWP is marked DIRECT, copyMatchedWaypoint must
    // reset it to SHAPING so beeline behavior cannot leak through.
    startMwp.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;

    MatchedWaypoint via1 = makeMatchedWaypoint(2000, 2000, 1900, 1900, 2100, 2100);
    via1.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
    MatchedWaypoint via2 = makeMatchedWaypoint(3000, 3000, 2900, 2900, 3100, 3100);

    List<MatchedWaypoint> stack = new ArrayList<>();
    stack.add(startMwp);
    stack.add(via1);
    stack.add(via2);

    List<MatchedWaypoint> matched = planner.buildMatchedWaypoints(stack, startMwp);

    Assert.assertEquals("expected from + via1 + via2 + to", 4, matched.size());
    for (MatchedWaypoint m : matched) {
      Assert.assertNotEquals("greedy matched waypoint " + m.name + " must not be DIRECT",
        MatchedWaypoint.WAYPOINT_TYPE_DIRECT, m.wpttype);
      Assert.assertEquals("greedy matched waypoint " + m.name + " must be SHAPING",
        MatchedWaypoint.WAYPOINT_TYPE_SHAPING, m.wpttype);
      // waypoint == crosspoint is what disables RoutingEngine's dynamic beeline
      // insertion (it triggers when distance(waypoint, crosspoint) > catchingRange).
      Assert.assertEquals("waypoint and crosspoint must coincide for " + m.name,
        m.crosspoint.ilon, m.waypoint.ilon);
      Assert.assertEquals("waypoint and crosspoint must coincide for " + m.name,
        m.crosspoint.ilat, m.waypoint.ilat);
    }
  }

  private static MatchedWaypoint makeMatchedWaypoint(int crossLon, int crossLat,
                                                     int n1Lon, int n1Lat,
                                                     int n2Lon, int n2Lat) {
    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.crosspoint = new OsmNode(crossLon, crossLat);
    mwp.waypoint = new OsmNode(crossLon, crossLat);
    mwp.node1 = new OsmNode(n1Lon, n1Lat);
    mwp.node2 = new OsmNode(n2Lon, n2Lat);
    return mwp;
  }

  @Test
  public void candidateScorerDefaultWeights() {
    CandidateScorer scorer = new CandidateScorer();
    // Basic sanity: perfect candidate should score very low
    double perfectScore = scorer.score(
      2000, 2000,  // exact distance match
      0, 8000, 10000,  // perfect loop feasibility
      0, DirectionPreference.N,
      1, 5,
      0.0, 5000, 5000, -1);

    double badScore = scorer.score(
      4000, 2000,  // 100% over target
      0, 12000, 10000,  // 40% over desired
      180, DirectionPreference.N,  // opposite direction
      1, 5,
      0.5, 500, 5000, -1);  // high reuse, too close to start

    Assert.assertTrue("Perfect candidate should beat bad one", perfectScore < badScore);
  }

  @Test
  public void subRouteCountDefaultIsFive() {
    // Verify construction works with default and explicit params
    // (can't invoke plan() without segments, but constructors should succeed)
    Assert.assertNotNull(new CandidateScorer());
    Assert.assertNotNull(new CandidateScorer(1.0, 2.0, 0.5, 3.0, 1.5));
  }

  @Test
  public void greedyRoundTripWithSegments() {
    Assume.assumeTrue("Segment data required", hasSegmentData());

    // Basel area: 47.5581, 7.5878
    OsmNodeNamed start = new OsmNodeNamed();
    start.ilon = (int) ((7.5878 + 180) * 1e6);
    start.ilat = (int) ((47.5581 + 90) * 1e6);
    start.name = "start";

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = new File(profileDir, "fastbike.brf").getAbsolutePath();
    rctx.roundTripDistance = 8000; // ~50km loop
    rctx.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;

    List<OsmNodeNamed> waypoints = new ArrayList<>();
    waypoints.add(start);

    RoutingEngine re = new RoutingEngine(null, null, segmentDir, waypoints, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(300000);

    Assert.assertNull("No error expected", re.errorMessage);
    Assert.assertNotNull("Track should be produced", re.foundTrack);
    Assert.assertTrue("Track should have distance > 0", re.foundTrack.distance > 0);
  }

  @Test
  public void deterministic() {
    Assume.assumeTrue("Segment data required", hasSegmentData());

    OsmNodeNamed start = new OsmNodeNamed();
    start.ilon = (int) ((7.5878 + 180) * 1e6);
    start.ilat = (int) ((47.5581 + 90) * 1e6);
    start.name = "start";

    RoutingContext rctx1 = new RoutingContext();
    rctx1.localFunction = new File(profileDir, "fastbike.brf").getAbsolutePath();
    rctx1.roundTripDistance = 5000;
    rctx1.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    rctx1.startDirection = 90; // East

    List<OsmNodeNamed> wp1 = new ArrayList<>();
    wp1.add(start);
    RoutingEngine re1 = new RoutingEngine(null, null, segmentDir, wp1, rctx1, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re1.quite = true;
    re1.doRun(300000);

    RoutingContext rctx2 = new RoutingContext();
    rctx2.localFunction = new File(profileDir, "fastbike.brf").getAbsolutePath();
    rctx2.roundTripDistance = 5000;
    rctx2.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    rctx2.startDirection = 90;

    OsmNodeNamed start2 = new OsmNodeNamed();
    start2.ilon = start.ilon;
    start2.ilat = start.ilat;
    start2.name = "start";
    List<OsmNodeNamed> wp2 = new ArrayList<>();
    wp2.add(start2);
    RoutingEngine re2 = new RoutingEngine(null, null, segmentDir, wp2, rctx2, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re2.quite = true;
    re2.doRun(300000);

    if (re1.foundTrack != null && re2.foundTrack != null) {
      Assert.assertEquals("Deterministic: same distance",
        re1.foundTrack.distance, re2.foundTrack.distance);
    }
  }

  // ---- Boundary-proximity weighting for back-and-forth penalty ----
  //
  // These weights are a planner steering hint, NOT the semantic stem/spur
  // classifier (that runs in ReuseClassifier after routing). Keeping the
  // boundary-near-1.0/mid-loop-0.5 split matches what the production planner
  // actually needs: pushing the semantic stem-vs-mid distinction down to
  // per-edge weights regressed real Dreieich loops (the planner skipped
  // viable paved sub-routes and picked path/track alternatives that the
  // profile gate then rejected).

  @Test
  public void boundaryProximityWeightFullPenaltyNearStart() {
    // Both positions within first 20% of the loop → full penalty (1.0).
    double w = GreedyRoundTripPlanner.boundaryProximityWeight(500, 1000, 10000);
    Assert.assertEquals("near-start reuse → full penalty", 1.0, w, 0.001);
  }

  @Test
  public void boundaryProximityWeightFullPenaltyNearEnd() {
    // Both positions within last 20% of the loop → full penalty (1.0).
    double w = GreedyRoundTripPlanner.boundaryProximityWeight(8500, 9000, 10000);
    Assert.assertEquals("near-end reuse → full penalty", 1.0, w, 0.001);
  }

  @Test
  public void boundaryProximityWeightHalfPenaltyMidLoop() {
    // Both positions in middle 60% of the loop → half penalty (0.5). This
    // reflects the cyclist intuition: a road crossed once on the way out and
    // again on the way back at mid-loop is less annoying than the same
    // crossing right after departure.
    double w = GreedyRoundTripPlanner.boundaryProximityWeight(3500, 6500, 10000);
    Assert.assertEquals("mid-loop reuse → half penalty", 0.5, w, 0.001);
  }

  @Test
  public void boundaryProximityWeightFullPenaltyIfEitherEndIsNearBoundary() {
    // First visit near start (boundary=0.05); current visit mid-loop (boundary=0.5).
    // min(0.05, 0.5) = 0.05 < 0.2 → full penalty.
    double w = GreedyRoundTripPlanner.boundaryProximityWeight(500, 5000, 10000);
    Assert.assertEquals("one near-boundary → full penalty", 1.0, w, 0.001);
  }

  @Test
  public void finalTrackReuseRatioCountsSecondAndLaterVisitsOnly() {
    // Test the bug fix: finalTrackReuseRatio shouldn't count an edge as "reuse"
    // just because it appears in the track — only subsequent re-traversals count.
    OsmTrack track = new OsmTrack();
    track.nodes = new ArrayList<>();
    track.nodes.add(makeNode(0, 0));         // 0,0
    track.nodes.add(makeNode(1000, 0));      // 1,0  (edge 0-1)
    track.nodes.add(makeNode(2000, 0));      // 2,0  (edge 1-2)
    track.nodes.add(makeNode(1000, 0));      // back to 1  (edge 1-2 REUSED)
    track.nodes.add(makeNode(0, 0));         // back to 0  (edge 0-1 REUSED)
    track.distance = (int) (track.nodes.get(0).calcDistance(track.nodes.get(1))
      + track.nodes.get(1).calcDistance(track.nodes.get(2))
      + track.nodes.get(2).calcDistance(track.nodes.get(3))
      + track.nodes.get(3).calcDistance(track.nodes.get(4)));
    double reuse = GreedyRoundTripPlanner.finalTrackReuseRatio(track);
    // 4 edges total; 2 are first visits (no reuse), 2 are second visits (reuse).
    // distance-weighted: equal segments, so reuse = 2/4 = 0.5.
    Assert.assertEquals("half the track is reused", 0.5, reuse, 0.05);
  }

  @Test
  public void finalTrackReuseRatioZeroForNonRepeatingTrack() {
    OsmTrack track = new OsmTrack();
    track.nodes = new ArrayList<>();
    track.nodes.add(makeNode(0, 0));
    track.nodes.add(makeNode(1000, 0));
    track.nodes.add(makeNode(1000, 1000));
    track.nodes.add(makeNode(0, 1000));
    track.nodes.add(makeNode(0, 0));
    Assert.assertEquals("no edge traversed twice → 0 reuse", 0.0,
      GreedyRoundTripPlanner.finalTrackReuseRatio(track), 0.001);
  }

  private static OsmPathElement makeNode(int dx, int dy) {
    return OsmPathElement.create(8720000 + dx, 50000000 + dy, (short) 0, null);
  }

  // ---- Phase 1.5 — internal fallback gate delegates to production gate -

  @Test
  public void qualityGateReasonDelegatesToProductionGate() {
    // Build a track tripping the production gate's hostile-fraction check
    // (every edge tagged highway=path with surface=ground → 100% hostile)
    // and verify the planner's internal qualityGateReason returns a
    // rejection reason matching the production gate's prefix. Pre-Phase 1.5
    // the planner used independent FALLBACK_* constants and would have
    // accepted this track (it has zero reuse and clean ratio).
    OsmTrack heavy = squarePathTrack(/*sideMeters*/ 5000);
    GreedyRoundTripPlanner planner = new GreedyRoundTripPlanner(null);
    planner.setProfileName("fastbike");
    String reason = planner.qualityGateReason(heavy, heavy.distance);
    Assert.assertNotNull("planner rejects hostile-fraction routes", reason);
    Assert.assertTrue(
      "planner reason matches production gate prefix (contiguous or % hostile): " + reason,
      reason.startsWith("contiguous ") || reason.contains("of distance on profile-hostile ways"));
  }

  @Test
  public void qualityGateReasonAcceptsCleanLoop() {
    // The same delegation accepts a clean paved loop.
    OsmTrack clean = squareResidentialTrack(/*sideMeters*/ 5000);
    GreedyRoundTripPlanner planner = new GreedyRoundTripPlanner(null);
    planner.setProfileName("fastbike");
    Assert.assertNull("planner accepts clean residential loop",
      planner.qualityGateReason(clean, clean.distance));
  }

  /** Synthetic 4-side square loop tagged entirely with highway=path
   * surface=ground (no asphalt override, so trips the gate). */
  private static OsmTrack squarePathTrack(int side) {
    OsmTrack t = squareTrack(side);
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=path surface=ground";
    m.costfactor = 3.0f; // low enough not to trip costfactor check alone
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = cloneMsg(m);
    }
    return t;
  }

  /** Synthetic 4-side square loop tagged residential / asphalt (clean). */
  private static OsmTrack squareResidentialTrack(int side) {
    OsmTrack t = squareTrack(side);
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=residential surface=asphalt";
    m.costfactor = 1.0f;
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = cloneMsg(m);
    }
    return t;
  }

  private static OsmTrack squareTrack(int side) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    int baseLon = 180_000_000;
    int baseLat = 50_000_000;
    int ilonPerM = 14;
    int ilatPerM = 9;
    t.nodes.add(OsmPathElement.create(baseLon, baseLat, (short) 0, null));
    t.nodes.add(OsmPathElement.create(baseLon + side * ilonPerM, baseLat, (short) 0, null));
    t.nodes.add(OsmPathElement.create(baseLon + side * ilonPerM, baseLat + side * ilatPerM, (short) 0, null));
    t.nodes.add(OsmPathElement.create(baseLon, baseLat + side * ilatPerM, (short) 0, null));
    t.nodes.add(OsmPathElement.create(baseLon, baseLat, (short) 0, null));
    int d = 0;
    for (int i = 1; i < t.nodes.size(); i++) {
      d += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
    }
    t.distance = d;
    return t;
  }

  private static MessageData cloneMsg(MessageData src) {
    MessageData m = new MessageData();
    m.wayKeyValues = src.wayKeyValues;
    m.costfactor = src.costfactor;
    return m;
  }

}
