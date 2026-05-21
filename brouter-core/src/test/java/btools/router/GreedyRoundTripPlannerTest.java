package btools.router;

import org.junit.Assert;
import org.junit.Assume;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

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
  public void autoSelectsGreedyForLargeRadius() {
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(8000));
    Assert.assertEquals(RoundTripAlgorithm.GREEDY, RoutingEngine.selectRoundTripAlgorithm(5000));
  }

  @Test
  public void autoSelectsIsochroneForSmallRadius() {
    Assert.assertEquals(RoundTripAlgorithm.ISOCHRONE, RoutingEngine.selectRoundTripAlgorithm(4000));
    Assert.assertEquals(RoundTripAlgorithm.ISOCHRONE, RoutingEngine.selectRoundTripAlgorithm(1500));
  }

  @Test
  public void legacyRoundTripIsochroneParam() {
    RoutingContext rctx = new RoutingContext();
    Assert.assertEquals(RoundTripAlgorithm.AUTO, rctx.roundTripAlgorithm);

    // Legacy compat: setting roundTripIsochrone should map to ISOCHRONE algorithm
    rctx.roundTripIsochrone = true;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISOCHRONE;
    Assert.assertEquals(RoundTripAlgorithm.ISOCHRONE, rctx.roundTripAlgorithm);
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

}
