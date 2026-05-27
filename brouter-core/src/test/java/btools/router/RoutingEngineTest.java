package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

public class RoutingEngineTest {
  private static final int START_ILON = 188720000; // ~8.72E
  private static final int START_ILAT = 140000000; // ~50.0N

  @Rule
  public TemporaryFolder outputDir = new TemporaryFolder();

  private File projectDir;

  @Before
  public void before() throws Exception {
    // Gradle sets cwd to the module directory (brouter-core/)
    projectDir = new File(".").getCanonicalFile().getParentFile();
  }

  @Test
  public void routeCrossingSegmentBorder() throws Exception {
    // Copy reference track into temp dir so engine finds it for comparison and produces an alternative
    copyResourceToDir("/testtrack0.gpx", outputDir.getRoot());
    String msg = calcRoute(8.720897, 50.002515, 8.723658, 49.997510, outputDir.getRoot(), "testtrack", new RoutingContext());
    Assert.assertNull("routing failed: " + msg, msg);

    File a1 = new File(outputDir.getRoot(), "testtrack1.gpx");
    Assert.assertTrue("result content mismatch", a1.exists());
  }

  @Test
  public void routeDestinationPointFarOff() {
    String msg = calcRoute(8.720897, 50.002515, 16.723658, 49.997510, outputDir.getRoot(), "notrack", new RoutingContext());
    Assert.assertTrue(msg, msg != null && msg.contains("not found"));
  }

  // check that a (short) route and an alternative route can be computed
  // while explicitely overriding a routing profile parameter
  @Test
  public void overrideParam() {
    // 1st route computing (with param) — writes paramTrack0.gpx
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    rctx.keyValues.put("avoid_unsafe", "1.0");
    String msg = calcRoute(8.723037, 50.000491, 8.712737, 50.002899, outputDir.getRoot(), "paramTrack", rctx);
    Assert.assertNull("routing failed (paramTrack 1st route): " + msg, msg);
    // 2nd route computing (same from/to & same param) — finds paramTrack0.gpx, produces alternative
    rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    rctx.keyValues.put("avoid_unsafe", "1.0");
    msg = calcRoute(8.723037, 50.000491, 8.712737, 50.002899, outputDir.getRoot(), "paramTrack", rctx);
    Assert.assertNull("routing failed (paramTrack 2nd route): " + msg, msg);

    File trackFile = new File(outputDir.getRoot(), "paramTrack1.gpx");
    Assert.assertTrue("result content mismatch", trackFile.exists());
  }

  // Round-trip from center of test area should produce a valid loop
  @Test
  public void roundTripBasicLoop() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0; // north
    rctx.roundTripDistance = 1000;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtBasic", rctx);

    RoundTripFixture.assertNoEngineErrorOrSkip(re, "round-trip routing");
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull("round-trip should produce a track", track);
    Assert.assertTrue("round-trip track should have nodes", track.nodes.size() > 2);

    // track should start and end near the origin
    OsmPathElement first = track.nodes.get(0);
    OsmPathElement last = track.nodes.get(track.nodes.size() - 1);
    int closingDistance = first.calcDistance(last);
    Assert.assertTrue("loop should close near origin, but gap is " + closingDistance + "m",
      closingDistance < 500);

    // verify no micro-detours remain: no node should appear twice within 350m of track distance
    Map<Long, Integer> lastSeen = new HashMap<>();
    int cumDist = 0;
    for (int i = 0; i < track.nodes.size(); i++) {
      if (i > 0) cumDist += track.nodes.get(i).calcDistance(track.nodes.get(i - 1));
      long id = track.nodes.get(i).getIdFromPos();
      Integer prevDist = lastSeen.put(id, cumDist);
      if (prevDist != null) {
        int loopDist = cumDist - prevDist;
        Assert.assertTrue("micro-detour found: node " + id + " revisited after " + loopDist + "m",
          loopDist > 350);
      }
    }
  }

  // Round-trip with large radius pushing waypoints outside road data area
  // Near the data edge the generated waypoints are filtered; if fewer than the loop
  // minimum (2 intermediate) remain, the engine must fail cleanly rather than emit a
  // degenerate single-waypoint out-and-back.
  @Test
  public void roundTripFailsCleanlyWhenDataEdgeFiltersWaypoints() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 90; // east, dreieich data runs out quickly
    rctx.roundTripDistance = 5000;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtEdge", rctx);

    Assert.assertNotNull("expected a clean failure at the data edge", re.getErrorMessage());
    Assert.assertTrue("error should explain the waypoint shortfall: " + re.getErrorMessage(),
      re.getErrorMessage().contains("form a loop"));
    Assert.assertNull("no track should be returned on failure", re.getFoundTrack());
  }

  // filterRoundTripWaypoints removes waypoints that snapped too far
  @Test
  public void filterRemovesDistantSnaps() {
    double searchRadius = 5000;
    RoutingEngine re = createDummyEngine(searchRadius);

    // Build matched waypoints: start, 4 intermediates, end (return to start)
    List<MatchedWaypoint> wpts = new ArrayList<>();
    int startIlon = START_ILON;
    int startIlat = START_ILAT;

    // start waypoint
    wpts.add(createMatchedWaypoint("from", startIlon, startIlat, startIlon, startIlat));
    // rt1: matched close (500m) — should be kept
    wpts.add(createMatchedWaypoint("rt1", startIlon + 7000, startIlat + 3000,
      startIlon + 7200, startIlat + 3100));
    // rt2: waypoint 60000 east, crosspoint 10000 east → snap ~3.5km > 2500m threshold
    wpts.add(createMatchedWaypoint("rt2", startIlon + 60000, startIlat,
      startIlon + 10000, startIlat));
    // rt3: waypoint 55000 east, crosspoint 8000 east → snap ~3.3km > 2500m threshold
    wpts.add(createMatchedWaypoint("rt3", startIlon + 55000, startIlat - 3000,
      startIlon + 8000, startIlat - 2000));
    // rt4: matched close (300m) — should be kept
    wpts.add(createMatchedWaypoint("rt4", startIlon + 3000, startIlat - 5000,
      startIlon + 3200, startIlat - 5100));
    // end waypoint (return to start)
    wpts.add(createMatchedWaypoint("to_rt", startIlon, startIlat, startIlon, startIlat));

    re.filterRoundTripWaypoints(wpts);

    Assert.assertEquals("should have 4 waypoints (start, rt1, rt4, end)", 4, wpts.size());
    Assert.assertEquals("from", wpts.get(0).name);
    Assert.assertEquals("rt1", wpts.get(1).name);
    Assert.assertEquals("rt4", wpts.get(2).name);
    Assert.assertEquals("to_rt", wpts.get(3).name);
  }

  // filterRoundTripWaypoints removes consecutive waypoints that matched too close
  @Test
  public void filterRemovesCloseConsecutive() {
    double searchRadius = 5000;
    RoutingEngine re = createDummyEngine(searchRadius);

    List<MatchedWaypoint> wpts = new ArrayList<>();
    int startIlon = START_ILON;
    int startIlat = START_ILAT;

    wpts.add(createMatchedWaypoint("from", startIlon, startIlat, startIlon, startIlat));
    // rt1 and rt2: crosspoints only 100m apart (< 500m threshold)
    wpts.add(createMatchedWaypoint("rt1", startIlon + 5000, startIlat + 5000,
      startIlon + 5000, startIlat + 5000));
    wpts.add(createMatchedWaypoint("rt2", startIlon + 5100, startIlat + 5100,
      startIlon + 5100, startIlat + 5100));
    // rt3: far enough from rt1 — should be kept
    wpts.add(createMatchedWaypoint("rt3", startIlon + 3000, startIlat - 5000,
      startIlon + 3000, startIlat - 5000));
    wpts.add(createMatchedWaypoint("to_rt", startIlon, startIlat, startIlon, startIlat));

    re.filterRoundTripWaypoints(wpts);

    Assert.assertEquals("should have 4 waypoints (start, rt1, rt3, end)", 4, wpts.size());
    Assert.assertEquals("rt1", wpts.get(1).name);
    Assert.assertEquals("rt3", wpts.get(2).name);
  }

  // filterRoundTripWaypoints preserves at least one intermediate waypoint
  @Test
  public void filterPreservesMinimumOneIntermediate() {
    double searchRadius = 1000;
    RoutingEngine re = createDummyEngine(searchRadius);

    List<MatchedWaypoint> wpts = new ArrayList<>();
    int startIlon = START_ILON;
    int startIlat = START_ILAT;

    wpts.add(createMatchedWaypoint("from", startIlon, startIlat, startIlon, startIlat));
    // all intermediates snap far — but one must be preserved
    wpts.add(createMatchedWaypoint("rt1", startIlon + 20000, startIlat,
      startIlon + 5000, startIlat));
    wpts.add(createMatchedWaypoint("rt2", startIlon, startIlat + 20000,
      startIlon, startIlat + 5000));
    wpts.add(createMatchedWaypoint("to_rt", startIlon, startIlat, startIlon, startIlat));

    re.filterRoundTripWaypoints(wpts);

    // at least one rt waypoint must remain
    int rtCount = 0;
    for (MatchedWaypoint mwp : wpts) {
      if (mwp.name != null && mwp.name.startsWith("rt")) rtCount++;
    }
    Assert.assertTrue("must preserve at least one intermediate waypoint", rtCount >= 1);
    Assert.assertTrue("total waypoints >= 3 (start + 1 intermediate + end)", wpts.size() >= 3);
  }

  // allowSamewayback round-trip should produce valid out-and-back route
  @Test
  public void roundTripAllowSamewayback() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0; // north
    rctx.roundTripDistance = 1000;
    rctx.allowSamewayback = true;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtSameway", rctx);

    RoundTripFixture.assertNoEngineErrorOrSkip(re, "allowSamewayback routing");
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull("allowSamewayback should produce a track", track);
    Assert.assertTrue("track should have significant length", track.distance > 200);
  }

  // No-beeline invariant: successful round-trip routes from any algorithm
  // (waypoint, isochrone, samewayback) must not contain generated DIRECT
  // waypoints or direct_segment messages, even with add_beeline enabled.
  @Test
  public void roundTripWaypointNoBeelineWithAddBeeline() {
    assertRoundTripHasNoBeeline("rtWpNoBl", false, false);
  }

  @Test
  public void roundTripIsochroneNoBeelineWithAddBeeline() {
    assertRoundTripHasNoBeeline("rtIsoNoBl", true, false);
  }

  @Test
  public void roundTripSamewaybackNoBeelineWithAddBeeline() {
    assertRoundTripHasNoBeeline("rtSwbNoBl", false, true);
  }

  private void assertRoundTripHasNoBeeline(String trackname, boolean isochrone, boolean samewayback) {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 1000;
    // Enable dynamic beeline insertion. With add_beeline=true the routing
    // engine would normally splice WAYPOINT_TYPE_DIRECT segments when a
    // waypoint can't be matched within the catching range — round-trip code
    // must defeat this by snapping points to the road graph beforehand.
    rctx.buildBeelineOnRange = true;
    rctx.roundTripIsochrone = isochrone;
    rctx.allowSamewayback = samewayback;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, trackname, rctx);

    RoundTripFixture.assertNoEngineErrorOrSkip(re, trackname);
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull(trackname + " should produce a track", track);

    if (track.matchedWaypoints != null) {
      for (MatchedWaypoint mwp : track.matchedWaypoints) {
        Assert.assertNotEquals(trackname + ": no DIRECT waypoint allowed (" + mwp.name + ")",
          MatchedWaypoint.WAYPOINT_TYPE_DIRECT, mwp.wpttype);
      }
    }
    if (track.messageList != null) {
      for (String msg : track.messageList) {
        Assert.assertFalse(trackname + ": message must not contain direct_segment: " + msg,
          msg != null && msg.contains("direct_segment="));
      }
    }
  }

  // mergeUserWaypointsIntoLoop inserts user waypoints and removes redundant circle points
  @Test
  public void mergeUserWaypointsIntoLoop() {
    double searchRadius = 5000;
    double startAngle = 90;
    int targetPoints = 5;

    RoutingEngine re = createDummyEngine(searchRadius);
    List<OsmNodeNamed> wps = buildStartWaypointList();
    re.buildPointsFromCircle(wps, startAngle, searchRadius, targetPoints);
    Assert.assertEquals(6, wps.size());

    List<OsmNodeNamed> userWps = new ArrayList<>();
    userWps.add(createNode("via1", START_ILON + 50000, START_ILAT));

    re.mergeUserWaypointsIntoLoop(wps, userWps, startAngle, targetPoints);

    // 1 start + targetPoints intermediates (4 circle kept + 1 user) + 1 closing
    Assert.assertEquals(targetPoints + 2, wps.size());
    Assert.assertEquals("from", wps.get(0).name);
    Assert.assertEquals("to_rt", wps.get(wps.size() - 1).name);

    boolean foundUser = false;
    for (OsmNodeNamed wp : wps) {
      if ("via1".equals(wp.name)) foundUser = true;
    }
    Assert.assertTrue("user waypoint must be in the loop", foundUser);
  }

  @Test
  public void mergeUserWaypointsEmptyList() {
    double searchRadius = 5000;
    RoutingEngine re = createDummyEngine(searchRadius);
    List<OsmNodeNamed> wps = buildStartWaypointList();
    re.buildPointsFromCircle(wps, 90, searchRadius, 5);
    int sizeBefore = wps.size();

    re.mergeUserWaypointsIntoLoop(wps, new ArrayList<>(), 90, 5);

    Assert.assertEquals("no change when no user waypoints", sizeBefore, wps.size());
  }

  @Test
  public void mergeUserWaypointsSortedByBearing() {
    double searchRadius = 5000;
    int targetPoints = 8;

    RoutingEngine re = createDummyEngine(searchRadius);
    List<OsmNodeNamed> wps = buildStartWaypointList();

    OsmNodeNamed closing = new OsmNodeNamed(wps.get(0));
    closing.name = "to_rt";
    wps.add(closing);

    List<OsmNodeNamed> userWps = new ArrayList<>();
    userWps.add(createNode("viaEast", START_ILON + 50000, START_ILAT + 50000));
    userWps.add(createNode("viaWest", START_ILON - 50000, START_ILAT + 50000));

    re.mergeUserWaypointsIntoLoop(wps, userWps, 0, targetPoints);

    // startAngle=0 (north): NW has negative relative bearing, NE has positive
    Assert.assertEquals("from", wps.get(0).name);
    Assert.assertEquals("viaWest", wps.get(1).name);
    Assert.assertEquals("viaEast", wps.get(2).name);
    Assert.assertEquals("to_rt", wps.get(3).name);
  }

  // User waypoints must never be silently dropped when targetPoints is small.
  @Test
  public void mergeUserWaypointsPreservesAllWhenOverTarget() {
    double searchRadius = 5000;
    int targetPoints = 3; // smaller than user waypoint count (5)

    RoutingEngine re = createDummyEngine(searchRadius);
    List<OsmNodeNamed> wps = buildStartWaypointList();
    re.buildPointsFromCircle(wps, 0, searchRadius, targetPoints);

    List<OsmNodeNamed> userWps = new ArrayList<>();
    userWps.add(createNode("via1", START_ILON + 30000, START_ILAT + 30000));
    userWps.add(createNode("via2", START_ILON + 30000, START_ILAT - 30000));
    userWps.add(createNode("via3", START_ILON - 30000, START_ILAT - 30000));
    userWps.add(createNode("via4", START_ILON - 30000, START_ILAT + 30000));
    userWps.add(createNode("via5", START_ILON, START_ILAT + 30000));

    re.mergeUserWaypointsIntoLoop(wps, userWps, 0, targetPoints);

    int userPresent = 0;
    for (OsmNodeNamed wp : wps) {
      if (wp.name != null && wp.name.startsWith("via")) userPresent++;
    }
    Assert.assertEquals("all 5 user vias must survive merge", 5, userPresent);
  }

  // Bearing-order is deterministic regardless of input order.
  @Test
  public void mergeUserWaypointsBearingOrderIndependentOfInputOrder() {
    double searchRadius = 5000;

    RoutingEngine re = createDummyEngine(searchRadius);
    List<OsmNodeNamed> wpsA = buildStartWaypointList();
    OsmNodeNamed closingA = new OsmNodeNamed(wpsA.get(0));
    closingA.name = "to_rt";
    wpsA.add(closingA);

    List<OsmNodeNamed> wpsB = buildStartWaypointList();
    OsmNodeNamed closingB = new OsmNodeNamed(wpsB.get(0));
    closingB.name = "to_rt";
    wpsB.add(closingB);

    OsmNodeNamed v1 = createNode("v1", START_ILON + 50000, START_ILAT + 50000);
    OsmNodeNamed v2 = createNode("v2", START_ILON - 50000, START_ILAT + 50000);
    OsmNodeNamed v3 = createNode("v3", START_ILON - 50000, START_ILAT - 50000);

    List<OsmNodeNamed> orderedA = new ArrayList<>();
    orderedA.add(v1); orderedA.add(v2); orderedA.add(v3);
    List<OsmNodeNamed> orderedB = new ArrayList<>();
    orderedB.add(v3); orderedB.add(v1); orderedB.add(v2);

    re.mergeUserWaypointsIntoLoop(wpsA, orderedA, 0, 5);
    re.mergeUserWaypointsIntoLoop(wpsB, orderedB, 0, 5);

    Assert.assertEquals("loop sizes differ", wpsA.size(), wpsB.size());
    for (int i = 0; i < wpsA.size(); i++) {
      Assert.assertEquals("position " + i, wpsA.get(i).name, wpsB.get(i).name);
    }
  }

  // Unsnappable user via must surface as a clear error, not be silently dropped.
  @Test
  public void unsnappableUserViaFailsClearly() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 1000;

    RoutingEngine re = calcRoundTripWithVias(8.720, 50.000, "rtUnsnap", rctx,
      new double[][]{{9.5, 50.0}}); // far outside test data

    Assert.assertNotNull("expected an error for unsnappable user via", re.getErrorMessage());
    Assert.assertTrue("error must mention the user waypoint: " + re.getErrorMessage(),
      re.getErrorMessage().contains("user waypoint"));
  }

  // GREEDY + user vias falls back to WAYPOINT and preserves the user vias.
  @Test
  public void greedyWithUserViaFallsBackAndPreservesVia() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 1000;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;

    RoutingEngine re = calcRoundTripWithVias(8.720, 50.000, "rtGreedyVia", rctx,
      new double[][]{{8.722, 50.001}});

    RoundTripFixture.assertNoEngineErrorOrSkip(re, "greedy+via fallback");
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull("greedy+via should produce a track", track);
    boolean foundVia = false;
    if (track.matchedWaypoints != null) {
      for (MatchedWaypoint mwp : track.matchedWaypoints) {
        if ("via1".equals(mwp.name)) foundVia = true;
        Assert.assertNotEquals("greedy+via fallback must not produce DIRECT (" + mwp.name + ")",
          MatchedWaypoint.WAYPOINT_TYPE_DIRECT, mwp.wpttype);
      }
    }
    Assert.assertTrue("user via1 must be present in matched waypoints", foundVia);
  }

  private RoutingEngine calcRoundTripWithVias(double lon, double lat, String trackname,
                                              RoutingContext rctx, double[][] vias) {
    String out = new File(outputDir.getRoot(), trackname).getAbsolutePath();
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    start.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    wplist.add(start);
    for (int i = 0; i < vias.length; i++) {
      OsmNodeNamed via = new OsmNodeNamed();
      via.name = "via" + (i + 1);
      via.ilon = 180000000 + (int) (vias[i][0] * 1000000 + 0.5);
      via.ilat = 90000000 + (int) (vias[i][1] * 1000000 + 0.5);
      wplist.add(via);
    }
    rctx.localFunction = profileFile().getAbsolutePath();
    RoutingEngine re = new RoutingEngine(out, out, segmentDir(), wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    return re;
  }

  @Test
  public void buildPointsFromCircleGeometry() {
    double searchRadius = 5000;
    double startAngle = 90;
    int points = 5;

    List<OsmNodeNamed> wps = buildStartWaypointList();
    OsmNodeNamed start = wps.get(0);

    RoutingEngine re = createDummyEngine(searchRadius);
    re.buildPointsFromCircle(wps, startAngle, searchRadius, points);

    // should add (points-1) intermediate + 1 return = points total added
    Assert.assertEquals("should have 1 start + 4 intermediate + 1 return", 1 + points, wps.size());

    // last waypoint should be at same position as start (return to origin)
    OsmNodeNamed last = wps.get(wps.size() - 1);
    Assert.assertEquals("return waypoint lon should match start", start.ilon, last.ilon);
    Assert.assertEquals("return waypoint lat should match start", start.ilat, last.ilat);
    Assert.assertEquals("to_rt", last.name);

    // intermediate waypoints should be approximately searchRadius from start
    for (int i = 1; i < wps.size() - 1; i++) {
      double dist = CheapRuler.distance(start.ilon, start.ilat, wps.get(i).ilon, wps.get(i).ilat);
      Assert.assertTrue("waypoint " + i + " distance " + (int) dist + "m should be near searchRadius",
        Math.abs(dist - searchRadius) < searchRadius * 0.1);
      Assert.assertTrue(wps.get(i).name.startsWith("rt"));
    }
  }

  // removeBackAndForthSegments removes overlapping nodes around a waypoint
  @Test
  public void removeBackAndForthAtWaypoint() {
    double searchRadius = 5000;
    RoutingEngine re = createDummyEngine(searchRadius);

    // Build a track: A, B, C, W, C, B, D, E
    // where W is a waypoint and C, B appear on both sides (back-and-forth)
    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(1000, 1000, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(2000, 2000, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(3000, 3000, (short) 0, null);
    OsmPathElement nodeW = OsmPathElement.create(4000, 4000, (short) 0, null);
    OsmPathElement nodeC2 = OsmPathElement.create(3000, 3000, (short) 0, null); // same pos as C
    OsmPathElement nodeB2 = OsmPathElement.create(2000, 2000, (short) 0, null); // same pos as B
    OsmPathElement nodeD = OsmPathElement.create(5000, 5000, (short) 0, null);
    OsmPathElement nodeE = OsmPathElement.create(6000, 6000, (short) 0, null);

    track.nodes.add(nodeA);   // 0
    track.nodes.add(nodeB);   // 1
    track.nodes.add(nodeC);   // 2
    track.nodes.add(nodeW);   // 3 - waypoint
    track.nodes.add(nodeC2);  // 4 - duplicate of C
    track.nodes.add(nodeB2);  // 5 - duplicate of B
    track.nodes.add(nodeD);   // 6
    track.nodes.add(nodeE);   // 7

    // Set up waypoints: start at index 0, intermediate waypoint W at index 3, end at index 7
    List<MatchedWaypoint> wpts = new ArrayList<>();
    MatchedWaypoint startWp = createMatchedWaypoint("from", 1000, 1000, 1000, 1000);
    startWp.indexInTrack = 0;
    wpts.add(startWp);

    MatchedWaypoint midWp = createMatchedWaypoint("rt1", 4000, 4000, 4000, 4000);
    midWp.indexInTrack = 3;
    wpts.add(midWp);

    MatchedWaypoint endWp = createMatchedWaypoint("to_rt", 6000, 6000, 6000, 6000);
    endWp.indexInTrack = 7;
    wpts.add(endWp);

    re.removeBackAndForthSegments(track, wpts);

    // After removal: A, B, D, E. The whole B-C-W-C-B spur is removed so the
    // cleanup does not create a synthetic W-D shortcut.
    Assert.assertEquals("should have 4 nodes after removal", 4, track.nodes.size());
    Assert.assertSame("node 0 should be A", nodeA, track.nodes.get(0));
    Assert.assertSame("node 1 should be B", nodeB, track.nodes.get(1));
    Assert.assertSame("node 2 should be D", nodeD, track.nodes.get(2));
    Assert.assertSame("node 3 should be E", nodeE, track.nodes.get(3));

    // Waypoint indices should be updated to the surviving branch/end.
    Assert.assertEquals("mid waypoint index should move to branch", 1, midWp.indexInTrack);
    Assert.assertEquals("end waypoint index should be adjusted", 3, endWp.indexInTrack);
  }

  // removeBackAndForthSegments does nothing when there's no overlap
  @Test
  public void removeBackAndForthNoOverlap() {
    double searchRadius = 5000;
    RoutingEngine re = createDummyEngine(searchRadius);

    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(1000, 1000, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(2000, 2000, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(3000, 3000, (short) 0, null);
    OsmPathElement nodeD = OsmPathElement.create(4000, 4000, (short) 0, null);
    OsmPathElement nodeE = OsmPathElement.create(5000, 5000, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeD);
    track.nodes.add(nodeE);

    List<MatchedWaypoint> wpts = new ArrayList<>();
    MatchedWaypoint startWp = createMatchedWaypoint("from", 1000, 1000, 1000, 1000);
    startWp.indexInTrack = 0;
    wpts.add(startWp);

    MatchedWaypoint midWp = createMatchedWaypoint("rt1", 3000, 3000, 3000, 3000);
    midWp.indexInTrack = 2;
    wpts.add(midWp);

    MatchedWaypoint endWp = createMatchedWaypoint("to_rt", 5000, 5000, 5000, 5000);
    endWp.indexInTrack = 4;
    wpts.add(endWp);

    re.removeBackAndForthSegments(track, wpts);

    Assert.assertEquals("should have 5 nodes (unchanged)", 5, track.nodes.size());
  }

  // removeMicroDetours removes small loops where the route visits the same node twice
  @Test
  public void removeMicroDetoursSimpleLoop() {
    RoutingEngine re = createDummyEngine(5000);

    // At ~50N: 1 ilon unit ≈ 0.072m, 1 ilat unit ≈ 0.111m
    // A is 215m from B (well beyond 50m proximity), loop B→C→D→B2 ≈ 126m
    int baseLon = START_ILON;
    int baseLat = START_ILAT;
    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(baseLon, baseLat, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(baseLon + 3000, baseLat, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(baseLon + 3000, baseLat + 400, (short) 0, null);
    OsmPathElement nodeD = OsmPathElement.create(baseLon + 3400, baseLat + 400, (short) 0, null);
    OsmPathElement nodeB2 = OsmPathElement.create(baseLon + 3000, baseLat, (short) 0, null); // same pos as B
    OsmPathElement nodeE = OsmPathElement.create(baseLon + 6000, baseLat, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeD);
    track.nodes.add(nodeB2);
    track.nodes.add(nodeE);

    re.removeMicroDetours(track, 150, new ArrayList<>());

    // After removal: A, B, E (loop C→D→B2 removed)
    Assert.assertEquals("should have 3 nodes after micro-detour removal", 3, track.nodes.size());
    Assert.assertSame("node 0 should be A", nodeA, track.nodes.get(0));
    Assert.assertSame("node 1 should be B", nodeB, track.nodes.get(1));
    Assert.assertSame("node 2 should be E", nodeE, track.nodes.get(2));
  }

  // removeMicroDetours catches loops returning to a nearby (but not identical) node
  @Test
  public void removeMicroDetoursProximityMatch() {
    RoutingEngine re = createDummyEngine(5000);

    // At ~50N: 1 ilon unit ≈ 0.072m, 1 ilat unit ≈ 0.111m
    // B and B2 are ~7m apart (within 50m proximity threshold).
    // A is ~72m from B2 (beyond proximity threshold).
    int baseLon = START_ILON;
    int baseLat = START_ILAT;
    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(baseLon, baseLat, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(baseLon + 1000, baseLat, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(baseLon + 1000, baseLat + 200, (short) 0, null);
    OsmPathElement nodeD = OsmPathElement.create(baseLon + 1200, baseLat + 200, (short) 0, null);
    OsmPathElement nodeB2 = OsmPathElement.create(baseLon + 1050, baseLat + 50, (short) 0, null); // ~7m from B
    OsmPathElement nodeE = OsmPathElement.create(baseLon + 2000, baseLat, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeD);
    track.nodes.add(nodeB2);
    track.nodes.add(nodeE);

    re.removeMicroDetours(track, 350, new ArrayList<>());

    // Loop B→C→D→B2 (~56m total) should be removed via proximity match
    Assert.assertEquals("should have 3 nodes after proximity detour removal", 3, track.nodes.size());
    Assert.assertSame(nodeA, track.nodes.get(0));
    Assert.assertSame(nodeB, track.nodes.get(1));
    Assert.assertSame(nodeE, track.nodes.get(2));
  }

  // removeMicroDetours does NOT remove loops that are too large
  @Test
  public void removeMicroDetoursKeepsLargeLoop() {
    RoutingEngine re = createDummyEngine(5000);

    // Build a track with a large loop using realistic coordinates (~50N, ~8E)
    // Each 1000 units ≈ 0.001 degrees ≈ 70-110m, so 5000 units apart ≈ 350-550m
    int baseLon = START_ILON;
    int baseLat = START_ILAT;
    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(baseLon, baseLat, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(baseLon + 5000, baseLat, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(baseLon + 10000, baseLat, (short) 0, null);
    OsmPathElement nodeB2 = OsmPathElement.create(baseLon + 5000, baseLat, (short) 0, null); // same as B
    OsmPathElement nodeD = OsmPathElement.create(baseLon + 15000, baseLat, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeB2);
    track.nodes.add(nodeD);

    re.removeMicroDetours(track, 150, new ArrayList<>());

    Assert.assertEquals("should have 5 nodes (loop too large to remove)", 5, track.nodes.size());
  }

  @Test
  public void removeMicroDetoursNoLoop() {
    RoutingEngine re = createDummyEngine(5000);

    // Nodes ~132m apart at 50N — well beyond 50m proximity threshold
    int baseLon = START_ILON;
    int baseLat = START_ILAT;
    OsmTrack track = new OsmTrack();
    track.nodes.add(OsmPathElement.create(baseLon, baseLat, (short) 0, null));
    track.nodes.add(OsmPathElement.create(baseLon + 1000, baseLat + 1000, (short) 0, null));
    track.nodes.add(OsmPathElement.create(baseLon + 2000, baseLat + 2000, (short) 0, null));

    re.removeMicroDetours(track, 150, new ArrayList<>());

    Assert.assertEquals("should have 3 nodes (unchanged)", 3, track.nodes.size());
  }

  @Test
  public void removeMicroDetoursMultipleLoops() {
    RoutingEngine re = createDummyEngine(5000);

    // Track: A → B → C → B → D → E → F → E → G
    // Two micro-detours: B→C→B and E→F→E
    // Groups are 3000 units (215m) apart — well beyond proximity threshold
    int baseLon = START_ILON;
    int baseLat = START_ILAT;
    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(baseLon, baseLat, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(baseLon + 3000, baseLat, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(baseLon + 3000, baseLat + 400, (short) 0, null);
    OsmPathElement nodeB2 = OsmPathElement.create(baseLon + 3000, baseLat, (short) 0, null);
    OsmPathElement nodeD = OsmPathElement.create(baseLon + 6000, baseLat, (short) 0, null);
    OsmPathElement nodeE = OsmPathElement.create(baseLon + 9000, baseLat, (short) 0, null);
    OsmPathElement nodeF = OsmPathElement.create(baseLon + 9000, baseLat + 400, (short) 0, null);
    OsmPathElement nodeE2 = OsmPathElement.create(baseLon + 9000, baseLat, (short) 0, null);
    OsmPathElement nodeG = OsmPathElement.create(baseLon + 12000, baseLat, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeB2);
    track.nodes.add(nodeD);
    track.nodes.add(nodeE);
    track.nodes.add(nodeF);
    track.nodes.add(nodeE2);
    track.nodes.add(nodeG);

    re.removeMicroDetours(track, 150, new ArrayList<>());

    // After removal: A, B, D, E, G
    Assert.assertEquals("should have 5 nodes after removing two detours", 5, track.nodes.size());
    Assert.assertSame(nodeA, track.nodes.get(0));
    Assert.assertSame(nodeB, track.nodes.get(1));
    Assert.assertSame(nodeD, track.nodes.get(2));
    Assert.assertSame(nodeE, track.nodes.get(3));
    Assert.assertSame(nodeG, track.nodes.get(4));
  }

  // --- Isochrone + combined strategy tests ---

  // Integration: isochrone=true produces a valid closed loop
  @Test
  public void roundTripIsochroneBasicLoop() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 1000;
    rctx.roundTripIsochrone = true;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtIsochrone", rctx);

    RoundTripFixture.assertNoEngineErrorOrSkip(re, "isochrone round-trip");
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull("isochrone should produce a track", track);
    Assert.assertTrue("track should have nodes", track.nodes.size() > 2);

    OsmPathElement first = track.nodes.get(0);
    OsmPathElement last = track.nodes.get(track.nodes.size() - 1);
    int closingDistance = first.calcDistance(last);
    Assert.assertTrue("loop should close near origin, gap=" + closingDistance + "m",
      closingDistance < 500);
  }

  // Integration: isochrone produces valid loops in all 4 directions
  @Test
  public void roundTripIsochroneAllDirections() {
    for (int dir : new int[]{0, 90, 180, 270}) {
      RoutingContext rctx = new RoutingContext();
      rctx.startDirection = dir;
      rctx.roundTripDistance = 3000; // need realistic radius for indirectness-based placement
      rctx.roundTripIsochrone = true;

      RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtIsoDir" + dir, rctx);

      RoundTripFixture.assertNoEngineErrorOrSkip(re, "isochrone dir=" + dir);
      OsmTrack track = re.getFoundTrack();
      Assert.assertNotNull("isochrone dir=" + dir + " should produce track", track);
      Assert.assertTrue("isochrone dir=" + dir + " should have >2 nodes", track.nodes.size() > 2);
      Assert.assertTrue("isochrone dir=" + dir + " should have positive distance", track.distance > 500);
    }
  }

  // Integration: isochrone and probe both produce valid routes for the same input
  @Test
  public void roundTripIsochroneAndProbeBothSucceed() {
    RoutingContext probeCtx = new RoutingContext();
    probeCtx.startDirection = 0;
    probeCtx.roundTripDistance = 1000;
    RoutingEngine probeRe = calcRoundTrip(8.720, 50.000, "rtBothProbe", probeCtx);

    RoutingContext isoCtx = new RoutingContext();
    isoCtx.startDirection = 0;
    isoCtx.roundTripDistance = 1000;
    isoCtx.roundTripIsochrone = true;
    RoutingEngine isoRe = calcRoundTrip(8.720, 50.000, "rtBothIso", isoCtx);

    Assert.assertNull("probe failed: " + probeRe.getErrorMessage(), probeRe.getErrorMessage());
    Assert.assertNull("isochrone failed: " + isoRe.getErrorMessage(), isoRe.getErrorMessage());

    OsmTrack probeTrack = probeRe.getFoundTrack();
    OsmTrack isoTrack = isoRe.getFoundTrack();
    Assert.assertNotNull(probeTrack);
    Assert.assertNotNull(isoTrack);
    Assert.assertTrue("probe should produce positive distance", probeTrack.distance > 100);
    Assert.assertTrue("isochrone should produce positive distance", isoTrack.distance > 100);
  }

  // Integration: isochrone distance accuracy is reasonable
  @Test
  public void roundTripIsochroneDistanceAccuracy() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 90;
    rctx.roundTripDistance = 3000;
    rctx.roundTripIsochrone = true;

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtIsoAccuracy", rctx);

    RoundTripFixture.assertNoEngineErrorOrSkip(re, "isochrone routing");
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull(track);

    double ratio = (double) track.distance / 3000;
    Assert.assertTrue("distance ratio " + String.format("%.2f", ratio) + " should be < 5.0", ratio < 5.0);
    Assert.assertTrue("distance ratio " + String.format("%.2f", ratio) + " should be > 0.2", ratio > 0.2);
  }

  // Integration: default (no flag) uses probe, not isochrone
  @Test
  public void roundTripDefaultUsesProbe() {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 1000;
    // roundTripIsochrone NOT set — should default to false

    Assert.assertFalse("roundTripIsochrone should default to false", rctx.roundTripIsochrone);

    RoutingEngine re = calcRoundTrip(8.720, 50.000, "rtDefaultProbe", rctx);
    Assert.assertNull(re.getErrorMessage());
    Assert.assertNotNull(re.getFoundTrack());
  }

  // --- mergeIsochroneWithProbe unit tests ---

  @Test
  public void mergeIsochroneWithProbeNoOverlap() {
    // Isochrone has N and S, probe adds E and W
    double[][] frontier = {{0, 500}, {180, 600}};
    double[] probe = {90, 270};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, probe, 1000);

    Assert.assertEquals("should have 4 entries", 4, merged.length);
    // Verify isochrone entries keep their distances
    boolean foundNorth = false, foundEast = false;
    for (double[] entry : merged) {
      if (Math.abs(entry[0] - 0) < 1) { foundNorth = true; Assert.assertEquals(500, entry[1], 0.1); }
      if (Math.abs(entry[0] - 90) < 1) { foundEast = true; Assert.assertEquals(1000, entry[1], 0.1); } // probe uses searchRadius
    }
    Assert.assertTrue("should contain north (isochrone)", foundNorth);
    Assert.assertTrue("should contain east (probe fill)", foundEast);
  }

  @Test
  public void mergeIsochroneWithProbeOverlap() {
    // Isochrone and probe both have direction 90 — isochrone distance should win
    double[][] frontier = {{90, 750}};
    double[] probe = {90, 180};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, probe, 1000);

    Assert.assertEquals("should have 2 entries", 2, merged.length);
    // Direction 90 should keep isochrone distance (750), not probe (1000)
    for (double[] entry : merged) {
      if (Math.abs(entry[0] - 90) < 1) {
        Assert.assertEquals("overlapping direction should keep isochrone distance", 750, entry[1], 0.1);
      }
    }
  }

  @Test
  public void mergeIsochroneWithProbeNullIsochrone() {
    // Isochrone failed — merge should use probe directions at searchRadius
    double[] probe = {0, 90, 180, 270};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(null, probe, 2000);

    Assert.assertEquals("should have 4 entries from probe", 4, merged.length);
    for (double[] entry : merged) {
      Assert.assertEquals("probe-only entries should use searchRadius", 2000, entry[1], 0.1);
    }
  }

  @Test
  public void mergeIsochroneWithProbeNullProbe() {
    // Probe failed — merge should return isochrone data unchanged
    double[][] frontier = {{45, 800}, {135, 600}, {225, 900}};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, null, 1000);

    Assert.assertEquals("should have 3 entries from isochrone", 3, merged.length);
    Assert.assertEquals(800, merged[0][1], 0.1);
  }

  @Test
  public void mergeIsochroneWithProbeBothNull() {
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(null, null, 1000);
    Assert.assertNull("both null should return null", merged);
  }

  @Test
  public void mergeIsochroneWithProbeCloseDirections() {
    // Isochrone has 90°, probe has 93° — should NOT add probe (within 5° threshold)
    double[][] frontier = {{90, 750}};
    double[] probe = {93};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, probe, 1000);

    Assert.assertEquals("close direction should not be added", 1, merged.length);
    Assert.assertEquals(750, merged[0][1], 0.1);
  }

  @Test
  public void mergeIsochroneWithProbeSorted() {
    // Result should be sorted by direction
    double[][] frontier = {{270, 500}, {90, 600}};
    double[] probe = {0, 180};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, probe, 1000);

    for (int i = 1; i < merged.length; i++) {
      Assert.assertTrue("result should be sorted by direction",
        merged[i][0] >= merged[i - 1][0]);
    }
  }

  @Test
  public void mergeIsochroneWithProbePreservesSixElementEntries() {
    // Isochrone-sourced entries are 6-element [dir, dist, cost, hits, ilon, ilat]
    // — the merge must pass these through unchanged so direct-ISOCHRONE
    // placement can find the road-native coord downstream.
    double[][] frontier = {{0, 2000, 2600, 5, 188_720_111, 140_000_222}};
    double[] probe = {180};
    double[][] merged = RoutingEngine.mergeIsochroneWithProbe(frontier, probe, 1000);

    Assert.assertEquals(2, merged.length);
    for (double[] entry : merged) {
      if (Math.abs(entry[0]) < 1) {
        Assert.assertEquals("iso entry preserves 6 elements", 6, entry.length);
        Assert.assertEquals(188_720_111, (int) entry[4]);
        Assert.assertEquals(140_000_222, (int) entry[5]);
      } else {
        // Probe-only injection — 4 elements, no road-native data.
        Assert.assertEquals("probe-only entry stays 4 elements", 4, entry.length);
      }
    }
  }

  // --- direct ISOCHRONE road-native waypoint placement ---

  /**
   * Fallback path: when no candidate pool is supplied, placement uses the
   * frontier entry's road-native coord (entry[4]/entry[5]) rather than
   * synthesizing a position. Production passes the full candidate pool to get
   * airDist-aware selection — see
   * {@link #placeWaypointsFromIsochronePicksCandidateMatchingPlacementRadius}.
   */
  @Test
  public void placeWaypointsFromIsochroneUsesRoadNativeCoordsWhenAvailable() {
    RoutingEngine re = createDummyEngine(0);

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = START_ILON;
    start.ilat = START_ILAT;
    wps.add(start);

    // Frontier with 6-element entries, each at a distinct road-native coord
    // far enough from start (airDist > 0.4 * searchRadius) and with hits >= 3
    // so they pass the in-method usability filter.
    double searchRadius = 2000;
    int[][] expectedPositions = {
      {START_ILON + 30_000, START_ILAT},
      {START_ILON, START_ILAT + 25_000},
      {START_ILON - 28_000, START_ILAT + 2_000},
      {START_ILON - 5_000, START_ILAT - 27_000},
    };
    double[][] frontier = {
      {  0, 1500, 1950, 5, expectedPositions[0][0], expectedPositions[0][1]},
      { 90, 1600, 2080, 5, expectedPositions[1][0], expectedPositions[1][1]},
      {180, 1550, 2015, 5, expectedPositions[2][0], expectedPositions[2][1]},
      {270, 1500, 1950, 5, expectedPositions[3][0], expectedPositions[3][1]},
    };

    re.placeWaypointsFromIsochrone(wps, frontier, null, searchRadius, 0, 5);

    // 1 start + 4 intermediates + 1 closing == 6
    Assert.assertEquals("expected start + 4 rt + closing", 6, wps.size());
    Assert.assertEquals(START_ILON, wps.get(0).ilon);
    Assert.assertEquals(START_ILAT, wps.get(0).ilat);
    Assert.assertEquals("closing waypoint must be a copy of start",
      START_ILON, wps.get(wps.size() - 1).ilon);
    Assert.assertEquals(START_ILAT, wps.get(wps.size() - 1).ilat);

    // Each intermediate waypoint must land on one of the road-native coords —
    // not at a CheapRuler.destination-synthesized position.
    java.util.Set<Long> expectedKeys = new java.util.HashSet<>();
    for (int[] pos : expectedPositions) {
      expectedKeys.add(new OsmNode(pos[0], pos[1]).getIdFromPos());
    }
    for (int i = 1; i < wps.size() - 1; i++) {
      OsmNodeNamed wp = wps.get(i);
      Assert.assertTrue("waypoint " + wp.name + " (" + wp.ilon + "," + wp.ilat
        + ") should match a road-native frontier coord",
        expectedKeys.contains(wp.getIdFromPos()));
    }
  }

  /**
   * Mixed frontier: 6-element entries reuse their road-native coord while
   * probe-only 4-element entries fall back to {@code CheapRuler.destination}
   * at the indirectness-compensated air-distance. Note that probe-only
   * (hits=0) entries enter the placement only via the relaxed-fallback branch
   * in placeWaypointsFromIsochrone (usable.size() < 4).
   */
  @Test
  public void placeWaypointsFromIsochroneFallsBackToSyntheticForProbeOnly() {
    RoutingEngine re = createDummyEngine(0);

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = START_ILON;
    start.ilat = START_ILAT;
    wps.add(start);

    double searchRadius = 2000;
    // Two iso entries (6-element) at 0° and 180°, two probe-only at 90° and 270°.
    int[] isoNorth = {START_ILON, START_ILAT + 24_000};
    int[] isoSouth = {START_ILON, START_ILAT - 24_000};
    double[][] frontier = {
      {  0, 1500, 1950, 5, isoNorth[0], isoNorth[1]},
      { 90, 2000, 2600, 0},  // probe-only, no coord
      {180, 1500, 1950, 5, isoSouth[0], isoSouth[1]},
      {270, 2000, 2600, 0},  // probe-only, no coord
    };

    re.placeWaypointsFromIsochrone(wps, frontier, null, searchRadius, 0, 5);

    Assert.assertEquals(6, wps.size());

    long northKey = new OsmNode(isoNorth[0], isoNorth[1]).getIdFromPos();
    long southKey = new OsmNode(isoSouth[0], isoSouth[1]).getIdFromPos();
    long startKey = new OsmNode(START_ILON, START_ILAT).getIdFromPos();
    boolean sawNorth = false, sawSouth = false;
    int syntheticCount = 0;
    for (int i = 1; i < wps.size() - 1; i++) {
      OsmNodeNamed wp = wps.get(i);
      long key = wp.getIdFromPos();
      if (key == northKey) sawNorth = true;
      else if (key == southKey) sawSouth = true;
      else {
        Assert.assertNotEquals("synthetic waypoint must not coincide with start",
          startKey, key);
        syntheticCount++;
      }
    }
    Assert.assertTrue("road-native north entry should appear", sawNorth);
    Assert.assertTrue("road-native south entry should appear", sawSouth);
    Assert.assertEquals("two probe-only directions placed synthetically", 2, syntheticCount);
  }

  /**
   * When a candidate pool is passed, placement must pick the candidate whose
   * air-distance best matches the indirectness-compensated target — not the
   * frontier-max coord (which sits at the cost-budget envelope and would
   * overshoot the requested loop size for small radii).
   */
  @Test
  public void placeWaypointsFromIsochronePicksCandidateMatchingPlacementRadius() {
    RoutingEngine re = createDummyEngine(0);

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = START_ILON;
    start.ilat = START_ILAT;
    wps.add(start);

    // 4 buckets at 0/90/180/270. For each bucket, give the candidate pool a
    // frontier-max far out (at 3000m) plus a 25%-contour candidate at ~1000m.
    // With searchRadius=2000 the placement target is roughly searchRadius, so
    // the 25%-contour candidate (near 1000m) should be picked over the
    // frontier-max (at 3000m).
    double searchRadius = 2000;
    int[][] frontierCoords = {
      {START_ILON + 60_000, START_ILAT},
      {START_ILON, START_ILAT + 50_000},
      {START_ILON - 60_000, START_ILAT},
      {START_ILON, START_ILAT - 50_000},
    };
    int[][] contourCoords = {
      {START_ILON + 20_000, START_ILAT},
      {START_ILON, START_ILAT + 16_000},
      {START_ILON - 20_000, START_ILAT},
      {START_ILON, START_ILAT - 16_000},
    };
    double[][] frontier = new double[4][];
    List<IsoCandidate> candidates = new ArrayList<>();
    int[] buckets = {0, 9, 18, 27};
    double[] bearings = {0, 90, 180, 270};
    for (int i = 0; i < 4; i++) {
      frontier[i] = new double[]{bearings[i], 3000, 3900, 5, frontierCoords[i][0], frontierCoords[i][1]};
      candidates.add(new IsoCandidate(frontierCoords[i][0], frontierCoords[i][1],
        bearings[i], 3000, 3900, buckets[i], 5, 100));
      candidates.add(new IsoCandidate(contourCoords[i][0], contourCoords[i][1],
        bearings[i], 1000, 1300, buckets[i], 5, 25));
    }

    re.placeWaypointsFromIsochrone(wps, frontier, candidates, searchRadius, 0, 5);

    Assert.assertEquals(6, wps.size());

    // Every intermediate waypoint should land on a contour-coord, not on the
    // far-out frontier-max coord.
    java.util.Set<Long> contourKeys = new java.util.HashSet<>();
    java.util.Set<Long> frontierKeys = new java.util.HashSet<>();
    for (int i = 0; i < 4; i++) {
      contourKeys.add(new OsmNode(contourCoords[i][0], contourCoords[i][1]).getIdFromPos());
      frontierKeys.add(new OsmNode(frontierCoords[i][0], frontierCoords[i][1]).getIdFromPos());
    }
    int contourHits = 0, frontierHits = 0;
    for (int i = 1; i < wps.size() - 1; i++) {
      long key = wps.get(i).getIdFromPos();
      if (contourKeys.contains(key)) contourHits++;
      else if (frontierKeys.contains(key)) frontierHits++;
    }
    Assert.assertEquals("airDist-aware selection should prefer the 1000m contour over the 3000m frontier",
      4, contourHits);
    Assert.assertEquals(0, frontierHits);
  }

  // --- Reachability-aware waypoint placement tests ---

  @Test
  public void angleDiffBasic() {
    Assert.assertEquals(0, RoutingEngine.angleDiff(0, 0), 0.01);
    Assert.assertEquals(90, RoutingEngine.angleDiff(0, 90), 0.01);
    Assert.assertEquals(180, RoutingEngine.angleDiff(0, 180), 0.01);
    Assert.assertEquals(90, RoutingEngine.angleDiff(0, 270), 0.01);
    Assert.assertEquals(10, RoutingEngine.angleDiff(355, 5), 0.01);
  }

  @Test
  public void selectSpreadDirectionsFullCircle() {
    // All 24 directions viable — should pick evenly spaced subset
    double[] viable = new double[24];
    for (int i = 0; i < 24; i++) viable[i] = i * 15;

    double[] selected = RoutingEngine.selectSpreadDirections(viable, 4, 0);
    Assert.assertEquals(4, selected.length);

    // Should be roughly 90° apart
    for (int i = 0; i < selected.length; i++) {
      for (int j = i + 1; j < selected.length; j++) {
        double diff = RoutingEngine.angleDiff(selected[i], selected[j]);
        Assert.assertTrue("selected directions should be well-spread, got diff=" + diff,
          diff >= 60);
      }
    }
  }

  @Test
  public void selectSpreadDirectionsValley() {
    // Only E-W directions viable (simulating an E-W valley)
    double[] viable = {60, 75, 90, 105, 120, 240, 255, 270, 285, 300};

    double[] selected = RoutingEngine.selectSpreadDirections(viable, 5, 90);
    Assert.assertEquals(5, selected.length);

    // First selected should be close to startDirection (90)
    Assert.assertTrue("first should be near 90°, got " + selected[0],
      RoutingEngine.angleDiff(selected[0], 90) <= 15);
  }

  @Test
  public void selectSpreadDirectionsFewerThanNeeded() {
    // Callers (placeWaypointsFromEnvelope etc.) cap 'needed' to viable.length
    // before calling selectSpreadDirections. When called with count <= viable.length,
    // it correctly selects all of them.
    double[] viable = {0, 90, 180};
    double[] selected = RoutingEngine.selectSpreadDirections(viable, 3, 0);
    Assert.assertEquals(3, selected.length);
    java.util.Set<Double> selectedSet = new java.util.HashSet<>();
    for (double d : selected) selectedSet.add(d);
    Assert.assertTrue("should contain 0", selectedSet.contains(0.0));
    Assert.assertTrue("should contain 90", selectedSet.contains(90.0));
    Assert.assertTrue("should contain 180", selectedSet.contains(180.0));
  }

  @Test
  public void sortDirectionsForLoop() {
    double[] dirs = {270, 90, 180, 0};
    double[] sorted = RoutingEngine.sortDirectionsForLoop(dirs, 0);
    // Should be ordered: 0, 90, 180, 270 (clockwise from north)
    Assert.assertEquals(0, sorted[0], 0.01);
    Assert.assertEquals(90, sorted[1], 0.01);
    Assert.assertEquals(180, sorted[2], 0.01);
    Assert.assertEquals(270, sorted[3], 0.01);
  }

  @Test
  public void sortDirectionsForLoopStartingSouth() {
    double[] dirs = {270, 90, 180, 0};
    double[] sorted = RoutingEngine.sortDirectionsForLoop(dirs, 180);
    // Starting from south (180): 180, 270, 0, 90
    Assert.assertEquals(180, sorted[0], 0.01);
    Assert.assertEquals(270, sorted[1], 0.01);
    Assert.assertEquals(0, sorted[2], 0.01);
    Assert.assertEquals(90, sorted[3], 0.01);
  }

  // --- Loop perimeter scaling tests ---

  @Test
  public void computeLoopPerimeterFactorFullCircle() {
    // 4 waypoints at 0, 90, 180, 270 — a full circle
    double[] sorted = RoutingEngine.sortDirectionsForLoop(new double[]{0, 90, 180, 270}, 0);
    double factor = RoutingEngine.computeLoopPerimeterFactor(sorted);
    // 3 chords of 90° each: 3 × 2×sin(45°) = 3 × 1.414 = 4.243, plus 2 radial = 6.243
    Assert.assertEquals(6.243, factor, 0.01);
  }

  @Test
  public void computeReferencePerimeterFactorFivePoints() {
    // 5 targetPoints → 4 intermediates, arc span = 2×(90-36) = 108°, 3 gaps of 36°
    double factor = RoutingEngine.computeReferencePerimeterFactor(5);
    // 3 × 2×sin(18°) + 2 = 3 × 0.618 + 2 = 3.854
    Assert.assertEquals(3.854, factor, 0.02);
  }

  @Test
  public void computeRadiusScaleFullCircle() {
    // Wide loop should get a scale < 1.0 to match v1.7.8's narrow arc
    double[] dirs = {0, 90, 180, 270};
    double[] sorted = RoutingEngine.sortDirectionsForLoop(dirs, 0);
    double scale = RoutingEngine.computeRadiusScale(sorted, 5);
    // refPerim(5) / actualPerim(360°) = 3.854 / 6.243 ≈ 0.617
    Assert.assertTrue("scale should be < 1.0", scale < 1.0);
    Assert.assertTrue("scale should be > 0.5", scale > 0.5);
    Assert.assertEquals(0.617, scale, 0.02);
  }

  @Test
  public void computeRadiusScaleNarrowArc() {
    // Narrow arc similar to v1.7.8 should give scale ≈ 1.0
    double[] dirs = {340, 350, 10, 20};
    double[] sorted = RoutingEngine.sortDirectionsForLoop(dirs, 0);
    double scale = RoutingEngine.computeRadiusScale(sorted, 5);
    Assert.assertTrue("narrow arc should give scale close to 1.0", scale >= 0.9);
  }

  // --- Ferry segment filtering test ---

  @Test
  public void filterRoundTripWaypointsRemovesFerrySegments() throws Exception {
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile().getAbsolutePath();
    RoutingEngine re = new RoutingEngine(null, null, new File("."), new ArrayList<>(),
      rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.roundTripSearchRadius = 5000;

    List<MatchedWaypoint> waypoints = new ArrayList<>();
    waypoints.add(createMatchedWaypoint("from", START_ILON, START_ILAT, START_ILON, START_ILAT));

    // Normal road waypoint at ~700m north (well-spaced, node1-node2 ~7m)
    waypoints.add(createMatchedWaypoint("rt1", START_ILON, START_ILAT + 10000,
      START_ILON, START_ILAT + 10000));

    // Ferry-like waypoint: node1-node2 very far apart (simulating a ferry segment)
    MatchedWaypoint ferryWp = createMatchedWaypoint("rt2", START_ILON + 10000, START_ILAT,
      START_ILON + 10000, START_ILAT);
    // Override node1/node2: ~3.6km apart geographically → ferry-like
    ferryWp.node1 = new OsmNode(START_ILON, START_ILAT);
    ferryWp.node2 = new OsmNode(START_ILON + 50000, START_ILAT);
    ferryWp.radius = 100;
    waypoints.add(ferryWp);

    // Normal road waypoint at ~700m west (well-spaced from rt1)
    waypoints.add(createMatchedWaypoint("rt3", START_ILON - 10000, START_ILAT,
      START_ILON - 10000, START_ILAT));

    waypoints.add(createMatchedWaypoint("to_rt", START_ILON, START_ILAT, START_ILON, START_ILAT));

    re.filterRoundTripWaypoints(waypoints);

    // The ferry waypoint (rt2) should be removed, others kept
    boolean ferryRemoved = true;
    for (MatchedWaypoint mwp : waypoints) {
      if ("rt2".equals(mwp.name)) ferryRemoved = false;
    }
    Assert.assertTrue("ferry waypoint rt2 should have been removed", ferryRemoved);
    // rt1 and rt3 should still be present
    boolean hasRt1 = false, hasRt3 = false;
    for (MatchedWaypoint mwp : waypoints) {
      if ("rt1".equals(mwp.name)) hasRt1 = true;
      if ("rt3".equals(mwp.name)) hasRt3 = true;
    }
    Assert.assertTrue("normal waypoint rt1 should be kept", hasRt1);
    Assert.assertTrue("normal waypoint rt3 should be kept", hasRt3);
  }

  private static OsmNodeNamed createNode(String name, int ilon, int ilat) {
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = name;
    n.ilon = ilon;
    n.ilat = ilat;
    return n;
  }

  private List<OsmNodeNamed> buildStartWaypointList() {
    List<OsmNodeNamed> wps = new ArrayList<>();
    wps.add(createNode("from", START_ILON, START_ILAT));
    return wps;
  }

  private MatchedWaypoint createMatchedWaypoint(String name, int wpIlon, int wpIlat, int cpIlon, int cpIlat) {
    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.name = name;
    mwp.waypoint = new OsmNode(wpIlon, wpIlat);
    mwp.crosspoint = new OsmNode(cpIlon, cpIlat);
    mwp.node1 = new OsmNode(cpIlon, cpIlat);
    mwp.node2 = new OsmNode(cpIlon + 100, cpIlat + 100);
    mwp.radius = mwp.waypoint.calcDistance(mwp.crosspoint);
    return mwp;
  }

  private void copyResourceToDir(String resource, File dir) throws Exception {
    try (InputStream in = getClass().getResourceAsStream(resource)) {
      Assert.assertNotNull("resource not found: " + resource, in);
      Files.copy(in, new File(dir, resource.substring(resource.lastIndexOf('/') + 1)).toPath());
    }
  }

  private File profileFile() {
    return new File(projectDir, "misc/profiles2/trekking.brf");
  }

  private File segmentDir() {
    return new File(projectDir, "brouter-map-creator/build/resources/test/tmp/segments");
  }

  private RoutingEngine createDummyEngine(double searchRadius) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = "from";
    n.ilon = START_ILON;
    n.ilat = START_ILAT;
    wplist.add(n);
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile().getAbsolutePath();
    RoutingEngine re = new RoutingEngine(
      null, null,
      segmentDir(),
      wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.roundTripSearchRadius = searchRadius;
    return re;
  }

  private RoutingEngine calcRoundTrip(double lon, double lat, String trackname, RoutingContext rctx) {
    String out = new File(outputDir.getRoot(), trackname).getAbsolutePath();

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = "from";
    n.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    n.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    wplist.add(n);

    rctx.localFunction = profileFile().getAbsolutePath();

    RoutingEngine re = new RoutingEngine(
      out, out,
      segmentDir(),
      wplist,
      rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);

    re.doRun(0);

    return re;
  }

  private String calcRoute(double flon, double flat, double tlon, double tlat, File dir, String trackname, RoutingContext rctx) {
    String out = new File(dir, trackname).getAbsolutePath();

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed n;
    n = new OsmNodeNamed();
    n.name = "from";
    n.ilon = 180000000 + (int) (flon * 1000000 + 0.5);
    n.ilat = 90000000 + (int) (flat * 1000000 + 0.5);
    wplist.add(n);

    n = new OsmNodeNamed();
    n.name = "to";
    n.ilon = 180000000 + (int) (tlon * 1000000 + 0.5);
    n.ilat = 90000000 + (int) (tlat * 1000000 + 0.5);
    wplist.add(n);

    rctx.localFunction = profileFile().getAbsolutePath();

    RoutingEngine re = new RoutingEngine(
      out, out,
      segmentDir(),
      wplist,
      rctx);

    re.doRun(0);

    return re.getErrorMessage();
  }

  // --- Phase 2.0: iso-asymmetry bearing computation -----------------------

  /** Build a synthetic 36-bucket frontier table. Bucket i is at bearing
   *  i*10°. Each bucket = [direction_deg, airDist_m, cost, hits, ilon, ilat].
   *  ilon/ilat are zeroed (not used by the bias computation). */
  private static double[][] frontier36(double[] airDist, double[] cost, int[] hits) {
    double[][] f = new double[36][];
    for (int i = 0; i < 36; i++) {
      f[i] = new double[]{i * 10.0, airDist[i], cost[i], hits[i], 0, 0};
    }
    return f;
  }

  /** Uniform-reach helper: every bucket has the same airDist/cost/hits. */
  private static double[][] uniformFrontier(double airDist, double cost, int hits) {
    double[] a = new double[36];
    double[] c = new double[36];
    int[] h = new int[36];
    for (int i = 0; i < 36; i++) {
      a[i] = airDist;
      c[i] = cost;
      h[i] = hits;
    }
    return frontier36(a, c, h);
  }

  @Test
  public void isoAsymmetry_symmetricFrontier_picksLowestBucketIndex() {
    // All buckets identical → tie-break by lowest bucket index → bucket 0 = 0°.
    double[][] f = uniformFrontier(8000.0, 10000.0, 5);
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertTrue("bias should fire when all buckets pass thresholds", bias.applied);
    Assert.assertEquals("tie → bucket 0 (bearing 0°)", 0.0, bias.bearingDegrees, 0.01);
  }

  @Test
  public void isoAsymmetry_asymmetricFrontier_picksLowestIndirectness() {
    // 5 buckets reach far at low cost (best reach); 31 reach moderately.
    double[] a = new double[36];
    double[] c = new double[36];
    int[] h = new int[36];
    for (int i = 0; i < 36; i++) {
      a[i] = 7000.0;
      c[i] = 12000.0; // indirectness ≈ 1.71
      h[i] = 5;
    }
    // The east sector (buckets 8-12, bearings 80°-120°) is the "valley":
    // long reach for less cost.
    for (int i = 8; i <= 12; i++) {
      a[i] = 9500.0;
      c[i] = 11000.0; // indirectness ≈ 1.16
      h[i] = 8;
    }
    double[][] f = frontier36(a, c, h);
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertTrue("bias should fire", bias.applied);
    Assert.assertTrue("bearing should be in the east sector (80°-120°)",
      bias.bearingDegrees >= 80.0 && bias.bearingDegrees <= 120.0);
    Assert.assertEquals("hits from the winning bucket", 8, bias.hits);
  }

  @Test
  public void isoAsymmetry_sparseBuckets_noBiasApplied() {
    // All buckets reach far enough but hit count is below the minHits=3 floor.
    double[][] f = uniformFrontier(8000.0, 10000.0, 1);
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertFalse("hits < 3 disqualifies all buckets", bias.applied);
  }

  @Test
  public void isoAsymmetry_reachFloorNotMet_noBiasApplied() {
    // hits OK but airDist below 0.6 * searchRadius (= 6000m).
    double[][] f = uniformFrontier(4000.0, 10000.0, 5);
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertFalse("airDist < 0.6 * searchRadius disqualifies all buckets", bias.applied);
  }

  @Test
  public void isoAsymmetry_emptyFrontier_noBiasApplied() {
    Assert.assertFalse(RoutingEngine.computeIsoAsymmetryBearing(new double[0][], 10000.0).applied);
    Assert.assertFalse(RoutingEngine.computeIsoAsymmetryBearing(null, 10000.0).applied);
  }

  @Test
  public void isoAsymmetry_probeOnlyEntriesIgnored() {
    // Mix of 6-element road-native entries and 4-element probe-only entries
    // (from IsochroneExpansionResult docs). All should be considered uniformly
    // for the bias since indices 0-3 are populated on both forms.
    double[][] f = new double[36][];
    for (int i = 0; i < 36; i++) {
      if (i % 2 == 0) {
        f[i] = new double[]{i * 10.0, 8000.0, 10000.0, 5, 0, 0}; // 6-element
      } else {
        f[i] = new double[]{i * 10.0, 8000.0, 10000.0, 5};       // 4-element
      }
    }
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertTrue("4-element probe entries must still be considered", bias.applied);
  }

  @Test
  public void isoAsymmetry_resultCarriesAllTelemetry() {
    double[][] f = uniformFrontier(8000.0, 10000.0, 5);
    RoutingEngine.IsoAsymmetryBias bias = RoutingEngine.computeIsoAsymmetryBearing(f, 10000.0);
    Assert.assertTrue(bias.applied);
    Assert.assertEquals(0.0, bias.bearingDegrees, 0.01);
    Assert.assertEquals(10000.0 / 8000.0, bias.indirectness, 0.001);
    Assert.assertEquals(5, bias.hits);
    Assert.assertEquals(8000, bias.airDistMeters);
  }

  @Test
  public void isoAsymmetryNone_carriesSentinels() {
    RoutingEngine.IsoAsymmetryBias none = RoutingEngine.IsoAsymmetryBias.NONE;
    Assert.assertFalse(none.applied);
    Assert.assertTrue(Double.isNaN(none.bearingDegrees));
    Assert.assertTrue(Double.isNaN(none.indirectness));
    Assert.assertEquals(-1, none.hits);
    Assert.assertEquals(-1, none.airDistMeters);
  }
}
