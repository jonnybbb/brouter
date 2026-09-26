package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import org.junit.Test;

import static org.junit.Assert.*;

/** Expected slopes come from independent endpoint heights, not stored deltaH. */
public class GradientFinalizationTest {
  private RoutingEngine engine() {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = "../misc/profiles2/trekking.brf";
    return new RoutingEngine(null, null, new File("."), new ArrayList<>(), rc);
  }

  private OsmTrack track(short... heights) {
    OsmTrack track = new OsmTrack();
    for (int i = 0; i < heights.length; i++) {
      OsmPathElement node = OsmPathElement.create(188720000 + i * 1000, 140000000, heights[i], null);
      node.message = new MessageData();
      node.message.ele = heights[i];
      node.message.wayKeyValues = "highway=residential name=section" + Math.max(1, i);
      node.message.linkdist = 999; // Recalculation must replace stale distances as well.
      track.nodes.add(node);
    }
    return track;
  }

  @Test
  public void repairedElevationUpdatesReportedSlopeAndFilterMembership() throws Exception {
    OsmTrack track = track((short) 400, Short.MIN_VALUE, (short) 480, (short) 480);
    RoutingEngine engine = engine();
    engine.postElevationCheck(track);
    engine.roundTripOps().recalcTrack(track);
    assertEquals(440, track.nodes.get(1).getSElev());
    assertEquals(10.0 / 72 * 100, track.nodes.get(1).message.gradient, 0.001);
    assertEquals(2, track.filterSegments(-1, -1, 5, -1).size());
    assertEquals(1, track.filterSegments(-1, -1, -1, 1).size());
  }

  @Test
  public void finalGeometrySetsSignedUphillAndDownhillSlopes() {
    OsmTrack track = track((short) 400, (short) 440, (short) 400);
    engine().roundTripOps().recalcTrack(track);
    assertEquals(10.0 / 72 * 100, track.nodes.get(1).message.gradient, 0.001);
    assertEquals(-10.0 / 72 * 100, track.nodes.get(2).message.gradient, 0.001);
    List<String> rows = track.aggregateMessages();
    assertEquals("139", rows.get(0).split("\t", -1)[13]);
    assertEquals("-139", rows.get(1).split("\t", -1)[13]);
    assertEquals(2, track.filterSegments(-1, -1, 10, 15).size());
  }

  @Test
  public void inverseMessagesDescribeTheFollowingSegment() {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = "../misc/profiles2/trekking.brf";
    RoutingEngine engine = new RoutingEngine(null, null, new File("."), new ArrayList<>(), rc);
    rc.inverseRouting = true;
    OsmTrack track = track((short) 400, (short) 440, (short) 400);
    track.nodes.get(0).message.wayKeyValues = "highway=primary";
    track.nodes.get(1).message.wayKeyValues = "highway=secondary";
    track.nodes.get(2).message.wayKeyValues = null;
    engine.roundTripOps().recalcTrack(track);
    List<String> rows = track.aggregateMessages();
    assertEquals(2, rows.size());
    assertEquals("139", rows.get(0).split("\t", -1)[13]);
    assertEquals("-139", rows.get(1).split("\t", -1)[13]);
    assertEquals("72", rows.get(0).split("\t", -1)[3]);
    assertEquals("72", rows.get(1).split("\t", -1)[3]);
  }

  @Test
  public void unknownElevationIsBlankAndExcludedOnlyByGradientFilters() {
    OsmTrack track = track(Short.MIN_VALUE, Short.MIN_VALUE);
    engine().roundTripOps().recalcTrack(track);
    assertTrue(Float.isNaN(track.nodes.get(1).message.gradient));
    assertEquals("", track.aggregateMessages().get(0).split("\t", -1)[13]);
    assertEquals(1, track.filterSegments(50, -1, -1, -1).size());
    assertEquals(0, track.filterSegments(-1, -1, -1, 1).size());
    assertEquals(0, track.filterSegments(-1, -1, 0, -1).size());
  }

  @Test
  public void clippedEndpointsUseTheRiddenElevationInterval() throws Exception {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = new File("../misc/profiles2/gravel.brf").getCanonicalPath();
    ProfileCache.parseProfile(rc);
    try {
      OsmNode source = new OsmNode(188720000, 140000000);
      OsmNode target = new OsmNode(188724000, 140000000);
      source.selev = 400; // 100 m
      target.selev = 560; // 140 m
      OsmLink link = new OsmLink(source, target);
      int[] tags = rc.expctxWay.createNewLookupData();
      rc.expctxWay.addLookupValue("highway", "track", tags);
      link.descriptionBitmap = rc.expctxWay.encode(tags);
      OsmNodeNamed from = new OsmNodeNamed();
      from.ilon = 188721000;
      from.ilat = 140000000;
      from.radius = 1.5;
      OsmNodeNamed to = new OsmNodeNamed();
      to.ilon = 188723000;
      to.ilat = 140000000;
      to.radius = 1.5;
      rc.setWaypoint(from, to, false);
      OsmPath start = rc.createPath(new OsmLink(null, source));
      OsmPath path = rc.createPath(start, link, null, true);
      assertTrue(path.cost >= 0);
      OsmPathElement end = path.originElement;
      assertEquals(to.ilon, end.getILon());
      assertEquals(from.ilon, end.origin.getILon());
      OsmTrack clipped = new OsmTrack();
      clipped.nodes.add(end.origin);
      clipped.nodes.add(end);
      short exportedHeight = end.getSElev();
      engine().roundTripOps().recalcTrack(clipped);
      assertEquals("Reporting must not alter exported geometry elevations", exportedHeight, end.getSElev());
      // The ridden interval is 110 m -> 130 m, not 110 m -> the full endpoint's 140 m.
      assertEquals(20.0 / end.origin.calcDistance(end) * 100, end.message.gradient, 0.001);
      end.setSElev((short) 528); // A later repair replaces the exported endpoint with 132 m.
      engine().roundTripOps().recalcTrack(clipped);
      assertEquals(22.0 / end.origin.calcDistance(end) * 100, end.message.gradient, 0.001);
    } finally {
      rc.unsetWaypoint();
      ProfileCache.releaseProfile(rc);
    }
  }
}
