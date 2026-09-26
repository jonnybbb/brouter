package btools.router;

import java.io.File;

import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import org.junit.Assert;
import org.junit.Test;

/** Exercises real way evaluation and endpoint clipping without segment tiles. */
public class ClippedEndpointMessageTest {
  @Test
  public void clippedEndpointMessageDescribesTheRoutedPartialEdge() throws Exception {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = new File("../misc/profiles2/gravel.brf").getCanonicalPath();
    ProfileCache.parseProfile(rc);
    try {
      // The clipped geometry endpoint and full edge endpoint from the native
      // Liestal replay's first message/geometry mismatch (message row 37).
      OsmNode source = new OsmNode(187684015, 137431951);
      OsmNode target = new OsmNode(187683870, 137431922);
      source.selev = target.selev = 1600;
      OsmLink link = new OsmLink(source, target);
      int[] tags = rc.expctxWay.createNewLookupData();
      rc.expctxWay.addLookupValue("highway", "track", tags);
      rc.expctxWay.addLookupValue("surface", "gravel", tags);
      link.descriptionBitmap = rc.expctxWay.encode(tags);

      OsmNodeNamed endpoint = new OsmNodeNamed();
      endpoint.ilon = 187683935;
      endpoint.ilat = 137431935;
      endpoint.radius = 1.5;
      rc.setWaypoint(endpoint, true);
      OsmPath start = rc.createPath(new OsmLink(null, source));
      OsmPath path = rc.createPath(start, link, null, true);
      Assert.assertTrue("route must succeed", path.cost >= 0);
      Assert.assertTrue("real endpoint clipping must run", rc.shortestmatch);
      OsmPathElement clipped = path.originElement;
      Assert.assertEquals(endpoint.ilon, clipped.getILon());
      Assert.assertEquals(endpoint.ilat, clipped.getILat());
      Assert.assertNotEquals("the edge continues beyond the routed endpoint",
        target.getILon(), clipped.getILon());
      Assert.assertNotEquals(target.getILat(), clipped.getILat());
      Assert.assertTrue("way provenance must remain the traversed gravel edge",
        clipped.message.wayKeyValues.contains("surface=gravel"));
      Assert.assertEquals("distance must stop at the clipped endpoint",
        source.calcDistance(clipped), clipped.message.linkdist);
      Assert.assertEquals("message elevation stays aligned with geometry",
        clipped.getSElev(), clipped.message.ele);

      // Native greedy assembly retains this node/message without calling
      // OsmTrack.appendTrack's coordinate normalization.
      OsmTrack nativeLeg = new OsmTrack();
      nativeLeg.nodes.add(clipped.origin);
      nativeLeg.nodes.add(clipped);
      String[] row = nativeLeg.aggregateMessages().get(0).split("\\t");
      Assert.assertEquals("the message endpoint must be the routed endpoint",
        clipped.getILon() - 180000000, Integer.parseInt(row[0]));
      Assert.assertEquals(clipped.getILat() - 90000000, Integer.parseInt(row[1]));
    } finally {
      rc.unsetWaypoint();
      ProfileCache.releaseProfile(rc);
    }
  }
}
