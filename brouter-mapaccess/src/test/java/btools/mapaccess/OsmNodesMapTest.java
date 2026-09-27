package btools.mapaccess;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import org.junit.Test;

public class OsmNodesMapTest {

  @Test
  public void removeDropsNodesUnlessWovenNodesAreKept() {
    OsmNodesMap map = new OsmNodesMap();
    OsmNode n = new OsmNode(1000, 2000);
    map.put(n);
    assertEquals(1, map.size());

    map.remove(n);
    assertNull("default: a woven node leaves the map", map.get(1000, 2000));
    assertEquals(0, map.size());

    map.keepWoven = true;
    map.put(n);
    map.remove(n);
    assertSame("keepWoven: the node stays resolvable by position", n, map.get(1000, 2000));
    assertEquals(1, map.size());
  }

  @Test
  public void endNodesStayInTheMapEitherWay() {
    OsmNodesMap map = new OsmNodesMap();
    OsmNode end = new OsmNode(5, 6);
    map.put(end);
    map.endNode1 = end;
    map.remove(end);
    assertSame(end, map.get(5, 6));
  }
}
