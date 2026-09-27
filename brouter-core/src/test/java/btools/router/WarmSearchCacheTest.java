package btools.router;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import btools.router.roundtrip.RoundTripAlgorithm;

/**
 * The retained search graph is an optimization: a round trip planned on it
 * must be the same round trip, node for node, that the per-leg cache rebuild
 * produces. Runs both on the bundled Dreieich fixture.
 */
public class WarmSearchCacheTest {

  private static List<OsmPathElement> loopOrSkip(RoutingEngine re, String label) {
    RoundTripFixture.assertNoEngineErrorOrSkip(re, label);
    OsmTrack track = re.getFoundTrack();
    assertTrue(label + ": loop expected", track != null && track.nodes != null && track.nodes.size() > 10);
    return track.nodes;
  }

  private static void assertSameNodes(String label, List<OsmPathElement> expected, List<OsmPathElement> actual) {
    assertEquals(label + ": node count", expected.size(), actual.size());
    for (int i = 0; i < expected.size(); i++) {
      OsmPathElement e = expected.get(i);
      OsmPathElement a = actual.get(i);
      assertTrue(label + ": node " + i + " differs",
        e.getILon() == a.getILon() && e.getILat() == a.getILat());
    }
  }

  @Test
  public void greedyLoopIsIdenticalWithAndWithoutRetainedGraph() {
    RoutingEngine warm = RoundTripFixture.engine("gravel", 90, 1500,
      rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY);
    RoutingEngine cold = RoundTripFixture.engine("gravel", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripWarmCache = false;
    });
    List<OsmPathElement> warmNodes = loopOrSkip(warm, "greedy_warm");
    List<OsmPathElement> coldNodes = loopOrSkip(cold, "greedy_cold");
    assertSameNodes("greedy", coldNodes, warmNodes);
    assertTrue("planner legs were served from the retained graph", warm.getWarmSearchCount() > 0);
    assertEquals("the switch really disables retention", 0, cold.getWarmSearchCount());
  }

  @Test
  public void autoLoopIsIdenticalWithAndWithoutRetainedGraph() {
    RoutingEngine warm = RoundTripFixture.engine("fastbike", 0, 1500);
    RoutingEngine cold = RoundTripFixture.engine("fastbike", 0, 1500, rc -> rc.roundTripWarmCache = false);
    assertSameNodes("auto", loopOrSkip(cold, "auto_cold"), loopOrSkip(warm, "auto_warm"));
  }
}
