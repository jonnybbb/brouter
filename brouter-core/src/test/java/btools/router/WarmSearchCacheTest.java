package btools.router;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.function.Consumer;

import org.junit.Test;

import btools.router.roundtrip.RoundTripAlgorithm;

/**
 * The retained search graph is an optimization: a round trip planned on it
 * must be the same round trip — nodes, distance, cost, climb — that the
 * per-leg cache rebuild produces. Runs both on the bundled Dreieich fixture.
 */
public class WarmSearchCacheTest {

  private static OsmTrack loopOrSkip(RoutingEngine re, String label) {
    RoundTripFixture.assertNoEngineErrorOrSkip(re, label);
    OsmTrack track = re.getFoundTrack();
    assertTrue(label + ": loop expected", track != null && track.nodes != null && track.nodes.size() > 10);
    return track;
  }

  private static void assertSameLoop(String label, OsmTrack expected, OsmTrack actual) {
    assertEquals(label + ": node count", expected.nodes.size(), actual.nodes.size());
    for (int i = 0; i < expected.nodes.size(); i++) {
      OsmPathElement e = expected.nodes.get(i);
      OsmPathElement a = actual.nodes.get(i);
      assertTrue(label + ": node " + i + " differs",
        e.getILon() == a.getILon() && e.getILat() == a.getILat());
    }
    assertEquals(label + ": distance", expected.distance, actual.distance);
    assertEquals(label + ": cost", expected.cost, actual.cost);
    assertEquals(label + ": ascend", expected.ascend, actual.ascend);
  }

  private static RoutingEngine greedy(Consumer<RoutingContext> tweak) {
    return RoundTripFixture.engine("gravel", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      tweak.accept(rc);
    });
  }

  @Test
  public void greedyLoopIsIdenticalWithAndWithoutRetainedGraph() {
    RoutingEngine warm = greedy(rc -> { });
    RoutingEngine cold = greedy(rc -> rc.roundTripWarmCache = false);
    OsmTrack warmLoop = loopOrSkip(warm, "greedy_warm");
    OsmTrack coldLoop = loopOrSkip(cold, "greedy_cold");
    assertSameLoop("greedy", coldLoop, warmLoop);
    assertTrue("planner legs were served from the retained graph", warm.getWarmSearchCount() > 0);
    assertEquals("the fixture fits the memory class without a fallback", 0, warm.getWarmSearchFallbackCount());
    assertEquals("the switch really disables retention", 0, cold.getWarmSearchCount());
  }

  @Test
  public void autoLoopIsIdenticalWithAndWithoutRetainedGraph() {
    RoutingEngine warm = RoundTripFixture.engine("fastbike", 0, 1500);
    RoutingEngine cold = RoundTripFixture.engine("fastbike", 0, 1500, rc -> rc.roundTripWarmCache = false);
    assertSameLoop("auto", loopOrSkip(cold, "auto_cold"), loopOrSkip(warm, "auto_warm"));
  }

  /**
   * A retained graph that outgrows its budget mid-search is released and the
   * leg reruns on the bounded cold cache — the loop is still the cold loop. A
   * one-byte budget has no room for any retained node, so the first pop that
   * weaves trips it on every leg; the fixture is too small to exceed a real
   * memory class within one search.
   */
  @Test
  public void overBudgetSearchFallsBackToColdAndKeepsTheLoop() {
    RoutingEngine none = greedy(rc -> rc.roundTripWarmCacheBytes = 1);
    RoutingEngine cold = greedy(rc -> rc.roundTripWarmCache = false);
    OsmTrack noneLoop = loopOrSkip(none, "greedy_no_budget");
    assertSameLoop("greedy_no_budget", loopOrSkip(cold, "greedy_cold"), noneLoop);
    // Legs whose whole start neighbourhood was woven before the first pop never
    // reach the trigger (parity with the cold search's own check), so not every
    // warm leg falls back — but with no room at all, most must.
    assertTrue("legs must fall back to the cold search", none.getWarmSearchFallbackCount() > 0);
  }

  /** Kinematic models price junction crossings from every link at a node; they keep the cold search. */
  @Test
  public void kinematicProfileKeepsTheColdSearch() {
    RoutingEngine car = RoundTripFixture.engine("car-eco", 90, 1500,
      rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY);
    RoundTripFixture.assertNoEngineErrorOrSkip(car, "car_greedy");
    assertEquals("no leg search on the retained graph for a kinematic model", 0, car.getWarmSearchCount());
  }
}
