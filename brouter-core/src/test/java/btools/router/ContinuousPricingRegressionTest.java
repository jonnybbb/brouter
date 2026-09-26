package btools.router;

import java.util.Arrays;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.router.roundtrip.LoopCostOracle;
import btools.router.roundtrip.LoopPrice;
import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripEngineOps;

import static org.junit.Assert.*;

/** Negative geometry checks and differential raw/detailed pricing on the real fixture. */
public class ContinuousPricingRegressionTest {
  private RoundTripEngineOps ops;
  private List<MatchedWaypoint> waypoints;

  @Before
  public void setup() {
    RoutingEngine engine = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    ops = engine.roundTripOps();
    waypoints = engine.getLastRoundTripResult().getMatchedWaypoints();
  }

  private OsmTrack raw(MatchedWaypoint from, MatchedWaypoint to) {
    OsmTrack track = ops.findTrackTimed("pricing-regression", from, to, null, 5000L);
    assertNotNull(track);
    return track;
  }

  private OsmTrack detail(OsmTrack raw, MatchedWaypoint from, MatchedWaypoint to) {
    OsmTrack track = ops.retrackForDetail(raw, from, to, null);
    assertNotSame("No raw fallback in the reference geometry", raw, track);
    assertNotNull(track);
    return track;
  }

  @Test
  public void rejectsMissingNativeBendEvenWithoutMessages() {
    MatchedWaypoint from = waypoints.get(0);
    MatchedWaypoint to = waypoints.get(1);
    OsmTrack detailed = detail(raw(from, to), from, to);
    assertTrue(LoopCostOracle.priceCost(ops, detailed, from, to) > 0);
    // Pinned native bend from the review reproduction: four transfers between graph nodes.
    assertEquals(77, detailed.nodes.size());
    OsmTrack sparse = new OsmTrack();
    sparse.nodes.addAll(detailed.nodes);
    sparse.nodes.subList(6, 10).clear();
    assertEquals(-1, LoopCostOracle.priceCost(ops, sparse, from, to));
    // Metadata must never decide whether finished geometry is validated strictly.
    OsmTrack noMessages = new OsmTrack();
    for (OsmPathElement p : sparse.nodes) {
      noMessages.nodes.add(OsmPathElement.create(p.getILon(), p.getILat(), p.getSElev(), null));
    }
    assertEquals(-1, LoopCostOracle.priceCost(ops, noMessages, from, to));
  }

  @Test
  public void rejectsDisplacedClosingAndOpeningEndpoints() {
    MatchedWaypoint from = waypoints.get(2);
    MatchedWaypoint to = waypoints.get(3);
    OsmTrack detailed = detail(raw(from, to), from, to);
    assertTrue(LoopCostOracle.priceCost(ops, detailed, from, to) > 0);
    for (int index : new int[]{0, detailed.nodes.size() - 1}) {
      OsmTrack changed = new OsmTrack();
      changed.nodes.addAll(detailed.nodes);
      OsmPathElement original = detailed.nodes.get(index);
      OsmPathElement displaced = OsmPathElement.create(original.getILon() + 1000,
        original.getILat(), original.getSElev(), null);
      displaced.message = original.message;
      changed.nodes.set(index, displaced);
      assertEquals("Off-edge endpoint at " + index, -1, LoopCostOracle.priceCost(ops, changed, from, to));
    }
  }

  private void assertSameContinuousRide(MatchedWaypoint from, MatchedWaypoint via, MatchedWaypoint to) {
    OsmTrack first = raw(from, via);
    OsmTrack second = raw(via, to);
    OsmTrack firstDetailed = detail(first, from, via);
    OsmTrack secondDetailed = detail(second, via, to);
    assertEquals(firstDetailed.nodes.get(firstDetailed.nodes.size() - 1).getIdFromPos(),
      secondDetailed.nodes.get(0).getIdFromPos());
    OsmTrack joined = new OsmTrack();
    joined.appendTrack(firstDetailed);
    joined.appendTrack(secondDetailed);
    joined.setMatchedWaypoints(Arrays.asList(from, via, to));
    int exactCost = LoopCostOracle.priceCost(ops, joined, from, to);
    assertTrue("Reference detailed ride must be priceable", exactCost > 0);
    LoopPrice price = LoopCostOracle.evaluate(ops, Arrays.asList(first, second), Arrays.asList(from, via, to), 0L);
    assertEquals("No invented reversals at via", exactCost, price.cost);
    assertEquals("Distance must describe the continuously priced geometry", joined.distance, price.distance);
  }

  @Test
  public void midEdgeViaDoesNotInventBacktracking() {
    MatchedWaypoint from = waypoints.get(1);
    MatchedWaypoint to = waypoints.get(2);
    OsmTrack detailed = detail(raw(from, to), from, to);
    OsmPathElement point = detailed.nodes.get(3);
    MatchedWaypoint via = new MatchedWaypoint();
    via.name = "mid_edge";
    via.waypoint = new OsmNode(point.getILon(), point.getILat());
    ops.matchWaypointsToNodes(Arrays.asList(via), 50.0);
    assertTrue(via.crosspoint.calcDistance(via.node1) > 30);
    assertTrue(via.crosspoint.calcDistance(via.node2) > 30);
    assertSameContinuousRide(from, via, to);
  }

  @Test
  public void junctionViaUsesClippedDistanceDenominator() {
    for (int leg = 0; leg < 2; leg++) {
      assertSameContinuousRide(waypoints.get(leg), waypoints.get(leg + 1), waypoints.get(leg + 2));
    }
  }
}
