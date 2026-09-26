package btools.router.roundtrip;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.router.MessageData;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;
import btools.router.RoutingContext;
import btools.util.CheapRuler;

import static org.junit.Assert.*;

/** Exercises the actual cleanup call that used to splice sparse search nodes. */
public class BulgeRepairConstructionTest {
  @Test
  public void insertsDetailedConnectorWithExactClippedEndpoints() {
    Fixture f = new Fixture();
    f.repair();
    assertEquals(1, f.details);
    assertEquals(1, f.walks);
    assertFalse("Interior repairs must not use the opening heading", f.searchForced);
    assertTrue(f.track.nodes.contains(f.middle));
    assertFalse(f.track.nodes.contains(f.overshoot));
    assertTrue(f.track.nodes.size() < f.original.size());
    assertTrue(f.context.forceUseStartDirection);
    assertTrue(f.context.startDirectionValid);
  }

  @Test
  public void rejectsRawFallbackInsteadOfPublishingSearchChords() {
    Fixture f = new Fixture();
    f.rawFallback = true;
    f.repair();
    assertEquals(1, f.details);
    assertEquals(f.original, f.track.nodes);
  }

  @Test
  public void rejectsNearbyButDifferentConnectorEndpoint() {
    Fixture f = new Fixture();
    f.displacedEndpoint = true;
    f.repair();
    assertEquals(1, f.details);
    assertEquals(f.original, f.track.nodes);
  }

  @Test
  public void rejectsWholeRouteWithIllegalSpliceTurn() {
    Fixture f = new Fixture();
    f.price = -1;
    f.repair();
    assertEquals(1, f.walks);
    assertEquals(f.original, f.track.nodes);
    assertEquals(12, f.waypoints.get(1).indexInTrack);
  }

  @Test
  public void budgetFailureDuringValidationLeavesOriginalUntouched() {
    Fixture f = new Fixture();
    f.deadline = System.currentTimeMillis() + 10000;
    f.failure = FinalizationOutcome.TIMEOUT;
    try {
      f.repair();
      fail("Deadline must propagate");
    } catch (RefineBudget.Exceeded e) {
      assertEquals(FinalizationOutcome.TIMEOUT, e.outcome);
    }
    assertEquals(f.original, f.track.nodes);
    assertEquals(12, f.waypoints.get(1).indexInTrack);
    assertTrue(f.context.forceUseStartDirection);
    assertTrue(f.context.startDirectionValid);
  }

  @Test
  public void cancellationDuringDetailRestoresHeadingAndOriginal() {
    Fixture f = new Fixture();
    f.cancelDetail = true;
    try {
      f.repair();
      fail("Cancellation must propagate");
    } catch (RefineBudget.Exceeded e) {
      assertEquals(FinalizationOutcome.CANCELLED, e.outcome);
    }
    assertEquals(f.original, f.track.nodes);
    assertTrue(f.context.forceUseStartDirection);
    assertTrue(f.context.startDirectionValid);
  }

  @Test
  public void expiredRequestBudgetPreventsRepairAndRestoresEarlierScope() {
    Fixture f = new Fixture();
    f.deadline = Long.MAX_VALUE;
    f.remainingRequestMs = -100;
    try {
      f.repair();
      fail("Expired request must propagate");
    } catch (RefineBudget.Exceeded e) {
      assertEquals(FinalizationOutcome.TIMEOUT, e.outcome);
    }
    assertEquals(Long.MAX_VALUE, f.deadline);
    assertEquals(f.original, f.track.nodes);
    assertEquals(0, f.details);
  }

  @Test
  public void ordinaryGenerationKeepsFinishedRouteWhenOptionalRepairBudgetExpires() {
    Fixture f = new Fixture();
    f.remainingRequestMs = -100;
    f.repair();
    assertEquals(f.original, f.track.nodes);
    assertEquals(0, f.details);
    assertEquals(0, f.deadline);
  }

  private static final class Fixture {
    final OsmTrack track = new OsmTrack();
    final List<OsmPathElement> original;
    final List<MatchedWaypoint> waypoints;
    final RoutingContext context = new RoutingContext();
    final RoundTripEngineOps ops;
    OsmPathElement middle;
    OsmPathElement overshoot;
    OsmTrack raw;
    long deadline;
    long remainingRequestMs = Long.MAX_VALUE;
    int details;
    int walks;
    int price = 20000;
    boolean searchForced;
    boolean rawFallback;
    boolean displacedEndpoint;
    boolean cancelDetail;
    FinalizationOutcome failure;

    Fixture() {
      track.nodes.add(node(0, -10000));
      for (int y = 0; y <= 500; y += 100) track.nodes.add(node(0, y));
      for (int x = -150; x >= -600; x -= 150) track.nodes.add(node(x, 500));
      track.nodes.add(node(-600, 650));
      track.nodes.add(node(-600, 800));
      track.nodes.add(node(-600, 900));
      for (int x = -450; x <= 0; x += 150) track.nodes.add(node(x, 900));
      for (int y = 1000; y <= 1400; y += 100) track.nodes.add(node(0, y));
      track.nodes.add(node(0, 12000));
      int cost = 0;
      for (int i = 1; i < track.nodes.size(); i++) {
        int distance = track.nodes.get(i - 1).calcDistance(track.nodes.get(i));
        track.distance += distance;
        cost += distance * (i > 6 && i <= 17 ? 10 : 1);
        track.nodes.get(i).cost = cost;
      }
      track.cost = cost;
      original = new ArrayList<>(track.nodes);
      waypoints = Arrays.asList(waypoint(0), waypoint(12), waypoint(track.nodes.size() - 1));
      context.forceUseStartDirection = true;
      context.startDirectionValid = true;
      ops = (RoundTripEngineOps) Proxy.newProxyInstance(RoundTripEngineOps.class.getClassLoader(),
        new Class<?>[]{RoundTripEngineOps.class}, (proxy, method, args) -> {
          switch (method.getName()) {
            case "routingContext": return context;
            case "remainingRequestBudgetMs": return remainingRequestMs;
            case "refinementDeadline": return deadline;
            case "setRefinementDeadline": deadline = (long) args[0]; return null;
            case "isTerminated": return false;
            case "resetCache":
            case "checkRefinementBudget":
            case "logInfo":
              return null;
            case "matchWaypointsToNodes":
              @SuppressWarnings("unchecked")
              List<MatchedWaypoint> matched = (List<MatchedWaypoint>) args[0];
              for (MatchedWaypoint wp : matched) {
                wp.crosspoint = wp.waypoint;
                wp.node1 = wp.waypoint;
                wp.node2 = new OsmNode(wp.waypoint.ilon + 100, wp.waypoint.ilat);
              }
              return null;
            case "findTrackUnguided":
              // Search geometry ends at the graph node beyond the clipped mouth.
              searchForced = context.forceUseStartDirection;
              raw = new OsmTrack();
              raw.nodes.add(element(((MatchedWaypoint) args[1]).crosspoint));
              overshoot = node(0, 1100);
              raw.nodes.add(overshoot);
              raw.distance = 400;
              raw.cost = 400;
              return raw;
            case "retrackForDetail":
              details++;
              assertFalse(context.forceUseStartDirection);
              if (cancelDetail) throw new RefineBudget.Exceeded(FinalizationOutcome.CANCELLED);
              if (rawFallback) return raw;
              OsmTrack detailed = new OsmTrack();
              detailed.nodes.add(element(((MatchedWaypoint) args[1]).crosspoint));
              middle = node(0, 700);
              detailed.nodes.add(middle);
              OsmNode end = ((MatchedWaypoint) args[2]).crosspoint;
              detailed.nodes.add(element(displacedEndpoint ? new OsmNode(end.ilon + 1, end.ilat) : end));
              for (OsmPathElement n : detailed.nodes) n.message = new MessageData();
              detailed.distance = 400;
              detailed.cost = 400;
              return detailed;
            case "walkPathCost":
              walks++;
              OsmTrack candidate = (OsmTrack) args[0];
              assertNotSame(track, candidate);
              assertEquals(original, track.nodes);
              assertSame(original.get(0), candidate.nodes.get(0));
              assertSame(original.get(original.size() - 1), candidate.nodes.get(candidate.nodes.size() - 1));
              assertTrue("Full-route validation must restore the request heading", context.forceUseStartDirection);
              if (failure != null) throw new RefineBudget.Exceeded(failure);
              return price;
            default: throw new AssertionError("Unexpected operation " + method.getName());
          }
        });
    }

    MatchedWaypoint waypoint(int index) {
      MatchedWaypoint wp = new MatchedWaypoint();
      wp.name = "rt" + index;
      wp.indexInTrack = index;
      wp.crosspoint = new OsmNode(track.nodes.get(index).getILon(), track.nodes.get(index).getILat());
      return wp;
    }

    void repair() {
      new WaypointSnapper(ops, ops, ops).repairViaPinnedBulges(track, waypoints);
    }
  }

  private static OsmPathElement element(OsmNode n) {
    return OsmPathElement.create(n.ilon, n.ilat, (short) 0, null);
  }

  private static OsmPathElement node(double x, double y) {
    double[] scales = CheapRuler.getLonLatToMeterScales(140000000);
    return OsmPathElement.create(188720000 + (int) Math.round(x / scales[0]),
      140000000 + (int) Math.round(y / scales[1]), (short) 0, null);
  }
}
