package btools.router;

import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Round-trip feature/scenario coverage on the Dreieich fixture: each generation
 * strategy and the same-way-back mode must still produce a valid loop (or fail
 * cleanly). Scenarios are held to the same structural invariants as
 * {@link RoundTripInvariantTest}.
 */
public class RoundTripScenarioTest {

  private RoutingEngine engine(String profile, int direction, int radius, Consumer<RoutingContext> tweak) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = RoundTripFixture.START_ILON;
    start.ilat = RoundTripFixture.START_ILAT;
    wps.add(start);

    RoutingContext rc = new RoutingContext();
    rc.localFunction = RoundTripFixture.profileFile(profile).getAbsolutePath();
    rc.startDirection = direction;
    rc.roundTripDistance = radius;
    rc.turnInstructionMode = 2;
    tweak.accept(rc);

    RoutingEngine re = new RoutingEngine(null, null, RoundTripFixture.segmentDir(), wps, rc,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  private OsmTrack routeOk(String profile, int direction, int radius, Consumer<RoutingContext> tweak) {
    RoutingEngine re = engine(profile, direction, radius, tweak);
    Assert.assertNull("routing failed: " + re.getErrorMessage(), re.getErrorMessage());
    return re.getFoundTrack();
  }

  /** The explicit WAYPOINT strategy must produce valid loops for every cycling profile. */
  @Test
  public void waypointAlgorithmValidLoop() {
    for (String profile : new String[]{"trekking", "fastbike", "gravel", "mtb"}) {
      OsmTrack t = routeOk(profile, 90, 1500, rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.WAYPOINT);
      RoundTripFixture.assertValidLoop(t, "waypoint_" + profile, 30.0);
    }
  }

  /** The ISOCHRONE strategy (the small-radius default) must produce valid loops. */
  @Test
  public void isochroneAlgorithmValidLoop() {
    for (String profile : new String[]{"trekking", "fastbike", "gravel", "mtb"}) {
      OsmTrack t = routeOk(profile, 90, 1500, rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.ISOCHRONE);
      RoundTripFixture.assertValidLoop(t, "isochrone_" + profile, 30.0);
    }
  }

  /**
   * allowSamewayback is an out-and-back: at a feasible config it returns to the origin
   * (high reuse by design, but it must close with no beelines).
   */
  @Test
  public void allowSamewaybackProducesClosedLoop() {
    OsmTrack t = routeOk("trekking", 270, 1000, rc -> rc.allowSamewayback = true);
    RoundTripFixture.assertValidLoop(t, "samewayback", 100.0);
  }

  /**
   * When an out-and-back cannot return to its origin (the return leg has no path back
   * in that direction), the engine must report a clear failure with no track rather
   * than returning a one-way stub as success.
   */
  @Test
  public void allowSamewaybackNonClosingFailsCleanly() {
    RoutingEngine re = engine("trekking", 90, 1000, rc -> rc.allowSamewayback = true);
    Assert.assertNotNull("expected a failure for a non-closing out-and-back", re.getErrorMessage());
    Assert.assertNull("error set but a track was still returned", re.getFoundTrack());
  }
}
