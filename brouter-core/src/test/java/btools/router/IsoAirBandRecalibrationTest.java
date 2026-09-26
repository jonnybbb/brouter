package btools.router;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import btools.router.roundtrip.RoundTripAlgorithm;

/**
 * A high-cost profile must still be able to build a greedy loop.
 *
 * <p>The isochrone calibration band is defined in cost units ([0.7, 1.0] x
 * searchRadius as a cost). On a profile whose cost per air-metre is well above
 * the bike scale the band lies within a few dozen metres of the start, below the
 * 50 m sample floor, so the calibration never fires and the 4 x searchRadius
 * floor stops the start expansion long before the candidate ring: both greedy
 * planners then fail with "could not build any loop". Measured on the javik
 * gravel profile starting in town (25-120 cost per air-metre); reproduced here
 * with a uniform costfactor of 9 and a 500 m search radius on the Dreieich test
 * tile, where the start expansion stops after ~11 nodes without the fix.
 */
public class IsoAirBandRecalibrationTest {

  /**
   * Uniform costfactor 9 (below the default snap-reject ceiling of 10), ~12 cost per air-metre on
   * the test tile. At the 500 m radius the test uses, the cost band [0.7, 1.0] x 500 lies ~30-40 m
   * from the start, inside the 50 m sample floor, and the 4x floor (2000) reaches ~180 m of the
   * 350 m the band needs: the same starvation the javik gravel profile shows in town at its larger
   * radii (25-120 cost per air-metre, where per-way costs are amortised over short streets).
   */
  private static final String COSTLY_PROFILE = String.join("\n",
    "---context:global",
    "assign processUnusedTags = false",
    "assign validForBikes = true",
    "",
    "---context:way",
    "assign turncost = 0",
    "assign initialcost = 0",
    "assign costfactor",
    "  switch and highway= not route=ferry  100000 9",
    "",
    "---context:node",
    "assign initialcost = 0",
    "");

  @Rule
  public TemporaryFolder tmp = new TemporaryFolder();

  private File projectDir;

  @Before
  public void before() throws Exception {
    // Gradle sets cwd to the module directory (brouter-core/)
    projectDir = new File(".").getCanonicalFile().getParentFile();
  }

  @Test
  public void isoGreedyBuildsALoopOnAHighCostProfile() throws Exception {
    RoutingContext rctx = new RoutingContext();
    rctx.startDirection = 0;
    rctx.roundTripDistance = 500;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISO_GREEDY;

    RoutingEngine re = roundTrip(8.720, 50.000, rctx, costlyProfile());

    Assert.assertNull("ISO_GREEDY must build a loop on a high-cost profile: " + re.getErrorMessage(),
      re.getErrorMessage());
    Assert.assertNotNull(re.getFoundTrack());
    Assert.assertTrue("loop should be of the requested order of length, was " + re.getFoundTrack().distance,
      re.getFoundTrack().distance > 1500);
  }

  @Test
  public void starvedOnlyWhenShortOfTheAirBand() {
    // band opens at ISO_CALIBRATION_SAMPLE_LO (0.7) x searchRadius
    Assert.assertTrue(RoutingEngine.isoBudgetStarved(150, 6366));
    Assert.assertTrue(RoutingEngine.isoBudgetStarved(4455, 6366));
    Assert.assertFalse(RoutingEngine.isoBudgetStarved(4457, 6366));
    Assert.assertFalse(RoutingEngine.isoBudgetStarved(9000, 6366));
  }

  @Test
  public void airBandClosesAtTwiceTheFirstBandCost() {
    // first band pop at cost 7000 -> close at 7000 / 0.7^2 = 14285, so bearings up to ~2x more
    // expensive than the cheapest one still reach the band
    Assert.assertEquals(14285, RoutingEngine.airBandCloseCost(7000));
    Assert.assertEquals(Integer.MAX_VALUE / 2, RoutingEngine.airBandCloseCost(Integer.MAX_VALUE));
  }

  @Test
  public void airBandCostIsTheMedianAcrossDirectionsNotAcrossPops() {
    double[] perBucket = new double[36];
    java.util.Arrays.fill(perBucket, Double.POSITIVE_INFINITY);
    perBucket[3] = 10;  // one cheap arterial ...
    perBucket[12] = 40;
    perBucket[20] = 20;
    perBucket[30] = 60;
    // ... counts once: sorted 10, 20, 40, 60 -> upper median 40
    Assert.assertEquals(40.0, RoutingEngine.airBandCostPerMeter(perBucket), 1e-9);
  }

  @Test
  public void airBandCostNeedsThreeDirections() {
    double[] perBucket = new double[36];
    java.util.Arrays.fill(perBucket, Double.POSITIVE_INFINITY);
    perBucket[0] = 10;
    perBucket[18] = 20;
    Assert.assertTrue(Double.isNaN(RoutingEngine.airBandCostPerMeter(perBucket)));
    perBucket[9] = 30;
    Assert.assertEquals(20.0, RoutingEngine.airBandCostPerMeter(perBucket), 1e-9);
  }

  @Test
  public void airBandGivesUpAtThirtyTwoTimesTheStarvedBudget() {
    Assert.assertFalse(RoutingEngine.airBandGiveUp(25464 * 32, 25464));
    Assert.assertTrue(RoutingEngine.airBandGiveUp(25464 * 32 + 1, 25464));
    // overflow-safe comparison for large budgets
    Assert.assertFalse(RoutingEngine.airBandGiveUp(Integer.MAX_VALUE, Integer.MAX_VALUE / 2));
  }

  private File costlyProfile() throws IOException {
    File dir = tmp.newFolder("profiles");
    Files.copy(new File(projectDir, "misc/profiles2/lookups.dat").toPath(),
      new File(dir, "lookups.dat").toPath(), StandardCopyOption.REPLACE_EXISTING);
    File profile = new File(dir, "costly.brf");
    Files.write(profile.toPath(), COSTLY_PROFILE.getBytes(StandardCharsets.UTF_8));
    return profile;
  }

  private RoutingEngine roundTrip(double lon, double lat, RoutingContext rctx, File profile) {
    String out = new File(tmp.getRoot(), "costly").getAbsolutePath();
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = "from";
    n.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    n.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    wplist.add(n);
    rctx.localFunction = profile.getAbsolutePath();
    RoutingEngine re = new RoutingEngine(out, out,
      new File(projectDir, "brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    return re;
  }
}
