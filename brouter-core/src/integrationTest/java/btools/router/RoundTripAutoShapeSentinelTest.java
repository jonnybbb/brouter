package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.Assume;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;

import btools.router.roundtrip.LoopQualityMetrics;
import btools.router.roundtrip.RoundTripAlgorithm;

/**
 * Shape sentinels for what AUTO actually SHIPS, on the two cells of the
 * 2026-07-25 root-cause investigation. Both cells exposed the same defect
 * chain: a candidate pool of one (greedy skipped by graph-native absorption
 * or hard-failing on mtb) ranked by a scorer whose compactness term read the
 * convex hull — so a serpentine crumple (Vosges: net enclosed-area
 * compactness 0.03) and a necked two-lobe (Crete: 0.13) shipped unopposed
 * while the FAST tier produced visibly better loops the competition never
 * saw.
 *
 * <p>The assertion is on the shipped track's NET enclosed-area compactness —
 * the isoperimetric measure a rider's eye agrees with — with floors set
 * between the measured defect values and the known-available better loops
 * (Crete FAST ring ~0.4, Vosges FAST polygon ~0.32), leaving anti-flap
 * margin on both sides. Lenient gate (production semantics): the sentinel
 * grades what a real request ships.
 */
public class RoundTripAutoShapeSentinelTest {

  @Rule
  public org.junit.rules.Timeout perTestTimeout = org.junit.rules.Timeout.builder()
    .withTimeout(5, java.util.concurrent.TimeUnit.MINUTES)
    .withLookingForStuckThread(true)
    .build();

  private File projectDir;

  @Before
  public void setUp() throws Exception {
    projectDir = new File(".").getCanonicalFile().getParentFile();
  }

  @Test
  public void autoVosges75kmMtbWestShipsARealLoopNotACrumple() throws Exception {
    double netCompact = shippedNetCompactness(LoopTestRegion.VOSGES_LA_BRESSE, "mtb",
      11_937, 270);
    org.junit.Assert.assertTrue(String.format(Locale.US,
      "vosges 75km mtb W: AUTO shipped a crumple (net enclosed-area compactness %.3f < 0.10)",
      netCompact), netCompact >= 0.10);
  }

  @Test
  public void autoCreteSenesi75kmGravelSouthShipsARingNotANeckedTwoLobe() throws Exception {
    double netCompact = shippedNetCompactness(LoopTestRegion.CRETE_SENESI, "gravel",
      11_937, 180);
    org.junit.Assert.assertTrue(String.format(Locale.US,
      "crete_senesi 75km gravel S: AUTO shipped a necked loop (net enclosed-area compactness %.3f < 0.20)",
      netCompact), netCompact >= 0.20);
  }

  private double shippedNetCompactness(LoopTestRegion region, String profile,
                                       int searchRadius, int directionDeg) throws Exception {
    File segDir = new File(projectDir, "segments4");
    File segFile = new File(segDir, region.segmentFile);
    Assume.assumeTrue("segment file missing: " + segFile, segFile.exists());
    File profileFile = new File(projectDir, "misc/profiles2/" + profile + ".brf");
    Assume.assumeTrue("profile file missing: " + profileFile, profileFile.exists());

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = region.ilonFor(profile);
    start.ilat = region.ilatFor(profile);
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = directionDeg;
    rctx.roundTripDistance = searchRadius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    rctx.roundTripStrictQuality = false;

    RoutingEngine engine = new RoutingEngine(null, null, segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    engine.quite = true;
    engine.doRun(0);
    OsmTrack track = engine.getFoundTrack();
    org.junit.Assert.assertNotNull("AUTO produced no track", track);
    return LoopQualityMetrics.enclosedAreaCompactness(track.nodes, track.distance);
  }
}
