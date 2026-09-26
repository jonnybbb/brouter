package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripQualityResult;

/**
 * FAST shape retry on real map data. The optimized FAST placement is faster
 * but can produce loops the quality gate rejects where the legacy
 * probe+envelope placement stays clean (Basel fastbike 180 km: a 221.8 km
 * loop with a 6.7 km profile-hostile stretch). On a gate rejection the FAST
 * tier retries with the legacy placement and ships the better outcome.
 *
 * <p>The contract under test is IMPROVEMENT, not a fixed loop: with the
 * retry on, the shipped verdict must be accepted, or strictly better than
 * the retry-off baseline (fewer self-crossings). When map data changes and
 * the optimized loop passes the gate outright, the retry never fires and
 * both runs are identical and accepted — the assertion still holds.
 */
public class RoundTripFastShapeRetryIntegrationTest {

  private RoundTripQualityResult basel180Verdict() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, LoopTestRegion.BASEL);

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = (int) ((7.590 + 180.0) * 1e6);
    start.ilat = (int) ((47.560 + 90.0) * 1e6);
    wps.add(start);

    RoutingContext rc = new RoutingContext();
    rc.localFunction = new File(projectDir, "misc/profiles2/fastbike.brf").getAbsolutePath();
    rc.startDirection = 0;
    rc.roundTripLength = 180000;
    rc.roundTripAlgorithm = RoundTripAlgorithm.WAYPOINT;

    RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(0);
    Assert.assertNotNull("FAST must form a loop", re.getFoundTrack());
    RoundTripQualityResult quality = re.getLastRoundTripQuality();
    Assert.assertNotNull("gate verdict recorded", quality);
    return quality;
  }

  @Test
  public void shapeRetryImprovesTheShippedVerdict() throws Exception {
    RoundTripQualityResult baseline;
    try {
      System.setProperty("roundtrip.fast.shape.retry", "false");
      baseline = basel180Verdict();
    } finally {
      System.clearProperty("roundtrip.fast.shape.retry");
    }
    RoundTripQualityResult withRetry = basel180Verdict();

    if (baseline.isAccepted()) {
      // Optimized placement passed outright — the retry must not have made
      // anything worse.
      Assert.assertTrue("no-retry-needed case stays accepted", withRetry.isAccepted());
      return;
    }
    int baselineCrossings = Math.max(0, baseline.getSelfIntersections());
    boolean improved = withRetry.isAccepted()
      || (withRetry.getSelfIntersections() >= 0
        && withRetry.getSelfIntersections() < baselineCrossings);
    Assert.assertTrue("shape retry ships an accepted or strictly cleaner loop"
      + " (baseline: " + baseline + ", with retry: " + withRetry + ")", improved);
  }
}
