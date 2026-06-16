package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Does the RCS appeal-weight actually make AUTO ship more appealing routes? Routes AUTO loops for
 * Basel+Freiburg and reports the shipped route's RoadCharacterScore appeal. Run with
 * -Dloop.charweight=0.07 (old weight) vs default 0.17 (rebalanced): higher mean appeal at 0.17 means
 * the rebalance is delivering on "an appealing route should score higher".
 */
public class AppealSelectionTest {

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {30000, 50000, 80000};
  private static final int[] RADII = {4800, 8000, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};

  private File segDir, gravel, out;

  @Test
  public void appeal() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    out = new File(projectDir, "build/appeal"); out.mkdirs();

    System.out.println("=== AUTO shipped-route appeal (charweight="
      + System.getProperty("loop.charweight", "0.17") + ", steer off) ===");
    int n = 0; double sumAppeal = 0, sumResid = 0;
    for (LoopTestRegion region : REGIONS) {
      for (int i = 0; i < TARGETS.length; i++) {
        for (double dir : DIRS) {
          double[] r = appealOf(region.ilon, region.ilat, RADII[i], (int) dir);
          if (r == null) continue;
          n++; sumAppeal += r[0]; sumResid += r[1];
        }
      }
    }
    System.out.println(String.format("AGG n=%d  meanAppeal=%.3f  meanResid=%.0f%%", n, sumAppeal / n, sumResid / n * 100));
  }

  private double[] appealOf(int cx, int cy, int radius, int dir) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.startDirection = dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
      rc.roundTripStrictQuality = false;
      File o = new File(out, "_a");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.size() < 3) return null;
      RoadCharacterScore rcs = RoadCharacterScore.compute(t, "gravel");
      return new double[]{rcs.appeal, rcs.residentialFrac};
    } catch (Exception e) { return null; }
  }
}
