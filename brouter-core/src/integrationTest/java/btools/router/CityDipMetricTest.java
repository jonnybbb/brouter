package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import btools.util.CheapRuler;

/**
 * Deterministic proof that via-steering reduces city dips. Routes Basel+Freiburg loops with GREEDY
 * (so the dense boxes are on the routing context we can read), and measures TOWN-METRES = loop
 * distance running inside a dense box more than 2 km from the start (the start city exit is
 * unavoidable, so it is excluded). Run twice: steering OFF (-Dloop.denseboxwppenalty=0) vs ON
 * (default). Lower town-metres with steering on = fewer mid-loop city dips.
 */
public class CityDipMetricTest {

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {30000, 50000, 80000};
  private static final int[] RADII = {4800, 8000, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DL = {"N", "E", "S", "W"};

  private File segDir, gravel, out;

  @Test
  public void metric() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    out = new File(projectDir, "build/citydip"); out.mkdirs();

    double penalty = Double.parseDouble(System.getProperty("loop.denseboxwppenalty", "100"));
    System.out.println("=== city-dip metric (via penalty=" + penalty + ", steering "
      + (penalty > 0 ? "ON" : "OFF") + ") ===");
    int n = 0, dips = 0; double totalTownM = 0;
    for (LoopTestRegion region : REGIONS) {
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          double tm = townMeters(region.ilon, region.ilat, RADII[i], (int) DIRS[d]);
          if (tm < 0) continue;
          n++; totalTownM += tm; if (tm > 1000) dips++;
          System.out.println(String.format("  %s_%dkm_%s  townM(mid-loop)=%.0f%s",
            region.name().toLowerCase(), TARGETS[i] / 1000, DL[d], tm, tm > 1000 ? "  <-- CITY DIP" : ""));
        }
      }
    }
    System.out.println(String.format("\nAGG n=%d  meanTownM=%.0f  cityDips(>1km)=%d/%d", n, totalTownM / n, dips, n));
  }

  /** Loop metres inside a dense box, &gt;2 km from start (excludes the unavoidable start-city exit). -1 on failure. */
  private double townMeters(int cx, int cy, int radius, int dir) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.startDirection = dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripDenseResidential = true; // force boxes on this context (steering toggled by loop.denseboxwppenalty)
      File o = new File(out, "_c");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.size() < 3) return -1;
      double townM = 0;
      for (int i = 1; i < t.nodes.size(); i++) {
        OsmPathElement a = t.nodes.get(i - 1), b = t.nodes.get(i);
        int mlon = (a.getILon() + b.getILon()) >>> 1, mlat = (a.getILat() + b.getILat()) >>> 1;
        if (CheapRuler.distance(mlon, mlat, cx, cy) < 2000) continue; // skip start-city exit
        if (rc.isWithinDenseBox(mlon, mlat)) townM += a.calcDistance(b);
      }
      return townM;
    } catch (Exception e) { return -1; }
  }
}
