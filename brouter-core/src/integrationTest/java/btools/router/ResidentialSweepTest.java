package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Residential-cost sweep, judged by the validated {@link RoadCharacterScore} appeal metric (+ a
 * distance-OK guard). Compares baseline gravel against residential-penalty variants to pick the
 * cost that maximizes appeal WITHOUT the distance-control wobble that the crude all-4.5
 * gravel-throughonly profile showed. The cleaner variants (resid38/42/45) leave {@code service}
 * untouched (legit gravel connectors); throughonly is included to isolate the service-penalty effect.
 */
public class ResidentialSweepTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(60, TimeUnit.MINUTES).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};

  private static final String[] VARIANTS = {
    "gravel-resid38", "gravel-resid42", "gravel-resid45", "gravel-throughonly"};

  private File segDir, loopsDir;

  @Test
  public void sweep() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();
    File gravel = new File(projectDir, "misc/profiles2/gravel.brf");

    int nCases = REGIONS.length * TARGETS.length * DIRS.length;
    // baseline appeal per case (computed once)
    double[] baseAppeal = new double[nCases];
    System.out.println("=== residential-cost sweep (appeal = RoadCharacterScore, distOK = |distR-1|<=0.15) ===");

    // run baseline first
    int ci = 0;
    for (LoopTestRegion region : REGIONS)
      for (int i = 0; i < TARGETS.length; i++)
        for (double dir : DIRS) {
          Res b = run(gravel, region.ilon, region.ilat, RADII[i], TARGETS[i], dir);
          baseAppeal[ci++] = b.ok ? b.appeal : Double.NaN;
        }

    System.out.println(String.format(Locale.US, "%-22s %8s %8s %9s %9s %10s",
      "variant", "dAppeal", "resid%", "moreApp", "distOK", "appeal+OK"));
    for (String v : VARIANTS) {
      File prof = new File(projectDir, "misc/profiles2/" + v + ".brf");
      if (!prof.exists()) { System.out.println(v + ": MISSING"); continue; }
      int n = 0, moreApp = 0, distOk = 0, both = 0; double dApp = 0, residSum = 0;
      ci = 0;
      for (LoopTestRegion region : REGIONS)
        for (int i = 0; i < TARGETS.length; i++)
          for (double dir : DIRS) {
            Res r = run(prof, region.ilon, region.ilat, RADII[i], TARGETS[i], dir);
            double ba = baseAppeal[ci++];
            if (!r.ok || Double.isNaN(ba)) continue;
            n++;
            dApp += r.appeal - ba; residSum += r.residFrac;
            boolean appUp = r.appeal > ba + 0.005;
            boolean ok = Math.abs(r.distRatio - 1.0) <= 0.15;
            if (appUp) moreApp++;
            if (ok) distOk++;
            if (appUp && ok) both++;
          }
      System.out.println(String.format(Locale.US, "%-22s %+8.3f %7.0f%% %7d/%d %7d/%d %8d/%d",
        v, dApp / n, residSum / n * 100, moreApp, n, distOk, n, both, n));
    }
    System.out.println("[sweep] done");
  }

  private static final class Res {
    boolean ok; double distRatio, appeal, residFrac;
  }

  private Res run(File profile, int cx, int cy, int radius, int target, double dir) {
    Res r = new Res();
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      File o = new File(loopsDir, "_sweep");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.isEmpty()) { r.ok = false; return r; }
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      r.ok = re.getErrorMessage() == null;
      r.distRatio = m.getDistanceRatio();
      RoadCharacterScore rc2 = RoadCharacterScore.compute(t, "gravel");
      r.appeal = rc2.appeal; r.residFrac = rc2.residentialFrac;
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }
}
