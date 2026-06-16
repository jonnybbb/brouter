package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Steep-segment penalty sweep (user directive: cost for grades &gt; 10%). Routes gravel loops with
 * {@code steep_penalty} ∈ {0,60,80,120} (request-param override on stock gravel.brf) in a mountainous
 * region (Grenoble) + a rolling one (Basel), and measures steep EXPOSURE — % of distance with grade
 * &gt; 10% over a ~60 m window (single-segment grades are too noisy) + the max sustained grade. Goal:
 * does the penalty cut steep exposure without wrecking target distance? Picks the value to default.
 */
public class SteepSweepTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(40, TimeUnit.MINUTES).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.GRENOBLE, LoopTestRegion.BASEL};
  private static final int[] TARGETS = {40000, 60000};
  private static final int[] RADII = {6400, 9550};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final double[] PENALTIES = {0, 60, 80, 120};
  private static final double WINDOW_M = 60.0;
  private static final double STEEP = 0.10;

  private File segDir, loopsDir, gravel;

  @Test
  public void sweep() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();

    System.out.println("=== steep-segment sweep (steepPct = % distance grade>10% over " + (int) WINDOW_M + "m window) ===");
    for (LoopTestRegion region : REGIONS) {
      System.out.println("--- " + region.name() + " ---");
      System.out.println(String.format(Locale.US, "%-9s %9s %9s %9s %9s %9s",
        "penalty", "steepPct", "maxGrade%", "climb/km", "distOKpct", "appeal"));
      for (double pen : PENALTIES) {
        int n = 0, distOk = 0; double steepSum = 0, maxGradeSum = 0, climbKmSum = 0, appealSum = 0;
        for (int i = 0; i < TARGETS.length; i++) {
          for (double dir : DIRS) {
            Res r = run(pen, region.ilon, region.ilat, RADII[i], TARGETS[i], dir);
            if (!r.ok) continue;
            n++;
            steepSum += r.steepPct; maxGradeSum += r.maxGrade; climbKmSum += r.climbPerKm; appealSum += r.appeal;
            if (Math.abs(r.distRatio - 1.0) <= 0.15) distOk++;
          }
        }
        if (n == 0) { System.out.println(String.format(Locale.US, "%-9.0f   (no loops)", pen)); continue; }
        System.out.println(String.format(Locale.US, "%-9.0f %8.1f%% %8.1f%% %9.0f %8.0f%% %9.2f",
          pen, steepSum / n * 100, maxGradeSum / n * 100, climbKmSum / n, 100.0 * distOk / n, appealSum / n));
      }
    }
    System.out.println("[steep] done");
  }

  private static final class Res {
    boolean ok; double distKm, distRatio, appeal, climbPerKm, steepPct, maxGrade;
  }

  private Res run(double steepPenalty, int cx, int cy, int radius, int target, double dir) {
    Res r = new Res();
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      Map<String, String> kv = new HashMap<>();
      kv.put("steep_penalty", Double.toString(steepPenalty)); // 0 = off (baseline), else the swept value
      rc.keyValues = kv;
      File o = new File(loopsDir, "_steep");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.size() < 3) { r.ok = false; return r; }
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      r.ok = re.getErrorMessage() == null;
      r.distKm = m.getActualDistanceMeters() / 1000.0;
      r.distRatio = m.getDistanceRatio();
      r.appeal = RoadCharacterScore.compute(t, "gravel").appeal;
      double[] ex = steepExposure(t, WINDOW_M);
      r.steepPct = ex[0]; r.maxGrade = ex[1]; r.climbPerKm = r.distKm > 0 ? ex[2] / r.distKm : 0;
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }

  /** [steepFractionOfDistance, maxAbsGrade, totalClimbMeters] over non-overlapping ~windowM windows. */
  private static double[] steepExposure(OsmTrack t, double windowM) {
    List<OsmPathElement> nodes = t.nodes;
    double total = 0, steep = 0, maxGrade = 0, climb = 0;
    double wDist = 0, wDH = 0, prevE = Double.NaN;
    for (int i = 1; i < nodes.size(); i++) {
      double e = nodes.get(i).getElev();
      boolean valid = e > -1000 && e < 9000;
      int segDist = nodes.get(i - 1).calcDistance(nodes.get(i));
      if (segDist <= 0) { if (valid) prevE = e; continue; }
      double dh = (valid && !Double.isNaN(prevE)) ? e - prevE : 0;
      if (valid) { if (dh > 0) climb += dh; prevE = e; }
      wDist += segDist; wDH += dh; total += segDist;
      if (wDist >= windowM) {
        double grade = Math.abs(wDH) / wDist;
        if (grade > maxGrade) maxGrade = grade;
        if (grade > STEEP) steep += wDist;
        wDist = 0; wDH = 0;
      }
    }
    if (wDist > 0) { double g = Math.abs(wDH) / wDist; if (g > STEEP) steep += wDist; if (g > maxGrade) maxGrade = g; }
    return new double[]{total > 0 ? steep / total : 0, maxGrade, climb};
  }
}
