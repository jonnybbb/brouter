package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/** SCRATCH (do not commit): does ISOCHRONE actually beat WAYPOINT and GREEDY on real v11 data? */
public class IsochroneVerificationStudy {

  private static class Agg {
    double ratioSum, compSum, reuseSum, gapSum, continuitySum, compactSum, costPerMSum;
    int closureSum, ok, fail;
    int wins; // number of (region,dir,distance) where this algo had the top composite
    long timeMsSum; // wall-time across ok cases
  }

  @Test
  public void compareAlgorithms() throws Exception {
    Assume.assumeTrue(
      "Scratch verification study — opt in with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profile = new File(projectDir, "misc/profiles2/fastbike.brf");
    Assume.assumeTrue("need profile", profile.exists());

    int[] distances = {30000, 50000}; // 30km (ISOCHRONE domain via AUTO), 50km (GREEDY domain)
    int[] dirs = {0, 90, 180, 270};
    RoundTripAlgorithm[] algos = {
      RoundTripAlgorithm.WAYPOINT, RoundTripAlgorithm.ISOCHRONE,
      RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY };

    Map<RoundTripAlgorithm, Agg> agg = new HashMap<>();
    for (RoundTripAlgorithm a : algos) agg.put(a, new Agg());

    System.out.println("STUDY: region dist dir | W=WAYPOINT  I=ISOCHRONE  G=GREEDY  Q=ISO_GREEDY (composite winner shown)");
    for (LoopTestRegion region : LoopTestRegion.values()) {
      File seg = new File(segDir, region.segmentFile);
      if (!seg.exists()) { System.out.println("  skip " + region + " (no tile)"); continue; }
      for (int targetL : distances) {
        int r = (int) Math.round(targetL / (2 * Math.PI));
        for (int dir : dirs) {
          double[] comp = new double[algos.length];
          double[] ratio = new double[algos.length];
          long[] elapsed = new long[algos.length];
          for (int ai = 0; ai < algos.length; ai++) {
            long t0 = System.currentTimeMillis();
            LoopQualityMetrics m = run(segDir, profile, region, dir, r, algos[ai], targetL);
            elapsed[ai] = System.currentTimeMillis() - t0;
            Agg ag = agg.get(algos[ai]);
            if (m == null) { ag.fail++; comp[ai] = -1; continue; }
            ag.ok++;
            ag.timeMsSum += elapsed[ai];
            ag.ratioSum += m.getDistanceRatio();
            ag.compSum += m.compositeScore();
            ag.reuseSum += m.getRoadReusePercent();
            ag.gapSum += m.getMaxGapMeters();
            ag.continuitySum += m.getContinuityScore();
            ag.compactSum += m.getCompactnessScore();
            ag.closureSum += m.getClosureDistanceMeters();
            ag.costPerMSum += m.getAverageCostPerMeter();
            comp[ai] = m.compositeScore();
            ratio[ai] = m.getDistanceRatio();
          }
          // award a "win" to the highest-composite ok algo
          int winIdx = -1; double bestComp = -1;
          for (int ai = 0; ai < algos.length; ai++) if (comp[ai] > bestComp) { bestComp = comp[ai]; winIdx = ai; }
          if (winIdx >= 0) agg.get(algos[winIdx]).wins++;
          System.out.println(String.format(Locale.US,
            "  %-16s %5dm dir=%-3d | W:c=%s I:c=%s G:c=%s Q:c=%s | W:r=%s I:r=%s G:r=%s Q:r=%s -> winner: %s",
            region.name().toLowerCase(), targetL, dir,
            fmt(comp[0]), fmt(comp[1]), fmt(comp[2]), fmt(comp[3]),
            fmt(ratio[0]), fmt(ratio[1]), fmt(ratio[2]), fmt(ratio[3]),
            winIdx < 0 ? "(all failed)" : algos[winIdx].name()));
        }
      }
    }
    System.out.println("\nPER-ALGORITHM AGGREGATES (avg over ok cases):");
    System.out.println("algo       ok fail wins | avgTime avgRatio avgComp avgReuse avgMaxGap avgCost/m");
    for (RoundTripAlgorithm a : algos) {
      Agg ag = agg.get(a);
      int n = Math.max(1, ag.ok);
      System.out.println(String.format(Locale.US,
        "%-10s %2d %4d %4d | %-6dms %.2f     %.2f    %.0f%%      %-5d    %.2f",
        a.name(), ag.ok, ag.fail, ag.wins,
        ag.timeMsSum / n, ag.ratioSum / n, ag.compSum / n, ag.reuseSum / n,
        (int) (ag.gapSum / n), ag.costPerMSum / n));
    }
  }

  private LoopQualityMetrics run(File segDir, File profile, LoopTestRegion region, int dir,
                                 int radius, RoundTripAlgorithm algo, int targetL) {
    try {
      List<OsmNodeNamed> wps = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.name = "from"; s.ilon = region.ilon; s.ilat = region.ilat; wps.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.startDirection = dir; rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = algo;
      RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true; re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null || t.nodes.size() < 2) return null;
      return LoopQualityMetrics.compute(t, targetL, dir);
    } catch (Exception e) { return null; }
  }

  private String fmt(double v) { return v < 0 ? " --- " : String.format(Locale.US, "%.2f", v); }
}
