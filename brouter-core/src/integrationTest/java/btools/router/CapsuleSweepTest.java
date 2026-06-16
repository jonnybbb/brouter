package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Comprehensive capsule comparison — Basel + Freiburg, gravel + fastbike, all 4 directions ×
 * {40,60,80}km, baseline (capsule off) vs the best capsule config (cap_p1_3: capsule weight 3,
 * elev 0, Phase-1). For every loop it records quality (RCS, reuse, crossings, spurs, compactness,
 * cost/m, climb), PERFORMANCE (wall-clock ms + links-processed), and writes a GeoJSON LineString
 * so the loops can be rendered and judged visually (rider preference, residual-area behavior).
 * Data generator, not a pass/fail oracle. Uses local segments4; skips if absent.
 *
 * <p>Outputs under build/capsule-cmp/: summary.csv + {profile}_{region}_{dist}km_{dir}_{mode}.geojson.
 */
public class CapsuleSweepTest {

  @Rule
  public Timeout timeout = Timeout.builder()
    .withTimeout(90, TimeUnit.MINUTES).withLookingForStuckThread(true).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final String[] PROFILES = {"gravel", "fastbike"};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};

  // FAITHFUL build under test: leg-masking via soft no-go capsules (capsule weight 0 =
  // no candidate steering; pure leg masking). nogoWeight = soft per-meter cost inside.
  private static final double CAP_W = 0.0, ELEV_W = 0.0, PHASE2 = 0.0, OVERSHOOT = 0.12;
  private static final double NOGO_W = 3.0;

  private static final class Res {
    boolean ok; String err = "";
    double distKm, distRatio, reuse, costPerM, compactness, rcs, climbM;
    int selfX, spurs;
    long ms; int links;
    OsmTrack track;
  }

  @Test
  public void compare() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    File outDir = new File(projectDir, "build/capsule-cmp");
    outDir.mkdirs();

    List<String> csv = new ArrayList<>();
    csv.add("profile,region,dist_km,dir,mode,ok,distKm,distRatio,reuse_pct,cost_per_m,compactness,"
      + "selfX,spurs,climb_m,rcs,ms,links");

    // per-profile accumulators: [n, rcsWin, rcsLoss, dRcs, dSelfX, dSpurs, dCost, dCompact, dMs, dLinks, dReuse]
    Map<String, double[]> agg = new LinkedHashMap<>();
    for (String p : PROFILES) agg.put(p, new double[11]);

    for (String profileName : PROFILES) {
      File profile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
      if (!profile.exists()) { System.out.println("[skip] " + profile); continue; }
      System.out.println("\n==== profile: " + profileName + " ====");
      for (LoopTestRegion region : REGIONS) {
        for (int i = 0; i < TARGETS.length; i++) {
          for (int d = 0; d < DIRS.length; d++) {
            String tag = profileName + "_" + region.name().toLowerCase()
              + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];
            Res base = run(false, profileName, segDir, profile, region, RADII[i], TARGETS[i], DIRS[d]);
            Res cap = run(true, profileName, segDir, profile, region, RADII[i], TARGETS[i], DIRS[d]);
            csv.add(row(profileName, region, TARGETS[i], DIRLBL[d], "baseline", base));
            csv.add(row(profileName, region, TARGETS[i], DIRLBL[d], "capsule", cap));
            if (base.ok && base.track != null) writeGeoJson(new File(outDir, tag + "_baseline.geojson"), base, tag + " baseline");
            if (cap.ok && cap.track != null) writeGeoJson(new File(outDir, tag + "_capsule.geojson"), cap, tag + " capsule");
            if (base.ok && cap.ok) {
              double[] g = agg.get(profileName);
              g[0]++;
              if (cap.rcs > base.rcs + 0.005) g[1]++;
              if (cap.rcs < base.rcs - 0.005) g[2]++;
              g[3] += cap.rcs - base.rcs;
              g[4] += cap.selfX - base.selfX;
              g[5] += cap.spurs - base.spurs;
              g[6] += cap.costPerM - base.costPerM;
              g[7] += cap.compactness - base.compactness;
              g[8] += cap.ms - base.ms;
              g[9] += cap.links - base.links;
              g[10] += cap.reuse - base.reuse;
            }
            System.out.println(String.format(Locale.US,
              "%s  base rcs=%.2f selfX=%d spurs=%d climb=%d %dms %dlnk | cap dRCS=%+.2f dSelfX=%+d dSpurs=%+d dClimb=%+d dMs=%+d dLnk=%+d",
              tag, base.rcs, base.selfX, base.spurs, (int) base.climbM, base.ms, base.links,
              cap.ok && base.ok ? cap.rcs - base.rcs : 0.0, cap.selfX - base.selfX, cap.spurs - base.spurs,
              (int) (cap.climbM - base.climbM), cap.ms - base.ms, cap.links - base.links));
          }
        }
      }
    }

    try (Writer w = new FileWriter(new File(outDir, "summary.csv"))) {
      for (String l : csv) { w.write(l); w.write("\n"); }
    }

    System.out.println("\n==== AGGREGATE: capsule (cap_p1_3) vs baseline ====");
    System.out.println("profile   n  RCSwin RCSloss  dRCS   dSelfX dSpurs dCost/m dCompact  dMs(mean) dLinks(mean) dReuse%");
    for (String p : PROFILES) {
      double[] g = agg.get(p);
      int n = (int) g[0];
      if (n == 0) continue;
      System.out.println(String.format(Locale.US,
        "%-8s %2d   %2d     %2d    %+.3f %+.2f  %+.2f  %+.3f  %+.3f   %+8.0f  %+11.0f  %+.2f",
        p, n, (int) g[1], (int) g[2], g[3] / n, g[4] / n, g[5] / n, g[6] / n, g[7] / n,
        g[8] / n, g[9] / n, g[10] / n));
    }
    System.out.println("(dRCS/dCompact higher=better; dSelfX/dSpurs/dCost/dReuse lower=better; "
      + "dMs/dLinks = capsule perf cost per loop: POSITIVE = SLOWER/more work)");
    System.out.println("[capsule-cmp] " + outDir.getAbsolutePath());
  }

  private Res run(boolean capsule, String profileName, File segDir, File profile, LoopTestRegion region,
                  int radius, int target, double dir) {
    Res res = new Res();
    try {
      if (capsule) {
        System.setProperty("loop.capsule.weight", String.valueOf(CAP_W));
        System.setProperty("loop.capsule.elevweight", String.valueOf(ELEV_W));
        System.setProperty("loop.capsule.phase2", String.valueOf(PHASE2));
        System.setProperty("loop.capsule.overshoottol", String.valueOf(OVERSHOOT));
        System.setProperty("loop.capsule.nogoweight", String.valueOf(NOGO_W));
      } else {
        System.setProperty("loop.capsule.nogoweight", "0");
      }
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from"; start.ilon = region.ilon; start.ilat = region.ilat;
      wp.add(start);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripCapsule = capsule;
      File out = new File(segDir.getParentFile(), "build/capsule-cmp/_run");
      RoutingEngine re = new RoutingEngine(out.getAbsolutePath(), out.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      long t0 = System.nanoTime();
      re.doRun(0);
      res.ms = (System.nanoTime() - t0) / 1_000_000L;
      res.links = re.getLinksProcessed();
      String err = re.getErrorMessage();
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.isEmpty()) { res.ok = false; res.err = err != null ? err : "no track"; return res; }
      res.track = t;
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      res.ok = err == null;
      res.distKm = m.getActualDistanceMeters() / 1000.0;
      res.distRatio = m.getDistanceRatio();
      res.reuse = m.getRoadReusePercent();
      res.costPerM = m.getAverageCostPerMeter();
      res.compactness = m.getCompactnessScore();
      res.selfX = m.getSelfIntersections();
      res.spurs = m.getSpurCount();
      res.rcs = RouteChoiceScore.score(t, target, profileName, null, dir).qualityScore();
      res.climbM = totalClimb(t);
      return res;
    } catch (Exception e) {
      res.ok = false; res.err = String.valueOf(e); return res;
    }
  }

  private static double totalClimb(OsmTrack t) {
    double climb = 0, prev = Double.NaN;
    for (OsmPathElement pe : t.nodes) {
      double e = pe.getElev();
      if (e < -1000 || e > 9000) { prev = Double.NaN; continue; }
      if (!Double.isNaN(prev) && e > prev) climb += (e - prev);
      prev = e;
    }
    return climb;
  }

  private String row(String profile, LoopTestRegion region, int target, String dir, String mode, Res r) {
    return String.format(Locale.US, "%s,%s,%d,%s,%s,%b,%.2f,%.3f,%.2f,%.3f,%.3f,%d,%d,%.0f,%.3f,%d,%d",
      profile, region.name().toLowerCase(), target / 1000, dir, mode, r.ok, r.distKm, r.distRatio,
      r.reuse, r.costPerM, r.compactness, r.selfX, r.spurs, r.climbM, r.rcs, r.ms, r.links);
  }

  private void writeGeoJson(File f, Res r, String label) throws Exception {
    OsmTrack t = r.track;
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{")
      .append("\"label\":\"").append(label).append("\",")
      .append("\"distKm\":").append(String.format(Locale.US, "%.1f", r.distKm)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", r.reuse)).append(",")
      .append("\"selfX\":").append(r.selfX).append(",")
      .append("\"spurs\":").append(r.spurs).append(",")
      .append("\"climbM\":").append(String.format(Locale.US, "%.0f", r.climbM)).append(",")
      .append("\"costPerM\":").append(String.format(Locale.US, "%.2f", r.costPerM)).append(",")
      .append("\"rcs\":").append(String.format(Locale.US, "%.2f", r.rcs))
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) {
      OsmPathElement pe = t.nodes.get(i);
      double lon = (pe.getILon() - 180000000) / 1e6;
      double lat = (pe.getILat() - 90000000) / 1e6;
      if (i > 0) sb.append(",");
      sb.append("[").append(String.format(Locale.US, "%.6f", lon)).append(",")
        .append(String.format(Locale.US, "%.6f", lat)).append("]");
    }
    sb.append("]}}]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }
}
