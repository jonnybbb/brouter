package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Basel A/B for the urban-capsule loop prototype, paired with the
 * escape-countryside-gravel profile. Not a pass/fail oracle — a data generator:
 * for a small distance × direction matrix it runs GREEDY round-trips with
 * {@code roundTripCapsule} off (baseline) and on (capsule + elevation), computes
 * {@link LoopQualityMetrics} + {@link RouteChoiceScore}, prints a side-by-side
 * comparison, and writes one GeoJSON LineString per loop under
 * {@code build/capsule-ab/} for visual inspection.
 *
 * <p>Uses the local {@code segments4/} tiles and {@code misc/profiles2/}; skips if
 * the Basel tile or the escape profile is absent. Lenient gate (always returns the
 * shipped loop + metrics) so the A/B compares what each mode would actually ship.
 */
public class CapsuleBaselAbTest {

  @Rule
  public Timeout timeout = Timeout.builder()
    .withTimeout(20, TimeUnit.MINUTES).withLookingForStuckThread(true).build();

  private static final int BASE_ILON = 180000000 + (int) (7.590 * 1e6 + 0.5);
  private static final int BASE_ILAT = 90000000 + (int) (47.560 * 1e6 + 0.5);

  // radius ≈ targetDistance / (2*pi); same mapping the loop suite uses.
  // Small loops stay near the dense Basel start (where the capsule signal lives);
  // 80km is the countryside-dominated control.
  private static final int[] TARGETS = {25000, 40000, 80000};
  private static final int[] RADII = {4000, 6400, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};

  private static final class Row {
    String label;
    boolean ok;
    String err = "";
    double distKm, distRatio, reuse, dirDelta, costPerM, compactness, rcs, climbM;
    int closure, spurs, selfX, smallX, nodes;
    long ms;
  }

  @Test
  public void baselCapsuleAb() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    // Control = stock gravel; treatment-pairing = escape-countryside-gravel.
    String[] profileNames = {"gravel", "escape-countryside-gravel"};
    Assume.assumeTrue("Basel tile segments4/E5_N45.rd5 missing",
      new File(segDir, "E5_N45.rd5").isFile());

    File outDir = new File(projectDir, "build/capsule-ab");
    outDir.mkdirs();
    System.setProperty("loop.capsule.debug", "true"); // per-step classification counts

    List<String> csv = new ArrayList<>();
    csv.add("label,mode,ok,distKm,distRatio,reuse_pct,dirDelta,cost_per_m,compactness,closure_m,spurs,selfX,smallX,climb_m,rcs,ms");

    // Aggregate win counters (capsule better than baseline on each axis).
    int n = 0, betterReuse = 0, betterSelfX = 0, betterCost = 0, betterCompact = 0,
      betterRcs = 0, moreClimb = 0, betterSpurs = 0;

    System.out.println("\n==== Basel capsule A/B — GREEDY (gravel control + escape pairing) ====");
    System.out.println("(lower reuse/cost/m/selfX/spurs better; higher compactness/rcs/climb better)\n");

    for (String pf : profileNames) {
      File profile = new File(projectDir, "misc/profiles2/" + pf + ".brf");
      if (!profile.exists()) {
        System.out.println("[skip] profile missing: " + profile.getAbsolutePath());
        continue;
      }
      String pfTag = pf.replace("escape-countryside-", "esc-");
      System.out.println("\n-- profile: " + pf + " --");
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          String base = String.format(Locale.US, "basel_%s_%dkm_%s", pfTag, TARGETS[i] / 1000, DIRLBL[d]);
          Row off = run(base, "baseline", false, segDir, profile, RADII[i], TARGETS[i], DIRS[d], outDir);
          Row on = run(base, "capsule", true, segDir, profile, RADII[i], TARGETS[i], DIRS[d], outDir);
          printPair(off, on);
          csv.add(toCsv(off, "baseline"));
          csv.add(toCsv(on, "capsule"));
          if (off.ok && on.ok) {
            n++;
            if (on.reuse < off.reuse - 0.5) betterReuse++;
            if (on.selfX < off.selfX) betterSelfX++;
            if (on.costPerM < off.costPerM - 0.01) betterCost++;
            if (on.compactness > off.compactness + 0.01) betterCompact++;
            if (on.rcs > off.rcs + 0.005) betterRcs++;
            if (on.climbM > off.climbM + 20) moreClimb++;
            if (on.spurs < off.spurs) betterSpurs++;
          }
        }
      }
    }

    try (Writer w = new FileWriter(new File(outDir, "summary.csv"))) {
      for (String l : csv) {
        w.write(l);
        w.write("\n");
      }
    }

    System.out.println("\n---- aggregate over " + n + " comparable cases (capsule strictly better) ----");
    System.out.println("reuse↓ " + betterReuse + "  selfX↓ " + betterSelfX + "  cost/m↓ " + betterCost
      + "  compactness↑ " + betterCompact + "  RCS↑ " + betterRcs + "  climb↑ " + moreClimb
      + "  spurs↓ " + betterSpurs + "   (of " + n + ")");
    System.out.println("[capsule-ab] artifacts in " + outDir.getAbsolutePath());
  }

  private Row run(String base, String mode, boolean capsule, File segDir, File profile,
                  int radius, int target, double dir, File outDir) {
    Row r = new Row();
    r.label = base + "_" + mode;
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from";
      start.ilon = BASE_ILON;
      start.ilat = BASE_ILAT;
      wp.add(start);

      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false; // lenient: always return the shipped loop + metrics
      rc.roundTripCapsule = capsule;

      String outPath = new File(outDir, r.label).getAbsolutePath();
      RoutingEngine re = new RoutingEngine(outPath, outPath, segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);

      long t0 = System.nanoTime();
      re.doRun(0);
      r.ms = (System.nanoTime() - t0) / 1_000_000L;

      String err = re.getErrorMessage();
      OsmTrack track = re.getFoundTrack();
      if (track == null) track = re.getLastRejectedTrack();
      if (track == null || track.nodes == null || track.nodes.isEmpty()) {
        r.ok = false;
        r.err = err != null ? err : "no track";
        return r;
      }

      LoopQualityMetrics m = LoopQualityMetrics.compute(track, target, dir);
      r.ok = err == null;
      r.err = err == null ? "" : err;
      r.distKm = m.getActualDistanceMeters() / 1000.0;
      r.distRatio = m.getDistanceRatio();
      r.reuse = m.getRoadReusePercent();
      r.dirDelta = m.getDirectionDeltaDegrees();
      r.costPerM = m.getAverageCostPerMeter();
      r.compactness = m.getCompactnessScore();
      r.closure = m.getClosureDistanceMeters();
      r.spurs = m.getSpurCount();
      r.selfX = m.getSelfIntersections();
      r.smallX = m.getSmallLoopCrossings();
      r.rcs = RouteChoiceScore.score(track, target, "gravel", null, dir).qualityScore();
      r.nodes = track.nodes.size();
      r.climbM = totalClimb(track);

      writeGeoJson(new File(outDir, r.label + ".geojson"), track, r);
      return r;
    } catch (Exception e) {
      r.ok = false;
      r.err = String.valueOf(e);
      return r;
    }
  }

  /** Sum of positive elevation deltas along the track (meters). */
  private static double totalClimb(OsmTrack track) {
    double climb = 0;
    double prev = Double.NaN;
    for (OsmPathElement pe : track.nodes) {
      double e = pe.getElev();
      if (e < -1000 || e > 9000) { prev = Double.NaN; continue; } // skip sentinel/no-ele nodes
      if (!Double.isNaN(prev) && e > prev) climb += (e - prev);
      prev = e;
    }
    return climb;
  }

  private void printPair(Row off, Row on) {
    System.out.println(off.label.replace("_baseline", ""));
    System.out.println("  " + fmtRow(off));
    System.out.println("  " + fmtRow(on));
  }

  private String fmtRow(Row r) {
    if (!r.ok && r.distKm == 0) {
      return String.format(Locale.US, "%-9s ERROR/empty: %s",
        r.label.endsWith("capsule") ? "capsule" : "baseline", r.err);
    }
    return String.format(Locale.US,
      "%-9s dist=%.1fkm ratio=%.2f reuse=%.1f%% dirD=%.0f cost/m=%.2f compact=%.2f closure=%dm spurs=%d selfX=%d climb=%.0fm rcs=%.2f %dms%s",
      r.label.endsWith("capsule") ? "capsule" : "baseline",
      r.distKm, r.distRatio, r.reuse, r.dirDelta, r.costPerM, r.compactness, r.closure,
      r.spurs, r.selfX, r.climbM, r.rcs, r.ms, r.ok ? "" : " (gate-warned)");
  }

  private String toCsv(Row r, String mode) {
    return String.format(Locale.US, "%s,%s,%b,%.2f,%.3f,%.2f,%.0f,%.3f,%.3f,%d,%d,%d,%d,%.0f,%.3f,%d",
      r.label, mode, r.ok, r.distKm, r.distRatio, r.reuse, r.dirDelta, r.costPerM, r.compactness,
      r.closure, r.spurs, r.selfX, r.smallX, r.climbM, r.rcs, r.ms);
  }

  private void writeGeoJson(File f, OsmTrack track, Row r) throws Exception {
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{")
      .append("\"label\":\"").append(r.label).append("\",")
      .append("\"distKm\":").append(String.format(Locale.US, "%.1f", r.distKm)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", r.reuse)).append(",")
      .append("\"selfX\":").append(r.selfX).append(",")
      .append("\"climbM\":").append(String.format(Locale.US, "%.0f", r.climbM)).append(",")
      .append("\"costPerM\":").append(String.format(Locale.US, "%.2f", r.costPerM))
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < track.nodes.size(); i++) {
      OsmPathElement pe = track.nodes.get(i);
      double lon = (pe.getILon() - 180000000) / 1e6;
      double lat = (pe.getILat() - 90000000) / 1e6;
      if (i > 0) sb.append(",");
      sb.append("[").append(String.format(Locale.US, "%.6f", lon)).append(",")
        .append(String.format(Locale.US, "%.6f", lat)).append("]");
    }
    sb.append("]}}]}");
    try (Writer w = new FileWriter(f)) {
      w.write(sb.toString());
    }
  }
}
