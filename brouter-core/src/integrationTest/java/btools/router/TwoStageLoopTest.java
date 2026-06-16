package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import btools.util.CheapRuler;

/**
 * Two-stage loop prototype (the user's "simplified initial graph → candidates → final routing"):
 *
 * <p>Stage 1 — plan on a SIMPLIFIED graph: a round-trip with a residential-penalised profile
 * (through-network preferred, village interiors avoided), so the candidate loop has no pointless
 * residential dip-ins and no urban spaghetti.
 *
 * <p>Stage 2 — final routing: sample dense waypoints (~every {@code SAMPLE_M}) from the candidate
 * and route them as an explicit multi-via route on the REAL graph with the NORMAL profile, so the
 * final loop hugs the clean candidate corridor but gets full surface/geometry fidelity and proper
 * last-mile access to the actual start (the explicit-waypoint exception).
 *
 * <p>A/B vs the plain baseline round-trip. Writes baseline / stage-1 candidate / stage-2 final loops
 * + the extracted waypoints as GeoJSON keyed for the viewer.
 */
public class TwoStageLoopTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(40, TimeUnit.MINUTES).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {40000, 80000};
  private static final int[] RADII = {6400, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};
  private static final double SAMPLE_M = 2000;   // stage-2 waypoint spacing
  private static final String STAGE1_ESCAPE = "8"; // escape_strength override → residential ~x4.2

  private File segDir, outDir, gravel, escape;

  @Test
  public void twoStage() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    escape = new File(projectDir, "misc/profiles2/escape-countryside-gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    Assume.assumeTrue("gravel.brf missing", gravel.exists());
    Assume.assumeTrue("escape profile missing", escape.exists());
    outDir = new File(projectDir, "build/two-stage"); outDir.mkdirs();
    File loopsDir = new File(outDir, "loops"); loopsDir.mkdirs();

    System.out.println("=== two-stage (stage1 simplified/residential-penalised → vias → stage2 real graph) ===");
    List<String> csv = new ArrayList<>();
    csv.add("key,mode,ok,distKm,distRatio,reuse,selfX,spurs,rcs");
    for (LoopTestRegion region : REGIONS) {
      int cx = region.ilon, cy = region.ilat;
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          String key = region.name().toLowerCase() + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];

          // baseline: plain gravel round-trip
          Res base = roundTrip(gravel, null, cx, cy, RADII[i], TARGETS[i], DIRS[d]);

          // stage 1: simplified (residential-penalised) candidate
          HashMap<String, String> kv = new HashMap<>();
          kv.put("escape_strength", STAGE1_ESCAPE);
          Res cand = roundTrip(escape, kv, cx, cy, RADII[i], TARGETS[i], DIRS[d]);

          // stage 2: dense vias from the candidate, routed on the real graph with normal gravel
          Res fin = null;
          List<int[]> vias = cand.track != null ? sampleVias(cand.track, SAMPLE_M) : null;
          if (vias != null && vias.size() >= 3) fin = viaRoute(gravel, cx, cy, vias, TARGETS[i], DIRS[d]);

          csv.add(row(key, "baseline", base));
          csv.add(row(key, "stage1cand", cand));
          if (fin != null) csv.add(row(key, "twostage", fin));
          if (base.track != null) writeLoop(new File(loopsDir, key + "__baseline.geojson"), base, "baseline", null);
          if (cand.track != null) writeLoop(new File(loopsDir, key + "__stage1cand.geojson"), cand, "stage1cand", null);
          if (fin != null && fin.track != null) writeLoop(new File(loopsDir, key + "__twostage.geojson"), fin, "twostage", vias);

          System.out.println(String.format(Locale.US,
            "%s  base rcs=%.2f reuse=%.1f spurs=%d selfX=%d | stage1 rcs=%.2f spurs=%d | TWOSTAGE rcs=%.2f reuse=%.1f spurs=%d selfX=%d vias=%d",
            key, base.rcs, base.reuse, base.spurs, base.selfX, cand.rcs, cand.spurs,
            fin == null ? 0 : fin.rcs, fin == null ? 0 : fin.reuse, fin == null ? 0 : fin.spurs,
            fin == null ? 0 : fin.selfX, vias == null ? 0 : vias.size()));
        }
      }
    }
    try (Writer w = new FileWriter(new File(outDir, "summary.csv"))) {
      for (String l : csv) { w.write(l); w.write("\n"); }
    }
    System.out.println("[two-stage] " + outDir.getAbsolutePath());
  }

  private static final class Res {
    boolean ok; double distKm, distRatio, reuse, rcs; int selfX, spurs; OsmTrack track;
  }

  private Res roundTrip(File profile, HashMap<String, String> kv, int cx, int cy, int radius, int target, double dir) {
    Res r = new Res();
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.keyValues = kv;
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      File o = new File(outDir, "_rt");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      fill(r, re, target, dir);
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }

  /** Explicit multi-via route on the real graph: start -> vias... -> start (closed loop). */
  private Res viaRoute(File profile, int cx, int cy, List<int[]> vias, int target, double dir) {
    Res r = new Res();
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      for (int[] v : vias) { OsmNodeNamed n = new OsmNodeNamed(); n.ilon = v[0]; n.ilat = v[1]; n.name = "via"; wp.add(n); }
      OsmNodeNamed e = new OsmNodeNamed(); e.ilon = cx; e.ilat = cy; e.name = "to"; wp.add(e);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      File o = new File(outDir, "_via");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
      re.doRun(0);
      fill(r, re, target, dir);
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }

  private void fill(Res r, RoutingEngine re, int target, double dir) {
    String err = re.getErrorMessage();
    OsmTrack t = re.getFoundTrack();
    if (t == null) t = re.getLastRejectedTrack();
    if (t == null || t.nodes == null || t.nodes.isEmpty()) { r.ok = false; return; }
    r.track = t;
    LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
    r.ok = err == null;
    r.distKm = m.getActualDistanceMeters() / 1000.0;
    r.distRatio = m.getDistanceRatio();
    r.reuse = m.getRoadReusePercent();
    r.selfX = m.getSelfIntersections();
    r.spurs = m.getSpurCount();
    r.rcs = RouteChoiceScore.score(t, target, "gravel", null, dir).qualityScore();
  }

  /** Sample the candidate every ~everyM along its length → via coordinates (excludes start/end). */
  private static List<int[]> sampleVias(OsmTrack t, double everyM) {
    List<int[]> vias = new ArrayList<>();
    double acc = 0;
    OsmPathElement prev = null;
    for (OsmPathElement n : t.nodes) {
      if (prev != null) acc += CheapRuler.distance(prev.getILon(), prev.getILat(), n.getILon(), n.getILat());
      if (acc >= everyM) { vias.add(new int[]{n.getILon(), n.getILat()}); acc = 0; }
      prev = n;
    }
    return vias;
  }

  private String row(String key, String mode, Res r) {
    return String.format(Locale.US, "%s,%s,%b,%.2f,%.3f,%.2f,%d,%d,%.3f",
      key, mode, r.ok, r.distKm, r.distRatio, r.reuse, r.selfX, r.spurs, r.rcs);
  }

  private void writeLoop(File f, Res r, String approach, List<int[]> vias) throws Exception {
    OsmTrack t = r.track;
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"loop\",\"approach\":\"").append(approach).append("\",")
      .append("\"distKm\":").append(String.format(Locale.US, "%.1f", r.distKm)).append(",")
      .append("\"rcs\":").append(String.format(Locale.US, "%.2f", r.rcs)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", r.reuse)).append(",")
      .append("\"selfX\":").append(r.selfX).append(",\"spurs\":").append(r.spurs)
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) { OsmPathElement n = t.nodes.get(i); if (i > 0) sb.append(","); lonlat(sb, n.getILon(), n.getILat()); }
    sb.append("]}}");
    if (vias != null) for (int[] v : vias) {
      sb.append(",{\"type\":\"Feature\",\"properties\":{\"kind\":\"waypoint\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
      lonlat(sb, v[0], v[1]); sb.append("}}");
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private static void lonlat(StringBuilder sb, int ilon, int ilat) {
    sb.append("[").append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6))
      .append(",").append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]");
  }
}
