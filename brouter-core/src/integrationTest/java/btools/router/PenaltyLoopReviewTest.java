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
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Final review of the residential-penalty WINNER, applied via the new {@code residential_penalty}
 * request parameter on the stock gravel.brf (proves the param-override path; no separate profile
 * file). For each Basel/Freiburg case it routes baseline (penalty 0) vs penalty=2.0, writes both as
 * {@code <key>__baseline/__penalty.geojson} for the viewer, and reports appeal + residential% +
 * distance + climb/km + the new RCS. The climb/km column gathers the data for the flat-terrain
 * question (is a typical loop actually flat, or already hilly?) before any elevation lever.
 */
public class PenaltyLoopReviewTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(40, TimeUnit.MINUTES).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};

  private File segDir, loopsDir, gravel;

  @Test
  public void review() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();
    double penalty = Double.parseDouble(System.getProperty("loop.residpenalty", "2.0"));

    System.out.println("=== penalty-loop review (residential_penalty=" + penalty + " via request param) ===");
    int n = 0, moreApp = 0, distOk = 0, both = 0, hillier = 0; double dApp = 0, dClimbKm = 0;
    for (LoopTestRegion region : REGIONS) {
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          String key = region.name().toLowerCase() + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];
          Res base = run(0.0, region.ilon, region.ilat, RADII[i], TARGETS[i], DIRS[d]);
          Res pen = run(penalty, region.ilon, region.ilat, RADII[i], TARGETS[i], DIRS[d]);
          if (base.ok) writeLoop(new File(loopsDir, key + "__baseline.geojson"), base, "baseline");
          if (pen.ok) writeLoop(new File(loopsDir, key + "__penalty.geojson"), pen, "penalty");
          if (base.ok && pen.ok) {
            n++;
            dApp += pen.appeal - base.appeal; dClimbKm += pen.climbPerKm - base.climbPerKm;
            boolean appUp = pen.appeal > base.appeal + 0.005;
            boolean ok = Math.abs(pen.distRatio - 1.0) <= 0.15;
            if (appUp) moreApp++;
            if (ok) distOk++;
            if (appUp && ok) both++;
            if (pen.climbPerKm > base.climbPerKm + 1) hillier++;
          }
          System.out.println(String.format(Locale.US,
            "%s base appeal=%.2f resid=%.0f%% distR=%.2f climb/km=%.0f | pen appeal=%.2f resid=%.0f%% distR=%.2f climb/km=%.0f rcs=%.2f",
            key, base.appeal, base.residFrac * 100, base.distRatio, base.climbPerKm,
            pen.appeal, pen.residFrac * 100, pen.distRatio, pen.climbPerKm, pen.rcs));
        }
      }
    }
    System.out.println(String.format(Locale.US,
      "\nAGG n=%d  meanDAppeal=%+.3f  more-appealing %d/%d  distOK %d/%d  appeal+distOK %d/%d  meanDClimb/km=%+.0f  hillier %d/%d",
      n, dApp / n, moreApp, n, distOk, n, both, n, dClimbKm / n, hillier, n));
    System.out.println("[review] wrote __baseline + __penalty loops into " + loopsDir.getAbsolutePath());
  }

  private static final class Res {
    boolean ok; double distKm, distRatio, reuse, rcs, appeal, residFrac, trackFrac, climbM, climbPerKm; int selfX, spurs; OsmTrack track;
    List<OsmNodeNamed> loopWaypoints;
  }

  private Res run(double penalty, int cx, int cy, int radius, int target, double dir) {
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
      if (penalty > 0) {
        Map<String, String> kv = new HashMap<>();
        kv.put("residential_penalty", Double.toString(penalty));
        rc.keyValues = kv;
        rc.roundTripDenseResidential = true; // density-box-scoped: penalty fires only inside dense areas
      }
      File o = new File(loopsDir, "_review");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.isEmpty()) { r.ok = false; return r; }
      r.track = t;
      RoundTripResult rtr = re.getLastRoundTripResult();
      r.loopWaypoints = (rtr != null) ? rtr.getLoopWaypoints() : null;
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      r.ok = re.getErrorMessage() == null;
      r.distKm = m.getActualDistanceMeters() / 1000.0;
      r.distRatio = m.getDistanceRatio();
      r.reuse = m.getRoadReusePercent();
      r.selfX = m.getSelfIntersections();
      r.spurs = m.getSpurCount();
      r.rcs = RouteChoiceScore.score(t, target, "gravel", null, dir).qualityScore();
      RoadCharacterScore rc2 = RoadCharacterScore.compute(t, "gravel");
      r.appeal = rc2.appeal; r.residFrac = rc2.residentialFrac; r.trackFrac = rc2.trackFrac;
      r.climbM = climb(t);
      r.climbPerKm = r.distKm > 0 ? r.climbM / r.distKm : 0;
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }

  private static double climb(OsmTrack t) {
    double c = 0, p = Double.NaN;
    for (OsmPathElement pe : t.nodes) { double e = pe.getElev(); if (e < -1000 || e > 9000) { p = Double.NaN; continue; } if (!Double.isNaN(p) && e > p) c += e - p; p = e; }
    return c;
  }

  private void writeLoop(File f, Res r, String approach) throws Exception {
    OsmTrack t = r.track;
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{\"kind\":\"loop\",\"approach\":\"").append(approach).append("\",")
      .append("\"distKm\":").append(String.format(Locale.US, "%.1f", r.distKm)).append(",")
      .append("\"appeal\":").append(String.format(Locale.US, "%.2f", r.appeal)).append(",")
      .append("\"residPct\":").append(String.format(Locale.US, "%.0f", r.residFrac * 100)).append(",")
      .append("\"trackPct\":").append(String.format(Locale.US, "%.0f", r.trackFrac * 100)).append(",")
      .append("\"climbM\":").append(String.format(Locale.US, "%.0f", r.climbM)).append(",")
      .append("\"climbPerKm\":").append(String.format(Locale.US, "%.0f", r.climbPerKm)).append(",")
      .append("\"rcs\":").append(String.format(Locale.US, "%.2f", r.rcs)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", r.reuse)).append(",")
      .append("\"selfX\":").append(r.selfX).append(",\"spurs\":").append(r.spurs)
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) {
      OsmPathElement nd = t.nodes.get(i);
      if (i > 0) sb.append(",");
      sb.append("[").append(String.format(Locale.US, "%.6f", (nd.getILon() - 180000000) / 1e6))
        .append(",").append(String.format(Locale.US, "%.6f", (nd.getILat() - 90000000) / 1e6)).append("]");
    }
    sb.append("]}}");
    // start + end markers (for a closed loop they coincide; shown distinctly anyway)
    OsmPathElement s0 = t.nodes.get(0), e0 = t.nodes.get(t.nodes.size() - 1);
    pt(sb, "start", s0.getILon(), s0.getILat());
    pt(sb, "end", e0.getILon(), e0.getILat());
    // planned loop waypoints (the vias the greedy planner placed)
    if (r.loopWaypoints != null) {
      for (OsmNodeNamed wp : r.loopWaypoints) pt(sb, "waypoint", wp.ilon, wp.ilat);
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private static void pt(StringBuilder sb, String kind, int ilon, int ilat) {
    sb.append(",{\"type\":\"Feature\",\"properties\":{\"kind\":\"").append(kind)
      .append("\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[")
      .append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6)).append(",")
      .append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]}}");
  }
}
