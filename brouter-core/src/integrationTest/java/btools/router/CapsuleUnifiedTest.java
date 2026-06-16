package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import btools.mapaccess.MatchedWaypoint;
import btools.util.CheapRuler;

/**
 * Unified, data-driven capsule comparison. Per (region, distance, direction) it runs FOUR approaches
 * from the same start — baseline, waypoint-steering, data-driven ring+cross contraction, and the
 * escape-countryside profile — at harmonized distances, and writes each loop + its waypoints + metrics
 * as GeoJSON keyed by case, so a single viewer can overlay all four on one map.
 *
 * <p>Capsules are detected DATA-DRIVEN across the whole searched area (not a hardcoded Basel circle):
 * the density grid is read from a probe round-trip, dense components are classified (road-density proxy
 * — the swap-point for a future OSM-landuse signal), the start component gets a real ring+cross
 * skeleton + soft-nogo quadrants, distant components (suburbs like Münchenstein/Riehen) get soft-nogo'd.
 */
public class CapsuleUnifiedTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(90, TimeUnit.MINUTES).build();

  private static final int CELL = RoutingEngine.DESIRABILITY_CELL;
  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};
  private static final double SOFT_W = 4.0;

  private File segDir, outDir, projectDir, gravel, escape;

  @Test
  public void unified() throws Exception {
    projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    escape = new File(projectDir, "misc/profiles2/escape-countryside-gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    Assume.assumeTrue("gravel.brf missing", gravel.exists());
    outDir = new File(projectDir, "build/capsule-unified");
    outDir.mkdirs();
    File loopsDir = new File(outDir, "loops"); loopsDir.mkdirs();

    for (LoopTestRegion region : REGIONS) {
      int cx = region.ilon, cy = region.ilat;
      String rg = region.name().toLowerCase();

      // 1. DATA-DRIVEN capsule detection across the searched area (from the widest grid).
      Map<Long, double[]> grid = probeGrid(cx, cy, 12700);
      Set<Long> dense = CapsuleNogoBuilder.closing(CapsuleNogoBuilder.classifyDense(grid, 0.80, 10));
      List<Set<Long>> comps = CapsuleNogoBuilder.components(dense, 4);
      long startCell = cellKey(cx, cy);
      Set<Long> startComp = null;
      for (Set<Long> c : comps) if (c.contains(startCell)) { startComp = c; break; }

      List<OsmNodeNamed> conNogos = new ArrayList<>();
      CapsuleSkeleton.Result skel = null;
      if (startComp != null) {
        int[] ctr = centroid(startComp);
        double rad = radius(startComp, ctr);
        skel = CapsuleSkeleton.build(ctr[0], ctr[1], rad, 8, (a, b, c2, d) -> p2p(gravel, a, b, c2, d));
        Set<Long> skelCells = skeletonCells(skel);
        skelCells.add(startCell);
        for (long nk : neighbors8(startCell)) skelCells.add(nk);
        Set<Long> mask = new HashSet<>(startComp);
        mask.removeAll(skelCells);
        conNogos.addAll(CapsuleNogoBuilder.polygonsFromCells(mask, CELL, SOFT_W, 2));
      }
      int distant = 0;
      for (Set<Long> c : comps) {
        if (c == startComp) continue;
        List<OsmNodeNamed> p = CapsuleNogoBuilder.polygonsFromCells(c, CELL, SOFT_W, 2);
        conNogos.addAll(p); distant += p.size();
      }
      System.out.println(rg + ": capsules=" + comps.size() + " (start " + (startComp == null ? "none" : startComp.size() + "cells")
        + ", distant=" + distant + ") conNogos=" + conNogos.size()
        + (skel == null ? "" : " skel portals=" + skel.portalCount() + " ring=" + skel.ring.size() + " cross=" + skel.cross.size()));
      writeSkeleton(new File(loopsDir, rg + "__skeleton.geojson"), skel, conNogos);

      // 2. Four approaches per (distance, direction).
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          String key = rg + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];
          run(loopsDir, key, "baseline", gravel, "gravel", cx, cy, RADII[i], TARGETS[i], DIRS[d], false, null);
          run(loopsDir, key, "waypoint", gravel, "gravel", cx, cy, RADII[i], TARGETS[i], DIRS[d], true, null);
          run(loopsDir, key, "contraction", gravel, "gravel", cx, cy, RADII[i], TARGETS[i], DIRS[d], false, conNogos);
          if (escape.exists())
            run(loopsDir, key, "escape", escape, "gravel", cx, cy, RADII[i], TARGETS[i], DIRS[d], false, null);
        }
      }
    }
    System.out.println("[capsule-unified] " + outDir.getAbsolutePath());
  }

  /** Run one approach, write its loop+waypoints+metrics geojson. */
  private void run(File dout, String key, String approach, File profile, String scoreProfile,
                   int cx, int cy, int radius, int target, double dir, boolean wpSteer, List<OsmNodeNamed> nogos) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      rc.startDirection = (int) dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripCapsule = wpSteer;
      System.setProperty("loop.capsule.nogoweight", "0"); // wp = candidate steering only, not leg-nogo
      if (nogos != null && !nogos.isEmpty()) rc.nogopoints = new ArrayList<>(nogos);
      File o = new File(dout, "_run");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      long t0 = System.nanoTime();
      re.doRun(0);
      long ms = (System.nanoTime() - t0) / 1_000_000L;
      int links = re.getLinksProcessed();
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.isEmpty()) return;
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      double rcs = RouteChoiceScore.score(t, target, scoreProfile, null, dir).qualityScore();
      writeLoop(new File(dout, key + "__" + approach + ".geojson"), t, approach, m, rcs, ms, links);
    } catch (Exception e) {
      System.out.println("  " + key + "/" + approach + " ERROR: " + e);
    }
  }

  private Map<Long, double[]> probeGrid(int cx, int cy, int radius) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripCapsule = true; // triggers the density-grid build
      System.setProperty("loop.capsule.nogoweight", "0");
      File o = new File(outDir, "_grid");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      return re.desirabilityGrid;
    } catch (Exception e) { return new java.util.HashMap<>(); }
  }

  private OsmTrack p2p(File profile, int fl, int fa, int tl, int ta) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed a = new OsmNodeNamed(); a.ilon = fl; a.ilat = fa; a.name = "from"; wp.add(a);
      OsmNodeNamed b = new OsmNodeNamed(); b.ilon = tl; b.ilat = ta; b.name = "to"; wp.add(b);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profile.getAbsolutePath();
      File o = new File(outDir, "_p2p");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
      re.doRun(0);
      return re.getFoundTrack();
    } catch (Exception e) { return null; }
  }

  // ---- geometry helpers -------------------------------------------------------
  private static long cellKey(int ilon, int ilat) { return (long) (ilon / CELL) * 1_000_000L + (ilat / CELL); }

  private static int[] cellCenter(long k) {
    long cx = k / 1_000_000L, cy = k % 1_000_000L;
    return new int[]{(int) (cx * CELL + CELL / 2), (int) (cy * CELL + CELL / 2)};
  }

  private static int[] centroid(Set<Long> comp) {
    double sx = 0, sy = 0;
    for (long k : comp) { int[] c = cellCenter(k); sx += c[0]; sy += c[1]; }
    return new int[]{(int) (sx / comp.size()), (int) (sy / comp.size())};
  }

  private static double radius(Set<Long> comp, int[] ctr) {
    double r = 0;
    for (long k : comp) { int[] c = cellCenter(k); r = Math.max(r, CheapRuler.distance(ctr[0], ctr[1], c[0], c[1])); }
    return r + CELL / 2.0;
  }

  private static long[] neighbors8(long k) {
    long cx = k / 1_000_000L, cy = k % 1_000_000L;
    List<Long> ns = new ArrayList<>();
    for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
      if (dx != 0 || dy != 0) ns.add((cx + dx) * 1_000_000L + (cy + dy));
    long[] a = new long[ns.size()];
    for (int i = 0; i < a.length; i++) a[i] = ns.get(i);
    return a;
  }

  private static Set<Long> skeletonCells(CapsuleSkeleton.Result skel) {
    Set<Long> cells = new HashSet<>();
    List<OsmTrack> all = new ArrayList<>(skel.ring); all.addAll(skel.cross);
    for (OsmTrack t : all) for (OsmPathElement nd : t.nodes) cells.add(cellKey(nd.getILon(), nd.getILat()));
    return cells;
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

  // ---- geojson ----------------------------------------------------------------
  private void writeLoop(File f, OsmTrack t, String approach, LoopQualityMetrics m, double rcs, long ms, int links) throws Exception {
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"loop\",\"approach\":\"").append(approach).append("\",")
      .append("\"distKm\":").append(String.format(Locale.US, "%.1f", m.getActualDistanceMeters() / 1000.0)).append(",")
      .append("\"rcs\":").append(String.format(Locale.US, "%.2f", rcs)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", m.getRoadReusePercent())).append(",")
      .append("\"selfX\":").append(m.getSelfIntersections()).append(",")
      .append("\"spurs\":").append(m.getSpurCount()).append(",")
      .append("\"climbM\":").append(String.format(Locale.US, "%.0f", totalClimb(t))).append(",")
      .append("\"costPerM\":").append(String.format(Locale.US, "%.2f", m.getAverageCostPerMeter())).append(",")
      .append("\"ms\":").append(ms).append(",\"links\":").append(links)
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) { OsmPathElement n = t.nodes.get(i); if (i > 0) sb.append(","); lonlat(sb, n.getILon(), n.getILat()); }
    sb.append("]}}");
    // waypoints (macro vias) if available
    if (t.matchedWaypoints != null) {
      for (MatchedWaypoint w : t.matchedWaypoints) {
        if (w.crosspoint == null) continue;
        sb.append(",{\"type\":\"Feature\",\"properties\":{\"kind\":\"waypoint\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
        lonlat(sb, w.crosspoint.getILon(), w.crosspoint.getILat());
        sb.append("}}");
      }
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private void writeSkeleton(File f, CapsuleSkeleton.Result skel, List<OsmNodeNamed> nogos) throws Exception {
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    boolean[] first = {true};
    if (skel != null) {
      for (OsmTrack t : skel.ring) line(sb, first, "ring", t);
      for (OsmTrack t : skel.cross) line(sb, first, "cross", t);
      for (int[] p : skel.portals) {
        if (p == null) continue;
        comma(sb, first);
        sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"portal\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
        lonlat(sb, p[0], p[1]); sb.append("}}");
      }
    }
    for (OsmNodeNamed ng : nogos) {
      if (!(ng instanceof OsmNogoPolygon)) continue;
      OsmNogoPolygon poly = (OsmNogoPolygon) ng;
      comma(sb, first);
      sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"nogo\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
      for (int i = 0; i < poly.points.size(); i++) { if (i > 0) sb.append(","); lonlat(sb, poly.points.get(i).x, poly.points.get(i).y); }
      if (!poly.points.isEmpty()) { sb.append(","); lonlat(sb, poly.points.get(0).x, poly.points.get(0).y); }
      sb.append("]}}");
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private static void line(StringBuilder sb, boolean[] first, String kind, OsmTrack t) {
    comma(sb, first);
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"").append(kind).append("\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) { OsmPathElement n = t.nodes.get(i); if (i > 0) sb.append(","); lonlat(sb, n.getILon(), n.getILat()); }
    sb.append("]}}");
  }

  private static void comma(StringBuilder sb, boolean[] first) { if (!first[0]) sb.append(","); first[0] = false; }

  private static void lonlat(StringBuilder sb, int ilon, int ilat) {
    sb.append("[").append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6))
      .append(",").append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]");
  }
}
