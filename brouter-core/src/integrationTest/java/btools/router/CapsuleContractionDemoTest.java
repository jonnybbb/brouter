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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import btools.util.CheapRuler;

/**
 * Working-graph CONTRACTION — full-matrix validation. For each region+profile, build the
 * start-capsule ring+cross skeleton ONCE ({@link CapsuleSkeleton}), carve its cells out of
 * the capsule disk, and SOFT-no-go the remaining interior quadrants (the tuned config:
 * keep the real ring+cross corridors routable, penalise the quadrant spaghetti). Then A/B
 * baseline vs contracted across Basel+Freiburg × gravel+fastbike × {40,60,80}km × 4 dirs,
 * measuring RCS + reuse + crossings + ms + links. The skeleton is per-start (city), so it
 * is amortised across all distances/directions — its build cost is excluded from per-loop ms,
 * and links (main-engine leg routing) is the fair node-count signal.
 */
public class CapsuleContractionDemoTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(60, TimeUnit.MINUTES).build();

  private static final int CELL = RoutingEngine.DESIRABILITY_CELL;
  // Messy-region test: Berlin (dense urban grid where the baseline makes spaghetti) with a larger
  // capsule to match its bigger dense core. The theory's home turf — the one place it should pay off.
  private static final double RADIUS_M = Double.parseDouble(System.getProperty("loop.capsule.radius", "4000"));
  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL};
  private static final String[] PROFILES = {"gravel", "fastbike"};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};

  private File segDir, outDir, projectDir;

  @Test
  public void contractionFullMatrix() throws Exception {
    projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    outDir = new File(projectDir, "build/capsule-contraction");
    outDir.mkdirs();
    double softW = Double.parseDouble(System.getProperty("loop.capsule.contractsoftw", "4.0"));

    List<String> csv = new ArrayList<>();
    csv.add("region,profile,dist_km,dir,mode,ok,distKm,distRatio,reuse,selfX,spurs,rcs,ms,links");
    Map<String, double[]> agg = new LinkedHashMap<>(); // profile -> [n,win,loss,dRcs,dMs,dLinks,dSelfX,dReuse]
    for (String p : PROFILES) agg.put(p, new double[8]);

    for (LoopTestRegion region : REGIONS) {
      int cx = region.ilon, cy = region.ilat;
      for (String profileName : PROFILES) {
        File profile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
        if (!profile.exists()) continue;

        // Build the start-capsule skeleton + soft-nogo quadrants ONCE per region+profile.
        CapsuleSkeleton.Router router = (fl, fa, tl, ta) -> p2p(profile, fl, fa, tl, ta);
        CapsuleSkeleton.Result skel = CapsuleSkeleton.build(cx, cy, RADIUS_M, 8, router);
        Set<Long> disk = diskCells(cx, cy, RADIUS_M);
        Set<Long> skeleton = skeletonCells(skel);
        skeleton.add(cellKey(cx, cy));
        for (long nk : neighbors8(cellKey(cx, cy))) skeleton.add(nk);
        Set<Long> mask = new HashSet<>(disk);
        mask.removeAll(skeleton);
        List<OsmNodeNamed> softNogos = CapsuleNogoBuilder.polygonsFromCells(mask, CELL, softW, 2);
        System.out.println(region.name() + "/" + profileName + ": portals=" + skel.portalCount()
          + " ring=" + skel.ring.size() + " cross=" + skel.cross.size()
          + " maskCells=" + mask.size() + " quadrants=" + softNogos.size());

        for (int i = 0; i < TARGETS.length; i++) {
          for (int d = 0; d < DIRS.length; d++) {
            Res base = roundTrip(profile, profileName, cx, cy, RADII[i], TARGETS[i], DIRS[d], null);
            Res con = roundTrip(profile, profileName, cx, cy, RADII[i], TARGETS[i], DIRS[d], softNogos);
            csv.add(rowCsv(region, profileName, TARGETS[i], DIRLBL[d], "baseline", base));
            csv.add(rowCsv(region, profileName, TARGETS[i], DIRLBL[d], "contracted", con));
            File loopsDir = new File(outDir, "loops"); loopsDir.mkdirs();
            String tag = region.name().toLowerCase() + "_" + profileName + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];
            if (base.ok) writeLoopGeoJson(new File(loopsDir, tag + "_baseline.geojson"), base, tag + " baseline");
            if (con.ok) writeLoopGeoJson(new File(loopsDir, tag + "_capsule.geojson"), con, tag + " contracted");
            if (base.ok && con.ok) {
              double[] g = agg.get(profileName);
              g[0]++;
              if (con.rcs > base.rcs + 0.005) g[1]++;
              if (con.rcs < base.rcs - 0.005) g[2]++;
              g[3] += con.rcs - base.rcs;
              g[4] += con.ms - base.ms;
              g[5] += con.links - base.links;
              g[6] += con.selfX - base.selfX;
              g[7] += con.reuse - base.reuse;
            }
            System.out.println(String.format(Locale.US,
              "%s/%s %dkm %s base rcs=%.2f selfX=%d %dlnk | con dRCS%+.2f dMs%+d dLnk%+d",
              region.name(), profileName, TARGETS[i] / 1000, DIRLBL[d], base.rcs, base.selfX, base.links,
              con.ok && base.ok ? con.rcs - base.rcs : 0, con.ms - base.ms, con.links - base.links));
          }
        }
      }
    }

    try (Writer w = new FileWriter(new File(outDir, "summary.csv"))) {
      for (String l : csv) { w.write(l); w.write("\n"); }
    }
    System.out.println("\n==== CONTRACTION AGGREGATE (soft+tight, vs baseline) ====");
    System.out.println("profile   n  win loss  dRCS    dMs(mean) dLinks(mean)  dSelfX dReuse");
    for (String p : PROFILES) {
      double[] g = agg.get(p);
      int n = (int) g[0];
      if (n == 0) continue;
      System.out.println(String.format(Locale.US, "%-8s %2d  %2d  %2d   %+.3f  %+8.0f  %+12.0f  %+.2f  %+.2f",
        p, n, (int) g[1], (int) g[2], g[3] / n, g[4] / n, g[5] / n, g[6] / n, g[7] / n));
    }
    System.out.println("(dRCS higher=better; dMs/dLinks/dSelfX/dReuse lower=better; dLinks<0 = fewer nodes)");
    System.out.println("[capsule-contraction] " + outDir.getAbsolutePath());
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

  private static final class Res {
    boolean ok; double distKm, distRatio, reuse, rcs, costPerM, climbM; int selfX, spurs; long ms; int links; OsmTrack track;
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

  private void writeLoopGeoJson(File f, Res r, String label) throws Exception {
    OsmTrack t = r.track;
    if (t == null) return;
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
      OsmPathElement n = t.nodes.get(i);
      if (i > 0) sb.append(",");
      sb.append("[").append(String.format(Locale.US, "%.6f", (n.getILon() - 180000000) / 1e6))
        .append(",").append(String.format(Locale.US, "%.6f", (n.getILat() - 90000000) / 1e6)).append("]");
    }
    sb.append("]}}]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private Res roundTrip(File profile, String profileName, int cx, int cy, int radius, int target,
                        double dir, List<OsmNodeNamed> nogos) {
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
      if (nogos != null && !nogos.isEmpty()) rc.nogopoints = new ArrayList<>(nogos);
      File o = new File(outDir, "_rt");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      long t0 = System.nanoTime();
      re.doRun(0);
      r.ms = (System.nanoTime() - t0) / 1_000_000L;
      r.links = re.getLinksProcessed();
      String err = re.getErrorMessage();
      OsmTrack t = re.getFoundTrack();
      if (t == null) t = re.getLastRejectedTrack();
      if (t == null || t.nodes == null || t.nodes.isEmpty()) { r.ok = false; return r; }
      r.track = t;
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, target, dir);
      r.ok = err == null;
      r.distKm = m.getActualDistanceMeters() / 1000.0;
      r.distRatio = m.getDistanceRatio();
      r.reuse = m.getRoadReusePercent();
      r.selfX = m.getSelfIntersections();
      r.spurs = m.getSpurCount();
      r.rcs = RouteChoiceScore.score(t, target, profileName, null, dir).qualityScore();
      r.costPerM = m.getAverageCostPerMeter();
      r.climbM = totalClimb(t);
      return r;
    } catch (Exception e) { r.ok = false; return r; }
  }

  private static long cellKey(int ilon, int ilat) { return (long) (ilon / CELL) * 1_000_000L + (ilat / CELL); }

  private static long[] neighbors8(long k) {
    long cx = k / 1_000_000L, cy = k % 1_000_000L;
    List<Long> ns = new ArrayList<>();
    for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
      if (dx != 0 || dy != 0) ns.add((cx + dx) * 1_000_000L + (cy + dy));
    long[] a = new long[ns.size()];
    for (int i = 0; i < a.length; i++) a[i] = ns.get(i);
    return a;
  }

  private static Set<Long> diskCells(int cx, int cy, double radiusM) {
    Set<Long> cells = new HashSet<>();
    int[] e = CheapRuler.destination(cx, cy, radiusM, 90), n = CheapRuler.destination(cx, cy, radiusM, 0);
    int dlon = Math.abs(e[0] - cx), dlat = Math.abs(n[1] - cy);
    for (int lon = cx - dlon; lon <= cx + dlon; lon += CELL / 2)
      for (int lat = cy - dlat; lat <= cy + dlat; lat += CELL / 2)
        if (CheapRuler.distance(cx, cy, lon, lat) <= radiusM) cells.add(cellKey(lon, lat));
    return cells;
  }

  private static Set<Long> skeletonCells(CapsuleSkeleton.Result skel) {
    Set<Long> cells = new HashSet<>();
    List<OsmTrack> all = new ArrayList<>(skel.ring);
    all.addAll(skel.cross);
    for (OsmTrack t : all)
      for (OsmPathElement nd : t.nodes) cells.add(cellKey(nd.getILon(), nd.getILat()));
    return cells;
  }

  private String rowCsv(LoopTestRegion region, String profile, int target, String dir, String mode, Res r) {
    return String.format(Locale.US, "%s,%s,%d,%s,%s,%b,%.2f,%.3f,%.2f,%d,%d,%.3f,%d,%d",
      region.name().toLowerCase(), profile, target / 1000, dir, mode, r.ok, r.distKm, r.distRatio,
      r.reuse, r.selfX, r.spurs, r.rcs, r.ms, r.links);
  }
}
