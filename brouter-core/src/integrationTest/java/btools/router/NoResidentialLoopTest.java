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
 * The narrow, direct fix for pointless residential dip-ins: route loops with a profile where
 * residential/living_street/service cost 4.5 (above through-roads, below tracks-stay-cheapest), so the
 * loop prefers the through-network and stops touring village interiors — WITHOUT the two-stage machinery.
 *
 * <p>A/B baseline (gravel) vs noresid (gravel-throughonly) on Basel+Freiburg. Writes the noresid loop as
 * {@code <key>__noresid.geojson} into the unified viewer's loops dir so it overlays the existing baseline.
 * Watches the failure modes: does it cut spurs (the dip-ins) while NOT degenerating (reuse/dist held)?
 */
public class NoResidentialLoopTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(40, TimeUnit.MINUTES).build();

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};
  private static final int[] TARGETS = {40000, 60000, 80000};
  private static final int[] RADII = {6400, 9550, 12700};
  private static final double[] DIRS = {0, 90, 180, 270};
  private static final String[] DIRLBL = {"N", "E", "S", "W"};

  private File segDir, loopsDir, gravel, through;

  @Test
  public void noResidential() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    through = new File(projectDir, "misc/profiles2/gravel-throughonly.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    Assume.assumeTrue("gravel-throughonly.brf missing", through.exists());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();

    System.out.println("=== residential-only penalty: baseline gravel vs gravel-throughonly ===");
    System.out.println("=== appeal = RoadCharacterScore (length-weighted highway desirability, profile-independent) ===");
    int n = 0, betterSpur = 0, worseReuse = 0, rcsHeld = 0, betterAppeal = 0, appealAndDist = 0;
    double dSpur = 0, dReuse = 0, dRcs = 0, dAppeal = 0, dResid = 0;
    for (LoopTestRegion region : REGIONS) {
      int cx = region.ilon, cy = region.ilat;
      for (int i = 0; i < TARGETS.length; i++) {
        for (int d = 0; d < DIRS.length; d++) {
          String key = region.name().toLowerCase() + "_" + (TARGETS[i] / 1000) + "km_" + DIRLBL[d];
          Res base = run(gravel, cx, cy, RADII[i], TARGETS[i], DIRS[d]);
          Res nr = run(through, cx, cy, RADII[i], TARGETS[i], DIRS[d]);
          if (base.ok && base.track != null) writeLoop(new File(loopsDir, key + "__baseline.geojson"), base, "baseline");
          if (nr.ok && nr.track != null) writeLoop(new File(loopsDir, key + "__noresid.geojson"), nr, "noresid");
          if (base.ok && nr.ok) {
            n++;
            dSpur += nr.spurs - base.spurs; dReuse += nr.reuse - base.reuse; dRcs += nr.rcs - base.rcs;
            dAppeal += nr.appeal - base.appeal; dResid += nr.residFrac - base.residFrac;
            if (nr.spurs < base.spurs) betterSpur++;
            if (nr.reuse > base.reuse + 1) worseReuse++;
            if (nr.rcs >= base.rcs - 0.01) rcsHeld++;
            boolean appealUp = nr.appeal > base.appeal + 0.005;
            boolean distOk = Math.abs(nr.distRatio - 1.0) <= 0.15;
            if (appealUp) betterAppeal++;
            if (appealUp && distOk) appealAndDist++;
          }
          System.out.println(String.format(Locale.US,
            "%s base appeal=%.2f resid=%.0f%% rcs=%.2f distR=%.2f | noresid appeal=%.2f resid=%.0f%% rcs=%.2f distR=%.2f",
            key, base.appeal, base.residFrac * 100, base.rcs, base.distRatio,
            nr.appeal, nr.residFrac * 100, nr.rcs, nr.distRatio));
        }
      }
    }
    System.out.println(String.format(Locale.US,
      "\nAGG n=%d  meanDAppeal=%+.3f meanDResid=%+.1f%% meanDRcs=%+.3f meanDReuse=%+.2f"
      + "\n  more-appealing %d/%d  appealing+distOK %d/%d  rcs-held %d/%d  degraded(reuse+>1) %d/%d",
      n, dAppeal / n, dResid / n * 100, dRcs / n, dReuse / n,
      betterAppeal, n, appealAndDist, n, rcsHeld, n, worseReuse, n));
    System.out.println("[noresid] wrote __baseline + __noresid loops into " + loopsDir.getAbsolutePath());
  }

  private static final class Res {
    boolean ok; double distKm, distRatio, reuse, rcs, costPerM, climbM; int selfX, spurs; OsmTrack track;
    double appeal, residFrac, trackFrac, artFrac;
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
      File o = new File(loopsDir, "_run");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
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
      r.rcs = RouteChoiceScore.score(t, target, "gravel", null, dir).qualityScore();
      r.costPerM = m.getAverageCostPerMeter();
      r.climbM = climb(t);
      RoadCharacterScore rc2 = RoadCharacterScore.compute(t, "gravel");
      r.appeal = rc2.appeal; r.residFrac = rc2.residentialFrac;
      r.trackFrac = rc2.trackFrac; r.artFrac = rc2.arterialFrac;
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
      .append("\"rcs\":").append(String.format(Locale.US, "%.2f", r.rcs)).append(",")
      .append("\"reuse\":").append(String.format(Locale.US, "%.1f", r.reuse)).append(",")
      .append("\"selfX\":").append(r.selfX).append(",\"spurs\":").append(r.spurs).append(",")
      .append("\"climbM\":").append(String.format(Locale.US, "%.0f", r.climbM)).append(",")
      .append("\"costPerM\":").append(String.format(Locale.US, "%.2f", r.costPerM))
      .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) {
      OsmPathElement nd = t.nodes.get(i);
      if (i > 0) sb.append(",");
      sb.append("[").append(String.format(Locale.US, "%.6f", (nd.getILon() - 180000000) / 1e6))
        .append(",").append(String.format(Locale.US, "%.6f", (nd.getILat() - 90000000) / 1e6)).append("]");
    }
    sb.append("]}}]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }
}
