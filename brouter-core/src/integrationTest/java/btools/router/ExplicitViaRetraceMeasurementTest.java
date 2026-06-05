package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.Assume;
import org.junit.Test;

/**
 * Measurement (not a pass/fail test): quantifies how often TODAY's explicit-via
 * round-trip path (plain Dijkstra through user vias) produces a route that
 * <em>retraces</em> or <em>undershoots</em> the requested distance — the two
 * defects that a hypothetical "greedy-with-anchors" planner would address.
 *
 * <p>Purpose: decide, with data instead of intuition, whether greedy-with-vias
 * is worth building. If retrace/undershoot are rare on a representative via-loop
 * corpus, the feature is low value. If common, the numbers tell us whether a
 * surgical fix (retrace-aware closing leg + length padding) suffices or the full
 * planner change is warranted.
 *
 * <p>Opt-in (touches real {@code segments4/} tiles and routes ~dozens of loops):
 * <pre>./gradlew :brouter-core:integrationTest --tests '*ExplicitViaRetraceMeasurementTest' -Dvia.measure=true</pre>
 *
 * <p><b>Realistic vias.</b> Rather than scatter vias on a geometric circle (which
 * lands points in the sea, on ferries, or on the only off-road track and thus
 * measures placement artifacts), each case first generates a known-good AUTO loop
 * with no vias, then samples {@code N} vias evenly <em>along that loop</em>. Those
 * are on-road, reachable, loop-shaped points — what a real user picking spots on a
 * route would choose. The explicit-via path is then asked to reproduce a good
 * route through them. A failure here is therefore a genuine explicit-via
 * deficiency, not a placement artifact. Cases where AUTO itself cannot produce a
 * baseline loop are reported separately ({@code AUTO-UNAVAILABLE}) and excluded
 * from the deficiency stats.
 *
 * <p>Each routed result is classified with the production
 * {@link RoundTripQualityGate}: the canonical {@link RouteShape}, the gate's total
 * reuse ratio (the retrace metric), and the distance ratio vs {@code 2π·radius}.
 */
public class ExplicitViaRetraceMeasurementTest {

  /** Loop radii in metres → loop circumference ≈ 2π·r (~19 km and ~38 km). */
  private static final int[] RADII_M = {3000, 6000};
  /** Number of user vias to scatter around the loop. 1 = out-and-back. */
  private static final int[] VIA_COUNTS = {1, 2, 3};
  /** Base bearing the via ring is rotated to. */
  private static final int BASE_DIRECTION = 30;

  /** A route is "meaningfully retracing" above this share of reused distance. */
  private static final double RETRACE_RATIO_THRESHOLD = 0.10;
  /** A route "undershoots" when it is below this fraction of the requested loop. */
  private static final double UNDERSHOOT_RATIO = 0.85;

  private record Case(LoopTestRegion region, String profile, int radiusM, int viaCount) {}

  private record Outcome(Case c, boolean routed, String failure,
                         RouteShape shape, double reuseRatio, double distanceRatio,
                         int distanceM) {}

  /** Baseline (plain explicit-via) vs densified (SPIKE A) result for one case. */
  private record CaseResult(Case c, boolean baselineAvailable, Outcome base, Outcome dense) {}

  @Test
  public void measureExplicitViaRetraceAndUndershoot() throws Exception {

    // Opt-in only (see class javadoc / build.gradle): this is a heavy,
    // assertion-free measurement sweep over every region × profile × radius ×
    // via-count, so it must not run as part of a normal integrationTest pass.
    Assume.assumeTrue("opt-in: pass -Dvia.measure=true", Boolean.getBoolean("via.measure"));

    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/ not found at " + segDir.getAbsolutePath(), segDir.isDirectory());

    List<CaseResult> results = new ArrayList<>();

    for (LoopTestRegion region : LoopTestRegion.values()) {
      LoopTestSegments.ensureRegion(segDir, region);
      for (String profile : region.supportedProfiles) {
        File profileFile = new File(projectDir, "misc/profiles2/" + profile + ".brf");
        if (!profileFile.exists()) {
          continue;
        }
        for (int radius : RADII_M) {
          for (int viaCount : VIA_COUNTS) {
            results.add(runCase(new Case(region, profile, radius, viaCount), segDir, profileFile));
          }
        }
      }
    }

    report(results);
  }

  private CaseResult runCase(Case c, File segDir, File profileFile) {
    // 1. Generate a known-good AUTO loop (no vias) to source realistic on-road vias from.
    OsmTrack baseline = autoLoop(c, segDir, profileFile);
    if (baseline == null || baseline.nodes == null || baseline.nodes.size() < c.viaCount() + 2) {
      return new CaseResult(c, false, null, null);
    }

    // 2. Sample N vias evenly ALONG the baseline loop (on-road, loop-shaped). Store as raw
    //    coords because snapping mutates node positions — each route run builds fresh nodes.
    List<int[]> viaCoords = new ArrayList<>();
    int m = baseline.nodes.size();
    for (int i = 0; i < c.viaCount(); i++) {
      int idx = (int) Math.round((double) (i + 1) / (c.viaCount() + 1) * (m - 1));
      idx = Math.max(1, Math.min(m - 2, idx));
      OsmPathElement n = baseline.nodes.get(idx);
      viaCoords.add(new int[] {n.getILon(), n.getILat()});
    }

    // 3. Route the SAME vias two ways: plain explicit-via (baseline) vs SPIKE-A densified.
    Outcome base = routeVia(c, segDir, profileFile, viaCoords, false);
    Outcome dense = routeVia(c, segDir, profileFile, viaCoords, true);
    return new CaseResult(c, true, base, dense);
  }

  /** Route start -> vias -> start through the explicit-via path, optionally densified. */
  private Outcome routeVia(Case c, File segDir, File profileFile, List<int[]> viaCoords, boolean densify) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = c.region().ilon;
    start.ilat = c.region().ilat;
    wps.add(start);
    for (int k = 0; k < viaCoords.size(); k++) {
      OsmNodeNamed via = new OsmNodeNamed();
      via.name = "via" + (k + 1);
      via.ilon = viaCoords.get(k)[0];
      via.ilat = viaCoords.get(k)[1];
      wps.add(via);
    }

    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    rc.startDirection = BASE_DIRECTION;
    rc.roundTripDistance = c.radiusM();
    rc.turnInstructionMode = 0;
    rc.explicitViaDensifyOverride = densify;

    double expected = 2 * Math.PI * c.radiusM();
    try {
      RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);

      OsmTrack track = re.getFoundTrack();
      if (track == null || track.nodes == null || track.nodes.isEmpty()) {
        String msg = re.getErrorMessage() == null ? "no track" : re.getErrorMessage();
        return new Outcome(c, false, msg, null, 0, 0, 0);
      }

      RoundTripQualityResult q = RoundTripQualityGate.evaluate(
        track, expected, c.profile(), false, true, true);
      double ratio = expected > 0 ? track.distance / expected : 0;
      return new Outcome(c, true, null, q.getShape(), q.getTotalReuseRatio(), ratio, track.distance);
    } catch (Throwable t) {
      return new Outcome(c, false, t.getClass().getSimpleName() + ": " + t.getMessage(),
        null, 0, 0, 0);
    }
  }

  /**
   * Export one case (AUTO baseline + plain explicit-via + densified explicit-via) as GeoJSON
   * for visual inspection. Opt-in:
   * <pre>./gradlew :brouter-core:integrationTest --tests '*ExplicitViaRetraceMeasurementTest' \
   *   -Dvia.export=true [-Dvia.export.case=DREIEICH:gravel:6000:2]</pre>
   */
  @Test
  public void exportSampleRoute() throws Exception {
    Assume.assumeTrue("opt-in: pass -Dvia.export=true", Boolean.getBoolean("via.export"));

    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/ not found", segDir.isDirectory());

    String specs = System.getProperty("via.export.case", "");
    if (specs.isBlank()) {
      specs = "DREIEICH:gravel:6000:2";
    }
    File outDir = new File(projectDir, "brouter-core/build/via-export");
    outDir.mkdirs();

    for (String spec : specs.split(",")) {
      spec = spec.trim();
      if (spec.isEmpty()) {
        continue;
      }
      String[] p = spec.split(":");
      Case c = new Case(LoopTestRegion.valueOf(p[0]), p[1], Integer.parseInt(p[2]), Integer.parseInt(p[3]));
      LoopTestSegments.ensureRegion(segDir, c.region());
      File profileFile = new File(projectDir, "misc/profiles2/" + c.profile() + ".brf");

      OsmTrack baseline = autoLoop(c, segDir, profileFile);
      if (baseline == null || baseline.nodes == null || baseline.nodes.size() < c.viaCount() + 2) {
        System.out.println("AUTO baseline unavailable for " + spec + " — skipped");
        continue;
      }

      int m = baseline.nodes.size();
      List<int[]> viaCoords = new ArrayList<>();
      for (int i = 0; i < c.viaCount(); i++) {
        int idx = Math.max(1, Math.min(m - 2, (int) Math.round((double) (i + 1) / (c.viaCount() + 1) * (m - 1))));
        OsmPathElement n = baseline.nodes.get(idx);
        viaCoords.add(new int[] {n.getILon(), n.getILat()});
      }

      OsmTrack baseTrack = routeViaTrack(c, segDir, profileFile, viaCoords, false);
      OsmTrack denseTrack = routeViaTrack(c, segDir, profileFile, viaCoords, true);

      String tag = spec.replace(':', '_');
      writeGeoJson(new File(outDir, tag + "_0-auto-baseline.geojson"), baseline, profileFile);
      writeGeoJson(new File(outDir, tag + "_1-explicit-via-baseline.geojson"), baseTrack, profileFile);
      writeGeoJson(new File(outDir, tag + "_2-explicit-via-densified.geojson"), denseTrack, profileFile);
      System.out.println("Exported " + spec);
    }
    System.out.println("GeoJSON written to " + outDir.getAbsolutePath());
  }

  /** Route variant of {@link #routeVia} that returns the OsmTrack (null on failure). */
  private OsmTrack routeViaTrack(Case c, File segDir, File profileFile, List<int[]> viaCoords, boolean densify) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = c.region().ilon;
    start.ilat = c.region().ilat;
    wps.add(start);
    for (int k = 0; k < viaCoords.size(); k++) {
      OsmNodeNamed via = new OsmNodeNamed();
      via.name = "via" + (k + 1);
      via.ilon = viaCoords.get(k)[0];
      via.ilat = viaCoords.get(k)[1];
      wps.add(via);
    }
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    rc.startDirection = BASE_DIRECTION;
    rc.roundTripDistance = c.radiusM();
    rc.turnInstructionMode = 0;
    rc.explicitViaDensifyOverride = densify;
    try {
      RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);
      OsmTrack found = re.getFoundTrack();
      if (found != null) {
        return found;
      }
      // Gate-rejected: export the rejected geometry so the failure mode is visible.
      OsmTrack rejected = re.getLastRejectedTrack();
      if (rejected != null) {
        System.out.println("  (densify REJECTED: " + re.getErrorMessage() + ")");
      }
      return rejected;
    } catch (Throwable t) {
      return null;
    }
  }

  private void writeGeoJson(File file, OsmTrack track, File profileFile) throws Exception {
    if (track == null) {
      System.out.println("  (skipped " + file.getName() + " — no route)");
      return;
    }
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    String geojson = new FormatJson(rc).format(track);
    try (java.io.FileWriter w = new java.io.FileWriter(file)) {
      w.write(geojson);
    }
    System.out.printf("  wrote %s (%d nodes, %dm)%n", file.getName(),
      track.nodes == null ? 0 : track.nodes.size(), track.distance);
  }

  /** Generate a baseline AUTO round-trip loop (no vias) to sample realistic vias from. */
  private OsmTrack autoLoop(Case c, File segDir, File profileFile) {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    rc.startDirection = BASE_DIRECTION;
    rc.roundTripDistance = c.radiusM();
    rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    rc.turnInstructionMode = 0;

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = c.region().ilon;
    start.ilat = c.region().ilat;
    wps.add(start);

    try {
      RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);
      return re.getFoundTrack();
    } catch (Throwable t) {
      return null;
    }
  }

  private void report(List<CaseResult> results) {
    System.out.println();
    System.out.println("======== EXPLICIT-VIA: BASELINE vs SPIKE-A (via-arc densification) ========");
    System.out.printf("%-16s %-9s %6s %4s | %-13s %6s %7s | %-13s %6s %7s | %s%n",
      "region", "profile", "radius", "vias",
      "base shape", "reuse%", "d/req", "dense shape", "reuse%", "d/req", "Δd/req");
    System.out.println("---------------------------------------------------------------------------------------------");

    List<CaseResult> usable = new ArrayList<>();
    for (CaseResult r : results) {
      if (!r.baselineAvailable()) {
        System.out.printf("%-16s %-9s %5dm %4d | AUTO-UNAVAILABLE (no baseline loop)%n",
          r.c().region(), r.c().profile(), r.c().radiusM(), r.c().viaCount());
        continue;
      }
      String baseShape = r.base().routed() ? r.base().shape().toString() : "NO-ROUTE";
      String denseShape = r.dense().routed() ? r.dense().shape().toString() : "NO-ROUTE";
      String dDelta = (r.base().routed() && r.dense().routed())
        ? String.format("%+.2f", r.dense().distanceRatio() - r.base().distanceRatio()) : "-";
      System.out.printf("%-16s %-9s %5dm %4d | %-13s %5.0f%% %7.2f | %-13s %5.0f%% %7.2f | %s%n",
        r.c().region(), r.c().profile(), r.c().radiusM(), r.c().viaCount(),
        baseShape, r.base().reuseRatio() * 100, r.base().distanceRatio(),
        denseShape, r.dense().reuseRatio() * 100, r.dense().distanceRatio(), dDelta);
      if (r.base().routed() && r.dense().routed()) {
        usable.add(r);
      }
    }

    System.out.println("---------------------------------------------------------------------------------------------");
    long autoUnavail = results.stream().filter(rr -> !rr.baselineAvailable()).count();
    System.out.printf("cases: %d total | %d comparable (both routed) | %d AUTO-UNAVAILABLE%n",
      results.size(), usable.size(), autoUnavail);
    if (usable.isEmpty()) {
      System.out.println("No comparable cases.");
      return;
    }

    summarisePair("ALL", usable);
    for (int viaCount : VIA_COUNTS) {
      List<CaseResult> g = usable.stream().filter(rr -> rr.c().viaCount() == viaCount).toList();
      if (!g.isEmpty()) {
        summarisePair(viaCount + "-via", g);
      }
    }

    // Regression firewall: densification must not degrade clean cases (esp. dense urban).
    System.out.println();
    System.out.println("REGRESSION CHECK (densified made it worse):");
    int regr = 0;
    // (a) Lost a route entirely: base routed, dense did not. The most severe regression.
    for (CaseResult r : results) {
      if (r.baselineAvailable() && r.base().routed() && !r.dense().routed()) {
        regr++;
        System.out.printf("  %-16s %-9s %5dm %4dvia : [LOST-ROUTE] %s%n",
          r.c().region(), r.c().profile(), r.c().radiusM(), r.c().viaCount(),
          r.dense().failure() == null ? "" : r.dense().failure().replaceAll("\\s+", " ").trim());
      }
    }
    for (CaseResult r : usable) {
      boolean shorter = r.dense().distanceRatio() < r.base().distanceRatio() - 0.05;
      boolean moreReuse = r.dense().reuseRatio() > r.base().reuseRatio() + 0.05;
      boolean shapeDegraded = r.base().shape() == RouteShape.STRICT_LOOP
        && r.dense().shape() != RouteShape.STRICT_LOOP;
      if (shorter || moreReuse || shapeDegraded) {
        regr++;
        System.out.printf("  %-16s %-9s %5dm %4dvia : %s%s%s%n",
          r.c().region(), r.c().profile(), r.c().radiusM(), r.c().viaCount(),
          shorter ? "[shorter] " : "", moreReuse ? "[more-reuse] " : "", shapeDegraded ? "[shape↓]" : "");
      }
    }
    long baselineRouted = results.stream().filter(r -> r.baselineAvailable() && r.base().routed()).count();
    System.out.printf("  %d / %d baseline-routed cases regressed (incl. lost routes).%n", regr, baselineRouted);

    System.out.println();
    System.out.println("Vias sampled from a known-good AUTO loop. Success = densified median d/req up toward");
    System.out.println("~0.95 and reuse not worse, with ~zero regressions (degrade-to-Dijkstra guard).");
    System.out.println("==============================================================================================");
  }

  private void summarisePair(String label, List<CaseResult> g) {
    System.out.printf("  %-7s n=%-3d  median d/req: base %.2f -> dense %.2f   undershoot<%.2f: base %2.0f%% -> dense %2.0f%%   "
        + "retrace>%.0f%%: base %2.0f%% -> dense %2.0f%%%n",
      label, g.size(),
      median(g.stream().map(r -> r.base().distanceRatio()).sorted().toList()),
      median(g.stream().map(r -> r.dense().distanceRatio()).sorted().toList()),
      UNDERSHOOT_RATIO,
      pct(g.stream().filter(r -> r.base().distanceRatio() < UNDERSHOOT_RATIO).count(), g.size()),
      pct(g.stream().filter(r -> r.dense().distanceRatio() < UNDERSHOOT_RATIO).count(), g.size()),
      RETRACE_RATIO_THRESHOLD * 100,
      pct(g.stream().filter(r -> r.base().reuseRatio() > RETRACE_RATIO_THRESHOLD).count(), g.size()),
      pct(g.stream().filter(r -> r.dense().reuseRatio() > RETRACE_RATIO_THRESHOLD).count(), g.size()));
  }

  private static double pct(long k, int n) {
    return n == 0 ? 0 : 100.0 * k / n;
  }

  private static double median(List<Double> sorted) {
    if (sorted.isEmpty()) {
      return 0;
    }
    List<Double> s = new ArrayList<>(sorted);
    Collections.sort(s);
    int m = s.size() / 2;
    return s.size() % 2 == 1 ? s.get(m) : (s.get(m - 1) + s.get(m)) / 2.0;
  }
}
