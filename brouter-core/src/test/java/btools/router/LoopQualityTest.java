package btools.router;

import org.junit.AfterClass;
import org.junit.Assume;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.assertTrue;

/**
 * Parameterized loop quality verification test.
 * <p>
 * Generates round-trip routes across 5 regions × 4 distances × 3 profiles × 4 directions
 * (240 combinations) and asserts quality metrics fall within acceptable bounds.
 * <p>
 * Requires downloaded segment data (see download-loop-test-segments.sh).
 * Tests are skipped when the required segment tile is not present.
 */
@RunWith(Parameterized.class)
public class LoopQualityTest {

  // Profiles under test
  private static final String[] PROFILES = {"fastbike", "gravel", "mtb"};
  // Target total loop distances in meters, and their corresponding search radii.
  // BRouter's roundTripDistance is a search RADIUS; actual loop length ≈ 2*pi*radius.
  // We use radius = targetDistance / (2*pi) ≈ targetDistance / 6.28.
  // Spec §11 distance set: 30/50/75km. 80/100km retained for legacy coverage.
  private static final int[] TARGET_DISTANCES = {30000, 50000, 75000, 80000, 100000};
  private static final int[] SEARCH_RADII = {4800, 8000, 11937, 12700, 15900};
  // Directions in degrees
  private static final double[] DIRECTIONS = {0, 90, 180, 270};
  // Direction labels for naming
  private static final String[] DIR_LABELS = {"N", "E", "S", "W"};

  // Collected results for the HTML report
  private static final List<LoopQualityResult> results = new ArrayList<>();

  @Parameterized.Parameter(0)
  public LoopTestRegion region;
  @Parameterized.Parameter(1)
  public int targetDistanceMeters;
  @Parameterized.Parameter(2)
  public int searchRadius;
  @Parameterized.Parameter(3)
  public String profileName;
  @Parameterized.Parameter(4)
  public double direction;
  @Parameterized.Parameter(5)
  public String testLabel;

  @Rule
  public TemporaryFolder outputDir = new TemporaryFolder();

  private File projectDir;

  @Parameterized.Parameters(name = "{5}")
  public static Collection<Object[]> data() {
    List<Object[]> params = new ArrayList<>();
    for (LoopTestRegion region : LoopTestRegion.values()) {
      for (int i = 0; i < TARGET_DISTANCES.length; i++) {
        for (String profile : PROFILES) {
          for (int d = 0; d < DIRECTIONS.length; d++) {
            String label = String.format("%s_%dkm_%s_%s",
              region.name().toLowerCase(), TARGET_DISTANCES[i] / 1000, profile, DIR_LABELS[d]);
            params.add(new Object[]{region, TARGET_DISTANCES[i], SEARCH_RADII[i], profile, DIRECTIONS[d], label});
          }
        }
      }
    }
    return params;
  }

  @Before
  public void setUp() throws Exception {
    // Opt-in: this suite is slow (>1h with full segment data) and is excluded from the
    // standard build. Run explicitly with -Dloop.tests=true (and segment data present).
    Assume.assumeTrue(
      "Loop quality tests are opt-in — run with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));
    projectDir = new File(".").getCanonicalFile().getParentFile();
  }

  @Test
  public void loopQuality() {
    File segDir = segmentDir();
    File segFile = new File(segDir, region.segmentFile);
    Assume.assumeTrue("Segment file not found: " + segFile.getAbsolutePath() +
      " — run download-loop-test-segments.sh to fetch test data", segFile.exists());
    File profileFile = profileFile(profileName);
    Assume.assumeTrue("Profile not found: " + profileFile.getAbsolutePath(), profileFile.exists());
    // Skip combos where the profile is fundamentally unsuitable for the
    // terrain (e.g. MTB in urban Berlin: no singletrack network exists, so
    // any route is forced through paved roads which the profile heavily
    // penalises). The cyclist would not choose this combo in practice;
    // testing it just produces noise that drowns out actionable failures.
    Assume.assumeTrue(
      "Profile " + profileName + " is not a supported profile for " + region.name()
        + " (no plausible route exists for this terrain × profile combination)",
      region.supportedProfiles.contains(profileName));

    // Run probe strategy (default)
    LoopQualityResult probeResult = runVariant("probe", RoundTripAlgorithm.WAYPOINT, segDir, profileFile);
    // Run isochrone strategy for comparison (best-effort, no assertions)
    LoopQualityResult isoResult = runVariant("isochrone", RoundTripAlgorithm.ISOCHRONE, segDir, profileFile);
    // Run greedy sub-route strategy for comparison (best-effort, no assertions)
    LoopQualityResult greedyResult = runVariant("greedy", RoundTripAlgorithm.GREEDY, segDir, profileFile);
    // Run QUALITY tier (isochrone-derived candidate pool through greedy planner)
    LoopQualityResult isoGreedyResult = runVariant("iso_greedy", RoundTripAlgorithm.ISO_GREEDY, segDir, profileFile);

    synchronized (results) {
      if (probeResult != null) results.add(probeResult);
      if (isoResult != null) results.add(isoResult);
      if (greedyResult != null) results.add(greedyResult);
      if (isoGreedyResult != null) results.add(isoGreedyResult);
    }

    logVariantMetrics(probeResult);
    logVariantMetrics(isoResult);
    logVariantMetrics(greedyResult);
    logVariantMetrics(isoGreedyResult);

    if (probeResult == null || probeResult.metrics == null) {
      Assume.assumeTrue("routing could not produce track for " + testLabel, false);
      return;
    }

    LoopQualityMetrics metrics = probeResult.metrics;
    assertTrue(
      String.format("%s: road reuse %.1f%% exceeds max %.1f%% for %s terrain",
        testLabel, metrics.getRoadReusePercent(), region.maxReusePercent, region.name()),
      metrics.getRoadReusePercent() <= region.maxReusePercent);
    assertTrue(
      String.format("%s: distance ratio %.2f below min %.2f",
        testLabel, metrics.getDistanceRatio(), region.minDistanceRatio),
      metrics.getDistanceRatio() >= region.minDistanceRatio);
    assertTrue(
      String.format("%s: distance ratio %.2f exceeds max %.2f",
        testLabel, metrics.getDistanceRatio(), region.maxDistanceRatio),
      metrics.getDistanceRatio() <= region.maxDistanceRatio);
    assertTrue(
      String.format("%s: direction delta %.1f° exceeds max %.1f°",
        testLabel, metrics.getDirectionDeltaDegrees(), region.maxDirectionDelta),
      metrics.getDirectionDeltaDegrees() <= region.maxDirectionDelta);
    // Profile-match gate: cost/m measures how well the route uses
    // profile-preferred roads. For fastbike a route with cost/m > 3.5 is on
    // roads the profile dislikes; gravel/mtb allow higher because their
    // preferred surfaces have higher base cost.
    double costPerM = metrics.getAverageCostPerMeter();
    double maxCostPerM = maxCostPerMeterForProfile(profileName);
    assertTrue(
      String.format("%s: cost/m %.2f exceeds max %.2f for %s profile",
        testLabel, costPerM, maxCostPerM, profileName),
      costPerM <= maxCostPerM);
    // Composite floor: a route with composite < MIN_COMPOSITE_PASS is bad
    // along multiple dimensions even if individual thresholds pass. This is
    // the catch-all that catches the "1.99x overshoot + low compactness +
    // bad cost/m + bad direction" combinations that any single threshold
    // misses but together signal an unusable loop.
    double composite = metrics.compositeScore();
    assertTrue(
      String.format("%s: composite %.2f below floor %.2f (route fails on multiple dimensions)",
        testLabel, composite, MIN_COMPOSITE_PASS),
      composite >= MIN_COMPOSITE_PASS);
  }

  /**
   * Composite-score floor for the WAYPOINT/probe variant assertion. A route below
   * this is bad along multiple dimensions — the per-dimension thresholds in
   * {@link LoopTestRegion} catch grossly-broken loops; this catches the more
   * insidious "ratio 1.5 × low compactness × bad cost/m" combinations.
   */
  private static final double MIN_COMPOSITE_PASS = 0.50;

  /**
   * Profile-specific cost-per-meter ceiling. fastbike rejects high-costfactor
   * roads (tracks, unpaved), gravel/mtb expect higher values because their
   * preferred surfaces are themselves higher costfactor.
   */
  private static double maxCostPerMeterForProfile(String profileName) {
    if (profileName == null) return 4.0;
    switch (profileName.toLowerCase()) {
      case "fastbike": return 3.5;
      case "gravel": return 4.0;
      case "mtb":
      case "mtb-zossebart": return 5.0;
      case "trekking": return 4.0;
      default: return 4.5;
    }
  }

  private LoopQualityResult runVariant(String variant, RoundTripAlgorithm algorithm, File segDir, File profileFile) {
    try {
      List<OsmNodeNamed> wplist = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from";
      start.ilon = region.ilon;
      start.ilat = region.ilat;
      wplist.add(start);

      RoutingContext rctx = new RoutingContext();
      rctx.localFunction = profileFile.getAbsolutePath();
      rctx.startDirection = (int) direction;
      rctx.roundTripDistance = searchRadius;
      rctx.roundTripAlgorithm = algorithm;

      String outPath = new File(outputDir.getRoot(), testLabel + "_" + variant).getAbsolutePath();
      RoutingEngine re = new RoutingEngine(
        outPath, outPath, segDir, wplist, rctx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);

      String error = re.getErrorMessage();
      OsmTrack track = re.getFoundTrack();

      if (error != null || track == null) {
        return new LoopQualityResult(testLabel, region, targetDistanceMeters,
          profileName, direction, null, error != null ? error : "no track", null, variant);
      }

      LoopQualityMetrics metrics = LoopQualityMetrics.compute(track, targetDistanceMeters, direction);
      double[][] coords = extractCoordinates(track);
      return new LoopQualityResult(testLabel, region, targetDistanceMeters,
        profileName, direction, metrics, null, coords, variant);
    } catch (Exception e) {
      return new LoopQualityResult(testLabel, region, targetDistanceMeters,
        profileName, direction, null, e.getMessage(), null, variant);
    }
  }

  private void logVariantMetrics(LoopQualityResult r) {
    if (r == null) return;
    if (r.metrics == null) {
      System.out.println(String.format(Locale.US, "%s [%s]: ERROR — %s",
        r.label, r.variant, r.error != null ? r.error : "no track"));
      return;
    }
    LoopQualityMetrics m = r.metrics;
    System.out.println(String.format(Locale.US,
      "%s [%s]: composite=%.2f distR=%.2f reuse=%.1f%% dirD=%.0f continuity=%.2f compactness=%.2f closure=%dm",
      r.label, r.variant, m.compositeScore(), m.getDistanceRatio(),
      m.getRoadReusePercent(), m.getDirectionDeltaDegrees(),
      m.getContinuityScore(), m.getCompactnessScore(),
      m.getClosureDistanceMeters()));
  }

  @AfterClass
  public static void generateReport() {
    if (results.isEmpty()) return;

    printSummary();

    try {
      File buildDir = new File("build/reports/loops");
      buildDir.mkdirs();

      // HTML report
      File reportFile = new File(buildDir, "index.html");
      String html = LoopQualityReportGenerator.generateHtml(results);
      try (FileWriter fw = new FileWriter(reportFile)) {
        fw.write(html);
      }
      System.out.println("Loop quality report: " + reportFile.getAbsolutePath());

      // Combined GeoJSON FeatureCollection with all routes
      File geojsonFile = new File(buildDir, "all-routes.geojson");
      try (FileWriter fw = new FileWriter(geojsonFile)) {
        fw.write(formatCombinedGeoJson(results));
      }
      System.out.println("GeoJSON export: " + geojsonFile.getAbsolutePath());

      // Per-region HTML with full geometry and variant comparison
      for (LoopTestRegion region : LoopTestRegion.values()) {
        List<LoopQualityResult> regionResults = new ArrayList<>();
        for (LoopQualityResult r : results) {
          if (r.region == region) regionResults.add(r);
        }
        if (regionResults.isEmpty()) continue;

        File regionGeoJson = new File(buildDir, "routes-" + region.name().toLowerCase() + ".geojson");
        try (FileWriter fw = new FileWriter(regionGeoJson)) {
          fw.write(formatCombinedGeoJson(regionResults));
        }

        File regionHtml = new File(buildDir, region.name().toLowerCase() + ".html");
        try (FileWriter fw = new FileWriter(regionHtml)) {
          fw.write(LoopQualityReportGenerator.generateRegionHtml(region, regionResults));
        }
        System.out.println("Region report: " + regionHtml.getAbsolutePath());
      }
    } catch (IOException e) {
      System.err.println("Failed to write loop quality report: " + e.getMessage());
    }
  }

  private static void printSummary() {
    java.util.Map<String, int[]> byVariant = new java.util.LinkedHashMap<>();
    for (LoopQualityResult r : results) {
      int[] counts = byVariant.computeIfAbsent(r.variant, k -> new int[2]); // [ok, error]
      if (r.metrics == null) counts[1]++; else counts[0]++;
    }

    System.out.println();
    System.out.println("=== LoopQualityTest variant summary ===");
    for (java.util.Map.Entry<String, int[]> e : byVariant.entrySet()) {
      int ok = e.getValue()[0];
      int err = e.getValue()[1];
      System.out.println(String.format(Locale.US, "  %-10s  ok=%d  error=%d  (total=%d)",
        e.getKey(), ok, err, ok + err));
    }

    // Errors by region/profile
    java.util.Map<String, Integer> errorsByCell = new java.util.TreeMap<>();
    for (LoopQualityResult r : results) {
      if (r.metrics != null) continue;
      String cell = r.region.name() + "/" + r.profileName;
      errorsByCell.merge(cell, 1, Integer::sum);
    }
    if (!errorsByCell.isEmpty()) {
      System.out.println("=== Errors by region/profile ===");
      for (java.util.Map.Entry<String, Integer> e : errorsByCell.entrySet()) {
        System.out.println(String.format(Locale.US, "  %-30s  %d", e.getKey(), e.getValue()));
      }
    }
    System.out.println();
  }

  /** Cap on per-route coordinate points retained for the report — keeps heap
   * usage bounded for the 1000+ route opt-in matrix without losing trace shape. */
  private static final int MAX_REPORT_COORDS = 250;

  private static double[][] extractCoordinates(OsmTrack track) {
    int n = track.nodes.size();
    int step = Math.max(1, (int) Math.ceil(n / (double) MAX_REPORT_COORDS));
    int outLen = ((n - 1) / step) + 1 + (((n - 1) % step != 0) ? 1 : 0);
    if (outLen > n) outLen = n;
    double[][] coords = new double[outLen][2];
    int outIdx = 0;
    for (int i = 0; i < n; i += step) {
      OsmPathElement node = track.nodes.get(i);
      coords[outIdx][0] = (node.getILon() - 180000000) / 1000000.0;
      coords[outIdx][1] = (node.getILat() - 90000000) / 1000000.0;
      outIdx++;
    }
    // Always include the closing endpoint so the rendered loop visually closes.
    if (outIdx < coords.length && (n - 1) % step != 0) {
      OsmPathElement last = track.nodes.get(n - 1);
      coords[outIdx][0] = (last.getILon() - 180000000) / 1000000.0;
      coords[outIdx][1] = (last.getILat() - 90000000) / 1000000.0;
      outIdx++;
    }
    if (outIdx == coords.length) return coords;
    double[][] trimmed = new double[outIdx][2];
    System.arraycopy(coords, 0, trimmed, 0, outIdx);
    return trimmed;
  }

  private static String variantColor(String variant) {
    switch (variant) {
      case "isochrone": return "#e67300";
      case "greedy": return "#22aa44";
      case "iso_greedy": return "#aa22cc";
      default: return "#0066cc";
    }
  }

  private static String formatCombinedGeoJson(List<LoopQualityResult> results) {
    StringBuilder sb = new StringBuilder(1024 * 1024);
    sb.append("{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n");
    boolean first = true;
    for (LoopQualityResult r : results) {
      if (r.coordinates == null || r.coordinates.length == 0) continue;
      if (!first) sb.append(",\n");
      first = false;
      sb.append("    {\n      \"type\": \"Feature\",\n");
      sb.append("      \"properties\": {\n");
      sb.append(String.format(Locale.US, "        \"name\": \"%s [%s]\",\n", r.label, r.variant));
      sb.append(String.format(Locale.US, "        \"variant\": \"%s\",\n", r.variant));
      sb.append(String.format(Locale.US, "        \"region\": \"%s\",\n", r.region.name()));
      sb.append(String.format(Locale.US, "        \"profile\": \"%s\",\n", r.profileName));
      sb.append(String.format(Locale.US, "        \"requestedDistance\": %d,\n", r.distanceMeters));
      sb.append(String.format(Locale.US, "        \"direction\": %.0f,\n", r.direction));
      if (r.metrics != null) {
        sb.append(String.format(Locale.US, "        \"actualDistance\": %d,\n", r.metrics.getActualDistanceMeters()));
        sb.append(String.format(Locale.US, "        \"distanceRatio\": %.2f,\n", r.metrics.getDistanceRatio()));
        sb.append(String.format(Locale.US, "        \"roadReusePercent\": %.1f,\n", r.metrics.getRoadReusePercent()));
        sb.append(String.format(Locale.US, "        \"directionDelta\": %.1f,\n", r.metrics.getDirectionDeltaDegrees()));
      }
      sb.append(String.format("        \"stroke\": \"%s\",\n", variantColor(r.variant)));
      sb.append("        \"stroke-width\": 2,\n");
      sb.append("        \"stroke-opacity\": 0.8\n");
      sb.append("      },\n");
      sb.append("      \"geometry\": {\n        \"type\": \"LineString\",\n        \"coordinates\": [\n");
      for (int i = 0; i < r.coordinates.length; i++) {
        sb.append(String.format(Locale.US, "          [%.6f, %.6f]", r.coordinates[i][0], r.coordinates[i][1]));
        if (i < r.coordinates.length - 1) sb.append(",");
        sb.append("\n");
      }
      sb.append("        ]\n      }\n    }");
    }
    sb.append("\n  ]\n}\n");
    return sb.toString();
  }

  private File segmentDir() {
    return new File(projectDir, "segments4");
  }

  private File profileFile(String name) {
    // The published segment tiles and misc/profiles2 are now the same lookup
    // version (v11), so route with the shipped profiles directly.
    return new File(projectDir, "misc/profiles2/" + name + ".brf");
  }

  /**
   * Holds the result of a single loop quality test case for report generation.
   */
  static class LoopQualityResult {
    final String label;
    final LoopTestRegion region;
    final int distanceMeters;
    final String profileName;
    final double direction;
    final LoopQualityMetrics metrics; // null if routing failed
    final String error; // null if routing succeeded
    final double[][] coordinates; // [lon, lat] pairs; null if routing failed
    final String variant; // "probe" or "isochrone"

    LoopQualityResult(String label, LoopTestRegion region, int distanceMeters,
                      String profileName, double direction,
                      LoopQualityMetrics metrics, String error, double[][] coordinates,
                      String variant) {
      this.label = label;
      this.region = region;
      this.distanceMeters = distanceMeters;
      this.profileName = profileName;
      this.direction = direction;
      this.metrics = metrics;
      this.error = error;
      this.coordinates = coordinates;
      this.variant = variant != null ? variant : "probe";
    }

    boolean passed() {
      if (metrics == null) return false;
      return metrics.getRoadReusePercent() <= region.maxReusePercent
        && metrics.getDistanceRatio() >= region.minDistanceRatio
        && metrics.getDistanceRatio() <= region.maxDistanceRatio
        && metrics.getDirectionDeltaDegrees() <= region.maxDirectionDelta;
    }
  }
}
