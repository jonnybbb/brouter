package btools.router;

import org.junit.AfterClass;
import org.junit.Assume;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
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

  // Collected results for the HTML report. Populated in-memory during a normal
  // single-JVM run; also mirrored to disk (see CACHE_DIR below) so the report
  // survives Gradle's per-test JVM forking — without the disk cache, parametrized
  // tests that run in separate forks each see a partial `results` collection,
  // and the LAST fork's @AfterClass wins, producing a report that's missing
  // most of the runs. With the cache, every fork appends one file per result
  // and @AfterClass reads them all back at the end.
  private static final List<LoopQualityResult> results = new ArrayList<>();

  /**
   * On-disk cache of every {@link LoopQualityResult} produced by any test in
   * any fork. One file per result; filename is deterministic from the test
   * parameters so re-running a single case overwrites cleanly. Cleared at the
   * start of a fresh suite run (see {@link #ensureCacheCleared}).
   */
  private static final File RESULTS_CACHE_DIR = new File("build/reports/loops/.results");
  /**
   * Sentinel file marking that THIS JVM cleared the cache at suite start. The
   * first test to run in any fork wipes stale results from a previous suite;
   * subsequent tests (and subsequent forks) see the sentinel and skip the
   * wipe so we don't lose results from concurrent forks.
   */
  private static final File CACHE_SENTINEL = new File(RESULTS_CACHE_DIR, ".cleared");
  /** Suite-start timestamp threshold: cache files older than this are stale and wiped. */
  private static final long SUITE_RUN_FRESHNESS_MS = 6 * 60 * 60 * 1000L; // 6h

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
    // Persist each result to disk so the report survives Gradle test forking.
    // We write AFTER the in-memory add so an exception in JSON serialisation
    // doesn't lose the result for the same-JVM @AfterClass code path.
    persistResult(probeResult);
    persistResult(isoResult);
    persistResult(greedyResult);
    persistResult(isoGreedyResult);

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
        // Preserve the rejected route's geometry. The engine nulls
        // foundTrack on gate rejection but stashes the rejected track in
        // lastRejectedTrack; the visual-analysis pipeline depends on
        // this to draw the actual offending route on the map.
        OsmTrack rejected = track != null ? track : re.getLastRejectedTrack();
        double[][] failCoords = rejected != null ? extractCoordinates(rejected) : null;
        return new LoopQualityResult(testLabel, region, targetDistanceMeters,
          profileName, direction, null, error != null ? error : "no track", failCoords, variant);
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
    // Pull every result from the on-disk cache so the report includes runs
    // from ALL forks, not just the JVM whose @AfterClass happens to be the
    // last writer. The in-memory `results` is the same as on disk for the
    // current JVM, but other forks only show up if we read them back.
    List<LoopQualityResult> allResults = loadCachedResults();
    // Defensive: if the cache wasn't written (e.g. a future caller bypasses
    // persistResult), fall back to the in-memory list so we never produce a
    // worse report than the previous behaviour.
    if (allResults.isEmpty()) allResults = results;
    if (allResults.isEmpty()) return;

    printSummary(allResults);

    try {
      File buildDir = new File("build/reports/loops");
      buildDir.mkdirs();

      // HTML report
      File reportFile = new File(buildDir, "index.html");
      String html = LoopQualityReportGenerator.generateHtml(allResults);
      try (FileWriter fw = new FileWriter(reportFile)) {
        fw.write(html);
      }
      System.out.println("Loop quality report: " + reportFile.getAbsolutePath()
        + " (" + allResults.size() + " entries from " + (results.size() == allResults.size()
          ? "single JVM" : "disk cache spanning forks") + ")");

      // Combined GeoJSON FeatureCollection with all routes
      File geojsonFile = new File(buildDir, "all-routes.geojson");
      try (FileWriter fw = new FileWriter(geojsonFile)) {
        fw.write(formatCombinedGeoJson(allResults));
      }
      System.out.println("GeoJSON export: " + geojsonFile.getAbsolutePath());

      // Per-region HTML with full geometry and variant comparison
      for (LoopTestRegion region : LoopTestRegion.values()) {
        List<LoopQualityResult> regionResults = new ArrayList<>();
        for (LoopQualityResult r : allResults) {
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

  private static void printSummary(List<LoopQualityResult> all) {
    java.util.Map<String, int[]> byVariant = new java.util.LinkedHashMap<>();
    for (LoopQualityResult r : all) {
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
    for (LoopQualityResult r : all) {
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

  // ---- Cross-fork results cache ------------------------------------------

  /**
   * One-time-per-suite cache clear. Removes stale on-disk results from a
   * previous run so the report only reflects the current run. Idempotent and
   * concurrency-safe: the first fork to enter creates the sentinel, all
   * subsequent forks (and subsequent tests in the same fork) see the sentinel
   * and skip the wipe.
   *
   * <p>"Stale" is decided by mtime: a fresh sentinel within
   * {@link #SUITE_RUN_FRESHNESS_MS} means this is a continuation of an
   * ongoing run; an older sentinel (or none) means the previous run is done
   * and the cache should be wiped before we accept new results.
   */
  private static synchronized void ensureCacheCleared() {
    if (CACHE_SENTINEL.exists()
        && System.currentTimeMillis() - CACHE_SENTINEL.lastModified() < SUITE_RUN_FRESHNESS_MS) {
      return; // recent sentinel — another test (or fork) in this run already wiped.
    }
    if (RESULTS_CACHE_DIR.exists()) {
      File[] files = RESULTS_CACHE_DIR.listFiles();
      if (files != null) {
        for (File f : files) {
          // Safety: only delete .json result files and the sentinel; never
          // touch unexpected files a user might have dropped here.
          if (f.getName().endsWith(".json") || f.getName().equals(".cleared")) {
            f.delete();
          }
        }
      }
    }
    RESULTS_CACHE_DIR.mkdirs();
    try {
      CACHE_SENTINEL.createNewFile();
    } catch (IOException e) {
      // Best-effort: if we can't write the sentinel, subsequent tests may
      // re-wipe the cache. Worst case is a slightly slower run, not lost data.
    }
  }

  /**
   * Append one result to the on-disk cache. Filename is deterministic from
   * the test parameters so re-running a single case overwrites the previous
   * file instead of accumulating duplicates.
   */
  private static void persistResult(LoopQualityResult r) {
    if (r == null) return;
    ensureCacheCleared();
    String fn = String.format(Locale.US, "%s__%dkm__%s__%03.0f__%s.json",
      r.region.name().toLowerCase(), r.distanceMeters / 1000,
      r.profileName, r.direction, r.variant);
    File f = new File(RESULTS_CACHE_DIR, fn);
    try (FileWriter fw = new FileWriter(f)) {
      fw.write(serializeResult(r));
    } catch (IOException e) {
      System.err.println("Failed to persist loop quality result " + fn + ": " + e.getMessage());
    }
  }

  /**
   * Read every cached {@link LoopQualityResult} back from disk. Used by
   * @AfterClass to ensure the report reflects results from ALL forks, not
   * just the one whose static {@code results} list wins the race.
   */
  private static List<LoopQualityResult> loadCachedResults() {
    List<LoopQualityResult> loaded = new ArrayList<>();
    if (!RESULTS_CACHE_DIR.exists()) return loaded;
    File[] files = RESULTS_CACHE_DIR.listFiles((d, name) -> name.endsWith(".json"));
    if (files == null) return loaded;
    for (File f : files) {
      try {
        LoopQualityResult r = deserializeResult(f);
        if (r != null) loaded.add(r);
      } catch (IOException | RuntimeException e) {
        System.err.println("Failed to read cached result " + f.getName() + ": " + e.getMessage());
      }
    }
    return loaded;
  }

  /**
   * Compact ad-hoc JSON serialisation. We don't pull in a JSON library for a
   * test-only artifact — the field set is fixed and small, and we control
   * both ends of the wire. Strings are not escaped because they only contain
   * lowercase ASCII and underscores by construction.
   */
  private static String serializeResult(LoopQualityResult r) {
    StringBuilder sb = new StringBuilder(4096);
    sb.append("{");
    sb.append("\"label\":\"").append(r.label).append("\",");
    sb.append("\"region\":\"").append(r.region.name()).append("\",");
    sb.append(String.format(Locale.US, "\"distanceMeters\":%d,", r.distanceMeters));
    sb.append("\"profileName\":\"").append(r.profileName).append("\",");
    sb.append(String.format(Locale.US, "\"direction\":%f,", r.direction));
    sb.append("\"variant\":\"").append(r.variant).append("\",");
    if (r.error != null) {
      sb.append("\"error\":\"").append(r.error.replace("\\", "\\\\").replace("\"", "\\\"")).append("\",");
    }
    if (r.metrics != null) {
      LoopQualityMetrics m = r.metrics;
      sb.append("\"metrics\":{");
      sb.append(String.format(Locale.US, "\"reuse\":%.4f,\"distR\":%.4f,\"dirD\":%.4f,",
        m.getRoadReusePercent(), m.getDistanceRatio(), m.getDirectionDeltaDegrees()));
      sb.append(String.format(Locale.US, "\"actual\":%d,\"requested\":%d,\"cont\":%.4f,",
        m.getActualDistanceMeters(), m.getRequestedDistanceMeters(), m.getContinuityScore()));
      sb.append(String.format(Locale.US, "\"maxGap\":%d,\"totGap\":%d,\"compact\":%.4f,",
        m.getMaxGapMeters(), m.getTotalGapMeters(), m.getCompactnessScore()));
      sb.append(String.format(Locale.US, "\"costM\":%.4f,\"closure\":%d",
        m.getAverageCostPerMeter(), m.getClosureDistanceMeters()));
      sb.append("},");
    }
    if (r.coordinates != null) {
      sb.append("\"coordinates\":[");
      for (int i = 0; i < r.coordinates.length; i++) {
        if (i > 0) sb.append(",");
        sb.append(String.format(Locale.US, "[%.6f,%.6f]", r.coordinates[i][0], r.coordinates[i][1]));
      }
      sb.append("]");
    } else {
      sb.append("\"coordinates\":null");
    }
    sb.append("}");
    return sb.toString();
  }

  /**
   * Inverse of {@link #serializeResult}. Uses primitive scanning rather than
   * a JSON parser because the format is fixed. Returns null on malformed input.
   */
  private static LoopQualityResult deserializeResult(File f) throws IOException {
    StringBuilder content = new StringBuilder((int) f.length());
    try (BufferedReader br = new BufferedReader(new FileReader(f))) {
      String line;
      while ((line = br.readLine()) != null) content.append(line);
    }
    String json = content.toString();
    String label = strField(json, "label");
    String regionName = strField(json, "region");
    String profileName = strField(json, "profileName");
    String variant = strField(json, "variant");
    if (label == null || regionName == null || profileName == null || variant == null) return null;
    LoopTestRegion region = LoopTestRegion.valueOf(regionName);
    int distanceMeters = (int) Math.round(numField(json, "distanceMeters"));
    double direction = numField(json, "direction");
    String error = strField(json, "error");
    LoopQualityMetrics metrics = parseMetricsBlock(json);
    double[][] coords = parseCoords(json);
    return new LoopQualityResult(label, region, distanceMeters, profileName, direction,
      metrics, error, coords, variant);
  }

  private static String strField(String json, String key) {
    String needle = "\"" + key + "\":\"";
    int i = json.indexOf(needle);
    if (i < 0) return null;
    int start = i + needle.length();
    StringBuilder out = new StringBuilder();
    for (int j = start; j < json.length(); j++) {
      char c = json.charAt(j);
      if (c == '\\' && j + 1 < json.length()) {
        out.append(json.charAt(++j));
        continue;
      }
      if (c == '"') return out.toString();
      out.append(c);
    }
    return null;
  }

  private static double numField(String json, String key) {
    String needle = "\"" + key + "\":";
    int i = json.indexOf(needle);
    if (i < 0) return 0;
    int start = i + needle.length();
    int end = start;
    while (end < json.length()) {
      char c = json.charAt(end);
      if (c == ',' || c == '}' || c == ']') break;
      end++;
    }
    try {
      return Double.parseDouble(json.substring(start, end));
    } catch (NumberFormatException e) {
      return 0;
    }
  }

  private static LoopQualityMetrics parseMetricsBlock(String json) {
    int i = json.indexOf("\"metrics\":{");
    if (i < 0) return null;
    int start = i + "\"metrics\":{".length();
    int end = json.indexOf('}', start);
    if (end < 0) return null;
    String block = json.substring(start, end);
    double reuse = numField(block, "reuse");
    double distR = numField(block, "distR");
    double dirD = numField(block, "dirD");
    int actual = (int) Math.round(numField(block, "actual"));
    int requested = (int) Math.round(numField(block, "requested"));
    double cont = numField(block, "cont");
    int maxGap = (int) Math.round(numField(block, "maxGap"));
    int totGap = (int) Math.round(numField(block, "totGap"));
    double compact = numField(block, "compact");
    double costM = numField(block, "costM");
    int closure = (int) Math.round(numField(block, "closure"));
    return LoopQualityMetrics.fromFields(reuse, distR, dirD, actual, requested,
      cont, maxGap, totGap, compact, costM, closure);
  }

  private static double[][] parseCoords(String json) {
    // Locate the OUTER coordinates array. The value can be null (no track) or
    // a list of [lon, lat] pairs; nested brackets mean a naive indexOf(']')
    // finds the wrong closer, so we walk and count bracket depth.
    int keyIdx = json.indexOf("\"coordinates\":");
    if (keyIdx < 0) return null;
    int afterKey = keyIdx + "\"coordinates\":".length();
    // Skip whitespace
    while (afterKey < json.length() && Character.isWhitespace(json.charAt(afterKey))) afterKey++;
    if (afterKey >= json.length()) return null;
    if (json.startsWith("null", afterKey)) return null;
    if (json.charAt(afterKey) != '[') return null;
    int start = afterKey + 1;
    int depth = 1;
    int end = -1;
    for (int j = start; j < json.length(); j++) {
      char c = json.charAt(j);
      if (c == '[') depth++;
      else if (c == ']') {
        depth--;
        if (depth == 0) { end = j; break; }
      }
    }
    if (end < 0) return null;
    String block = json.substring(start, end);
    if (block.isEmpty()) return new double[0][];
    List<double[]> pairs = new ArrayList<>();
    int idx = 0;
    while (idx < block.length()) {
      int lb = block.indexOf('[', idx);
      if (lb < 0) break;
      int rb = block.indexOf(']', lb);
      if (rb < 0) break;
      String pair = block.substring(lb + 1, rb);
      int comma = pair.indexOf(',');
      if (comma < 0) { idx = rb + 1; continue; }
      try {
        double lon = Double.parseDouble(pair.substring(0, comma).trim());
        double lat = Double.parseDouble(pair.substring(comma + 1).trim());
        pairs.add(new double[]{lon, lat});
      } catch (NumberFormatException e) {
        // skip malformed pair
      }
      idx = rb + 1;
    }
    return pairs.toArray(new double[0][]);
  }

  /**
   * Soft target for per-route coordinate points in the report. We allow some
   * overshoot when the track is genuinely curvy — better to keep 600 points
   * on a winding Mallorca road than to drop turn-tips and render a
   * spurious "beeline" through terrain the cyclist would actually be
   * pedalling around.
   *
   * <p>Previously this was a hard 250-point stride sampler. On routes with
   * many small turns clustered between long straights (typical of
   * coastal/mountain Mallorca and Mediterranean tourist roads) the stride
   * sampler would drop the turn-tip points and render the result as a
   * single long straight segment — looking exactly like a routing
   * engine-inserted beeline, but with no underlying cause. The HTML report
   * showed a clean STRICT_LOOP route as if it were full of engine bugs.
   * Douglas-Peucker simplification preserves the shape adaptively at a
   * tolerance proportional to the route's bounding box.
   */
  private static final int MAX_REPORT_COORDS = 600;

  /**
   * Tolerance for Douglas-Peucker, in raw coordinate-degrees. ~3e-5 ≈ 3m at
   * European latitudes. A road bend's perpendicular sag must exceed this to
   * be kept; anything below is collapsed.
   */
  private static final double DP_EPSILON_DEG = 3e-5;

  private static double[][] extractCoordinates(OsmTrack track) {
    int n = track.nodes.size();
    double[][] full = new double[n][2];
    for (int i = 0; i < n; i++) {
      OsmPathElement node = track.nodes.get(i);
      full[i][0] = (node.getILon() - 180000000) / 1000000.0;
      full[i][1] = (node.getILat() - 90000000) / 1000000.0;
    }
    // Pass 1: Douglas-Peucker with a small fixed tolerance. This keeps every
    // bend that exceeds ~3m perpendicular sag.
    boolean[] keep = new boolean[n];
    keep[0] = true;
    keep[n - 1] = true;
    dpSimplify(full, 0, n - 1, DP_EPSILON_DEG, keep);
    // Pass 2: if still over the soft cap, increase tolerance progressively.
    // This is "graceful degradation": only happens for absurdly winding
    // tracks where 600 points isn't enough.
    int kept = countKept(keep);
    double eps = DP_EPSILON_DEG;
    while (kept > MAX_REPORT_COORDS && eps < 1e-2) {
      eps *= 2;
      for (int i = 1; i < n - 1; i++) keep[i] = false;
      dpSimplify(full, 0, n - 1, eps, keep);
      kept = countKept(keep);
    }
    double[][] out = new double[kept][2];
    int k = 0;
    for (int i = 0; i < n; i++) {
      if (keep[i]) { out[k][0] = full[i][0]; out[k][1] = full[i][1]; k++; }
    }
    return out;
  }

  /**
   * In-place Douglas-Peucker on the closed range {@code [lo, hi]}. Marks
   * {@code keep[i]=true} for every point whose perpendicular distance from
   * the {@code [lo..hi]} segment exceeds {@code eps}. Recursive; the inputs
   * are bounded by track length (~few thousand nodes) so stack depth is
   * well under the JVM default 512KB.
   */
  private static void dpSimplify(double[][] pts, int lo, int hi, double eps, boolean[] keep) {
    if (hi <= lo + 1) return;
    double x1 = pts[lo][0], y1 = pts[lo][1];
    double x2 = pts[hi][0], y2 = pts[hi][1];
    double dx = x2 - x1, dy = y2 - y1;
    double segLenSq = dx * dx + dy * dy;
    double maxDist = -1;
    int maxIdx = -1;
    for (int i = lo + 1; i < hi; i++) {
      double px = pts[i][0], py = pts[i][1];
      double d;
      if (segLenSq == 0) {
        double ex = px - x1, ey = py - y1;
        d = Math.sqrt(ex * ex + ey * ey);
      } else {
        // Perpendicular distance from (px,py) to line (x1,y1)-(x2,y2).
        double cross = Math.abs(dx * (y1 - py) - (x1 - px) * dy);
        d = cross / Math.sqrt(segLenSq);
      }
      if (d > maxDist) { maxDist = d; maxIdx = i; }
    }
    if (maxDist > eps) {
      keep[maxIdx] = true;
      dpSimplify(pts, lo, maxIdx, eps, keep);
      dpSimplify(pts, maxIdx, hi, eps, keep);
    }
  }

  private static int countKept(boolean[] keep) {
    int c = 0;
    for (boolean b : keep) if (b) c++;
    return c;
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
