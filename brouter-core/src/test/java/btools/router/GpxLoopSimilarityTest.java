package btools.router;

import org.junit.Assume;
import org.junit.Assert;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Aggregate real-GPX loop similarity benchmark.
 *
 * <p>This is deliberately separate from {@link btools.router.known.GpxReplayAggregateTest}.
 * GPX replay asks whether BRouter can follow cyclist-selected vias. This test
 * asks the harder planner question: starting from the same point, same length,
 * and same rough heading as a closed cyclist loop, does round-trip generation
 * produce a loop with similar shape and placement?
 *
 * <p>Run examples:
 * <pre>
 * ./gradlew :brouter-core:test -Dloop.tests=true -Dgpx.dir=tmp/gpx/Basel \
 *   -Dgpx.loop.limit=20 -Dloop.algorithms=greedy,iso_greedy,auto \
 *   --tests "btools.router.GpxLoopSimilarityTest"
 *
 * ./gradlew :brouter-core:test -Dloop.tests=true -Dgpx.dir=tmp/gpx/freiburg \
 *   -Dgpx.loop.limit=0 --tests "btools.router.GpxLoopSimilarityTest"
 * </pre>
 *
 * <p>The test is report-only by default. It writes markdown and JSON under
 * {@code docs/features/gpx-loop-similarity/}. Use the report to tune waypoint
 * placement; do not turn every current mismatch into a failing assertion.
 */
public class GpxLoopSimilarityTest {

  private static final Pattern TRKPT =
    Pattern.compile("<trkpt\\s+lat=\"([^\"]+)\"\\s+lon=\"([^\"]+)\"");
  private static final double EARTH_RADIUS = 6371000.0;
  private static final double[] RADIAL_FRACTIONS = {0.20, 0.40, 0.60, 0.80};

  @Test
  public void compareGeneratedLoopsToClosedGpxLoops() throws IOException {
    Assume.assumeTrue("loop-tests opt-in", Boolean.getBoolean("loop.tests"));

    String dirProp = System.getProperty("gpx.dir");
    Assume.assumeTrue("must pass -Dgpx.dir=<path>", dirProp != null);

    File segDir = segDir();
    Assume.assumeTrue("segments4 dir not found", segDir != null);

    Path root = resolveExistingPath(dirProp);
    Assume.assumeTrue("gpx.dir not found: " + dirProp, Files.exists(root));

    int limit = intProp("gpx.loop.limit", 20);
    int minKm = intProp("gpx.loop.minKm", 25);
    int maxKm = intProp("gpx.loop.maxKm", 130);
    int maxClosureMeters = intProp("gpx.loop.maxClosureMeters", 500);
    String forcedProfile = System.getProperty("gpx.loop.profile", "").trim();

    List<RoundTripAlgorithm> algorithms = parseAlgorithms(
      System.getProperty("loop.algorithms", "auto,greedy,iso_greedy"));

    List<Path> gpxFiles = new ArrayList<>();
    Files.walk(root)
      .filter(p -> p.toString().toLowerCase(Locale.ROOT).endsWith(".gpx"))
      .sorted()
      .forEach(gpxFiles::add);

    List<ReferenceLoop> references = new ArrayList<>();
    List<ScenarioResult> results = new ArrayList<>();
    for (Path gpx : gpxFiles) {
      ReferenceLoop ref = readReferenceLoop(gpx);
      if (ref == null) continue;
      int km = (int) Math.round(ref.stats.lengthMeters / 1000.0);
      if (km < minKm || km > maxKm) continue;
      if (ref.stats.closureMeters > maxClosureMeters) continue;

      File segFile = new File(segDir, segmentFileFor(ref.startLat(), ref.startLon()));
      if (!segFile.exists()) continue;

      ref.profileName = forcedProfile.isEmpty() ? inferProfile(gpx) : forcedProfile;
      File profileFile = profileFile(ref.profileName);
      if (profileFile == null) continue;

      references.add(ref);
      for (RoundTripAlgorithm algorithm : algorithms) {
        results.add(runGeneratedLoop(ref, algorithm, segDir, profileFile));
      }
      if (limit > 0 && references.size() >= limit) break;
    }

    Assume.assumeTrue("no closed GPX loops matched the configured filters", !references.isEmpty());
    writeReports(root, references, results, algorithms);
    if (Boolean.getBoolean("gpx.loop.assertAccepted")) {
      for (ScenarioResult result : results) {
        Assert.assertTrue(
          result.ref.path.getFileName() + " " + result.algorithm + " rejected: " + result.error,
          result.accepted());
      }
    }
  }

  private static ScenarioResult runGeneratedLoop(ReferenceLoop ref, RoundTripAlgorithm algorithm,
                                                 File segDir, File profileFile) {
    long t0 = System.currentTimeMillis();
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = ilon(ref.startLon());
    start.ilat = ilat(ref.startLat());
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.roundTripLength = (int) Math.round(ref.stats.lengthMeters);
    rctx.startDirection = (int) Math.round(ref.stats.farthestBearing);
    rctx.forceUseStartDirection = true;
    rctx.roundTripAlgorithm = algorithm;

    ScenarioResult result = new ScenarioResult(ref, algorithm);
    try {
      RoutingEngine re = new RoutingEngine(null, null, segDir, wplist, rctx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);

      result.runtimeMillis = System.currentTimeMillis() - t0;
      result.error = re.getErrorMessage();
      result.track = re.getFoundTrack();
      if (result.track == null && re.getLastRejectedTrack() != null) {
        result.track = re.getLastRejectedTrack();
      }
      if (result.track != null && result.track.nodes != null && !result.track.nodes.isEmpty()) {
        result.generated = RouteStats.fromTrack(result.track);
        result.distanceRatio = result.track.distance / ref.stats.lengthMeters;
        result.loopMetrics = LoopQualityMetrics.compute(result.track,
          (int) Math.round(ref.stats.lengthMeters), ref.stats.farthestBearing);
        result.gate = RoundTripQualityGate.evaluate(result.track, ref.stats.lengthMeters,
          ref.profileName, false, false, false);
        result.similarity = Similarity.compute(ref.stats, result.generated,
          result.distanceRatio, result.loopMetrics);
      }
    } catch (RuntimeException e) {
      result.runtimeMillis = System.currentTimeMillis() - t0;
      result.error = e.getClass().getSimpleName() + ": " + e.getMessage();
    }
    return result;
  }

  private static ReferenceLoop readReferenceLoop(Path gpx) throws IOException {
    String text = new String(Files.readAllBytes(gpx), StandardCharsets.UTF_8);
    Matcher m = TRKPT.matcher(text);
    List<Coord> coords = new ArrayList<>();
    while (m.find()) {
      coords.add(new Coord(Double.parseDouble(m.group(2)), Double.parseDouble(m.group(1))));
    }
    if (coords.size() < 10) return null;
    RouteStats stats = RouteStats.fromCoords(coords);
    return new ReferenceLoop(gpx, coords, stats);
  }

  private static List<RoundTripAlgorithm> parseAlgorithms(String prop) {
    List<RoundTripAlgorithm> algorithms = new ArrayList<>();
    for (String raw : prop.split(",")) {
      String s = raw.trim();
      if (s.isEmpty()) continue;
      RoundTripAlgorithm a = RoundTripAlgorithm.fromString(s);
      if (!algorithms.contains(a)) algorithms.add(a);
    }
    if (algorithms.isEmpty()) {
      algorithms.add(RoundTripAlgorithm.AUTO);
      algorithms.add(RoundTripAlgorithm.GREEDY);
      algorithms.add(RoundTripAlgorithm.ISO_GREEDY);
    }
    return algorithms;
  }

  private static String inferProfile(Path gpx) {
    String lower = gpx.toString().toLowerCase(Locale.ROOT);
    if (lower.contains("gravel") || lower.contains("grvl")) return "gravel";
    if (lower.contains("mtb")) return "mtb";
    return "fastbike";
  }

  private static File profileFile(String name) {
    File p = new File("misc/profiles2/" + name + ".brf");
    if (!p.exists()) p = new File("../misc/profiles2/" + name + ".brf");
    return p.exists() ? p : null;
  }

  private static File segDir() {
    File f = new File("../segments4");
    if (!f.exists()) f = new File("segments4");
    return f.exists() ? f : null;
  }

  private static Path resolveExistingPath(String p) {
    Path root = Paths.get(p);
    if (Files.exists(root)) return root;
    Path fromProjectRoot = Paths.get("..").resolve(p).normalize();
    return Files.exists(fromProjectRoot) ? fromProjectRoot : root;
  }

  private static int intProp(String key, int fallback) {
    String v = System.getProperty(key);
    if (v == null || v.trim().isEmpty()) return fallback;
    try {
      return Integer.parseInt(v.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  private static String segmentFileFor(double lat, double lon) {
    int tileLon = (int) Math.floor(lon / 5) * 5;
    int tileLat = (int) Math.floor(lat / 5) * 5;
    return "E" + tileLon + "_N" + tileLat + ".rd5";
  }

  private static int ilon(double lon) {
    return (int) Math.round((lon + 180.0) * 1_000_000.0);
  }

  private static int ilat(double lat) {
    return (int) Math.round((lat + 90.0) * 1_000_000.0);
  }

  private static double haversine(Coord a, Coord b) {
    double lat1 = Math.toRadians(a.lat);
    double lat2 = Math.toRadians(b.lat);
    double dLat = Math.toRadians(b.lat - a.lat);
    double dLon = Math.toRadians(b.lon - a.lon);
    double s = Math.sin(dLat / 2) * Math.sin(dLat / 2)
      + Math.cos(lat1) * Math.cos(lat2)
      * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return 2 * EARTH_RADIUS * Math.asin(Math.sqrt(s));
  }

  private static double bearing(Coord a, Coord b) {
    double lat1 = Math.toRadians(a.lat);
    double lat2 = Math.toRadians(b.lat);
    double dLon = Math.toRadians(b.lon - a.lon);
    double y = Math.sin(dLon) * Math.cos(lat2);
    double x = Math.cos(lat1) * Math.sin(lat2)
      - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    double deg = Math.toDegrees(Math.atan2(y, x));
    return (deg + 360.0) % 360.0;
  }

  private static double angleDiff(double a, double b) {
    double d = Math.abs(a - b) % 360.0;
    return d > 180.0 ? 360.0 - d : d;
  }

  private static double clamp01(double v) {
    return Math.max(0.0, Math.min(1.0, v));
  }

  private static Coord interpolate(List<Coord> coords, double[] cumulative, double fraction) {
    double target = cumulative[cumulative.length - 1] * fraction;
    int i = 1;
    while (i < cumulative.length && cumulative[i] < target) i++;
    if (i >= cumulative.length) return coords.get(coords.size() - 1);
    double span = cumulative[i] - cumulative[i - 1];
    double f = span <= 0 ? 0 : (target - cumulative[i - 1]) / span;
    Coord a = coords.get(i - 1);
    Coord b = coords.get(i);
    return new Coord(a.lon + (b.lon - a.lon) * f, a.lat + (b.lat - a.lat) * f);
  }

  private static void writeReports(Path root, List<ReferenceLoop> refs,
                                   List<ScenarioResult> results,
                                   List<RoundTripAlgorithm> algorithms) throws IOException {
    Path outDir = reportDir();
    Files.createDirectories(outDir);
    String stem = root.getFileName() == null ? "gpx" : root.getFileName().toString();
    stem = stem.replaceAll("[^a-zA-Z0-9_-]", "_");

    Path md = outDir.resolve(stem + "-loop-similarity.md");
    try (PrintWriter w = new PrintWriter(md.toFile())) {
      w.println("# GPX Loop Similarity - " + root);
      w.println();
      w.printf(Locale.US, "References: %d closed loops%n", refs.size());
      w.printf(Locale.US, "Algorithms: %s%n", algorithms);
      w.printf(Locale.US, "Generated: %s%n", new java.util.Date());
      w.println();

      w.println("## Summary");
      w.println();
      w.println("| Algorithm | Accepted | Avg similarity | Avg distance ratio | Avg radial MAE | Avg max-radius delta | Avg bearing delta | Avg reuse |");
      w.println("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
      for (RoundTripAlgorithm algorithm : algorithms) {
        List<ScenarioResult> subset = filterByAlgorithm(results, algorithm);
        int accepted = 0;
        double sim = 0, ratio = 0, radial = 0, maxDelta = 0, bearing = 0, reuse = 0;
        for (ScenarioResult r : subset) {
          if (r.accepted()) {
            accepted++;
            sim += r.similarity.score;
            ratio += r.distanceRatio;
            radial += r.similarity.radialMae;
            maxDelta += r.similarity.maxRadiusRatioDelta;
            bearing += r.similarity.farthestBearingDelta;
            reuse += r.loopMetrics.getRoadReusePercent();
          }
        }
        if (accepted == 0) {
          w.printf(Locale.US, "| %s | 0/%d | - | - | - | - | - | - |%n",
            algorithm, subset.size());
        } else {
          w.printf(Locale.US, "| %s | %d/%d | %.3f | %.2f | %.2f | %.2f | %.0f deg | %.1f%% |%n",
            algorithm, accepted, subset.size(), sim / accepted, ratio / accepted,
            radial / accepted, maxDelta / accepted, bearing / accepted, reuse / accepted);
        }
      }
      w.println();

      w.println("## Worst accepted shape mismatches");
      w.println();
      w.println("| Score | Algorithm | Profile | Real km | Generated ratio | Radial MAE | Max-radius real/gen | Bearing delta | File |");
      w.println("| ---: | --- | --- | ---: | ---: | ---: | --- | ---: | --- |");
      List<ScenarioResult> accepted = new ArrayList<>();
      for (ScenarioResult r : results) if (r.accepted()) accepted.add(r);
      accepted.sort(Comparator.comparingDouble(r -> r.similarity.score));
      for (ScenarioResult r : accepted.subList(0, Math.min(25, accepted.size()))) {
        w.printf(Locale.US, "| %.3f | %s | %s | %.1f | %.2f | %.2f | %.2f/%.2f | %.0f deg | %s |%n",
          r.similarity.score, r.algorithm, r.ref.profileName,
          r.ref.stats.lengthMeters / 1000.0, r.distanceRatio,
          r.similarity.radialMae,
          r.ref.stats.maxRadiusRatio, r.generated.maxRadiusRatio,
          r.similarity.farthestBearingDelta,
          r.ref.path.getFileName());
      }
      w.println();

      w.println("## Failures");
      w.println();
      w.println("| Algorithm | Profile | Real km | File | Error |");
      w.println("| --- | --- | ---: | --- | --- |");
      int failures = 0;
      for (ScenarioResult r : results) {
        if (r.accepted()) continue;
        failures++;
        String error = r.error != null ? r.error
          : (r.gate == null ? "no accepted track" : r.gate.getRejectionReason());
        w.printf(Locale.US, "| %s | %s | %.1f | %s | %s |%n",
          r.algorithm, r.ref.profileName, r.ref.stats.lengthMeters / 1000.0,
          r.ref.path.getFileName(), escapeCell(error));
      }
      if (failures == 0) w.println("| - | - | - | - | - |");
    }

    Path json = outDir.resolve(stem + "-loop-similarity.json");
    try (PrintWriter w = new PrintWriter(json.toFile())) {
      writeJson(w, results);
    }

    System.out.println("Wrote " + md);
    System.out.println("Wrote " + json);
  }

  private static List<ScenarioResult> filterByAlgorithm(List<ScenarioResult> results,
                                                        RoundTripAlgorithm algorithm) {
    List<ScenarioResult> out = new ArrayList<>();
    for (ScenarioResult r : results) {
      if (r.algorithm == algorithm) out.add(r);
    }
    return out;
  }

  private static String escapeCell(String s) {
    if (s == null) return "";
    return s.replace("|", "\\|").replace("\n", " ");
  }

  private static Path reportDir() {
    Path projectRootDocs = Paths.get("..", "docs", "features");
    if (Files.exists(projectRootDocs)) {
      return projectRootDocs.resolve("gpx-loop-similarity").normalize();
    }
    return Paths.get("docs", "features", "gpx-loop-similarity");
  }

  private static void writeJson(PrintWriter w, List<ScenarioResult> results) {
    w.println("[");
    for (int i = 0; i < results.size(); i++) {
      ScenarioResult r = results.get(i);
      if (i > 0) w.println(",");
      w.print("  {");
      w.printf(Locale.US, "\"file\":\"%s\",", json(r.ref.path.getFileName().toString()));
      w.printf(Locale.US, "\"profile\":\"%s\",", json(r.ref.profileName));
      w.printf(Locale.US, "\"algorithm\":\"%s\",", r.algorithm);
      w.printf(Locale.US, "\"realMeters\":%.1f,", r.ref.stats.lengthMeters);
      w.printf(Locale.US, "\"closureMeters\":%.1f,", r.ref.stats.closureMeters);
      w.printf(Locale.US, "\"accepted\":%s,", r.accepted());
      if (r.error != null) w.printf(Locale.US, "\"error\":\"%s\",", json(r.error));
      if (r.similarity != null) {
        w.printf(Locale.US,
          "\"score\":%.5f,\"distanceRatio\":%.5f,\"radialMae\":%.5f,"
            + "\"maxRadiusRatioDelta\":%.5f,\"bearingDelta\":%.5f,",
          r.similarity.score, r.distanceRatio, r.similarity.radialMae,
          r.similarity.maxRadiusRatioDelta, r.similarity.farthestBearingDelta);
      }
      w.printf(Locale.US, "\"runtimeMillis\":%d", r.runtimeMillis);
      w.print("}");
    }
    w.println();
    w.println("]");
  }

  private static String json(String s) {
    if (s == null) return "";
    return s.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private static final class Coord {
    final double lon;
    final double lat;

    Coord(double lon, double lat) {
      this.lon = lon;
      this.lat = lat;
    }
  }

  private static final class ReferenceLoop {
    final Path path;
    final List<Coord> coords;
    final RouteStats stats;
    String profileName;

    ReferenceLoop(Path path, List<Coord> coords, RouteStats stats) {
      this.path = path;
      this.coords = coords;
      this.stats = stats;
    }

    double startLon() {
      return coords.get(0).lon;
    }

    double startLat() {
      return coords.get(0).lat;
    }
  }

  private static final class RouteStats {
    final double lengthMeters;
    final double closureMeters;
    final double idealRadius;
    final double maxRadiusRatio;
    final double farthestBearing;
    final double[] radialRatios;

    RouteStats(double lengthMeters, double closureMeters, double idealRadius,
               double maxRadiusRatio, double farthestBearing, double[] radialRatios) {
      this.lengthMeters = lengthMeters;
      this.closureMeters = closureMeters;
      this.idealRadius = idealRadius;
      this.maxRadiusRatio = maxRadiusRatio;
      this.farthestBearing = farthestBearing;
      this.radialRatios = radialRatios;
    }

    static RouteStats fromTrack(OsmTrack track) {
      List<Coord> coords = new ArrayList<>();
      for (OsmPathElement n : track.nodes) {
        coords.add(new Coord((n.getILon() - 180000000) / 1000000.0,
          (n.getILat() - 90000000) / 1000000.0));
      }
      return fromCoords(coords);
    }

    static RouteStats fromCoords(List<Coord> coords) {
      double[] cumulative = new double[coords.size()];
      for (int i = 1; i < coords.size(); i++) {
        cumulative[i] = cumulative[i - 1] + haversine(coords.get(i - 1), coords.get(i));
      }
      double length = cumulative[cumulative.length - 1];
      double ideal = length / (2 * Math.PI);
      Coord start = coords.get(0);
      Coord farthest = start;
      double maxRadius = 0;
      for (Coord c : coords) {
        double d = haversine(start, c);
        if (d > maxRadius) {
          maxRadius = d;
          farthest = c;
        }
      }
      double[] radial = new double[RADIAL_FRACTIONS.length];
      for (int i = 0; i < RADIAL_FRACTIONS.length; i++) {
        Coord p = interpolate(coords, cumulative, RADIAL_FRACTIONS[i]);
        radial[i] = ideal <= 0 ? 0 : haversine(start, p) / ideal;
      }
      return new RouteStats(length, haversine(start, coords.get(coords.size() - 1)),
        ideal, ideal <= 0 ? 0 : maxRadius / ideal, bearing(start, farthest), radial);
    }
  }

  private static final class Similarity {
    final double score;
    final double radialMae;
    final double maxRadiusRatioDelta;
    final double farthestBearingDelta;

    Similarity(double score, double radialMae, double maxRadiusRatioDelta,
               double farthestBearingDelta) {
      this.score = score;
      this.radialMae = radialMae;
      this.maxRadiusRatioDelta = maxRadiusRatioDelta;
      this.farthestBearingDelta = farthestBearingDelta;
    }

    static Similarity compute(RouteStats ref, RouteStats generated,
                              double distanceRatio, LoopQualityMetrics metrics) {
      double radial = 0;
      for (int i = 0; i < ref.radialRatios.length; i++) {
        radial += Math.abs(generated.radialRatios[i] - ref.radialRatios[i]);
      }
      radial /= ref.radialRatios.length;

      double maxDelta = generated.maxRadiusRatio - ref.maxRadiusRatio;
      double bearingDelta = angleDiff(generated.farthestBearing, ref.farthestBearing);
      double distancePenalty = Math.min(1.0, Math.abs(distanceRatio - 1.0) / 0.5);
      double radialPenalty = Math.min(1.0, radial / 1.0);
      double maxRadiusPenalty = Math.min(1.0, Math.abs(maxDelta) / 1.5);
      double bearingPenalty = bearingDelta / 180.0;
      double reusePenalty = metrics == null ? 0.0
        : Math.min(1.0, metrics.getRoadReusePercent() / 50.0);

      double penalty = 0.30 * distancePenalty
        + 0.30 * radialPenalty
        + 0.20 * maxRadiusPenalty
        + 0.10 * bearingPenalty
        + 0.10 * reusePenalty;
      return new Similarity(clamp01(1.0 - penalty), radial, maxDelta, bearingDelta);
    }
  }

  private static final class ScenarioResult {
    final ReferenceLoop ref;
    final RoundTripAlgorithm algorithm;
    OsmTrack track;
    RouteStats generated;
    LoopQualityMetrics loopMetrics;
    RoundTripQualityResult gate;
    Similarity similarity;
    String error;
    double distanceRatio;
    long runtimeMillis;

    ScenarioResult(ReferenceLoop ref, RoundTripAlgorithm algorithm) {
      this.ref = ref;
      this.algorithm = algorithm;
    }

    boolean accepted() {
      return error == null && track != null && gate != null && gate.isAccepted()
        && similarity != null && loopMetrics != null;
    }
  }
}
