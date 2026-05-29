package btools.router.known;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoundTripQualityGate;
import btools.router.RoundTripQualityGate.HostileStretch;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

import org.junit.Assume;
import org.junit.Test;

/**
 * Aggregate GPX-replay test — processes a whole directory of GPX files
 * at runtime, samples N waypoints from each, runs brouter point-to-point,
 * and reports a single summary instead of emitting per-route test
 * methods. Designed for large corpuses (the Freiburg directory has 956
 * GPXs — generating that many Java methods would be unwieldy).
 *
 * <p>Run with: {@code -Dloop.tests=true -Dgpx.dir=tmp/gpx/freiburg
 * --tests "btools.router.known.GpxReplayAggregateTest"}
 *
 * <p>Outputs a markdown report at
 * {@code docs/features/gpx-replay/<dir>-aggregate.md} with:
 * <ul>
 *   <li>pass / fail counts</li>
 *   <li>top failure causes (e.g. "highway=track tracktype=grade1" count)</li>
 *   <li>worst-case scenarios for human review</li>
 * </ul>
 */
public class GpxReplayAggregateTest {

  private static final int N_WAYPOINTS = 6;
  private static final Pattern TRKPT = Pattern.compile("<trkpt\\s+lat=\"([^\"]+)\"\\s+lon=\"([^\"]+)\"");

  private File profileFile(String name) {
    File p = new File("misc/profiles2/" + name + ".brf");
    if (!p.exists()) p = new File("../misc/profiles2/" + name + ".brf");
    return p.exists() ? p : null;
  }

  private File segDir() {
    File f = new File("../segments4");
    if (!f.exists()) f = new File("segments4");
    return f.exists() ? f : null;
  }

  private static String segmentFileFor(double lat, double lon) {
    int tileLon = (int) Math.floor(lon / 5) * 5;
    int tileLat = (int) Math.floor(lat / 5) * 5;
    return "E" + tileLon + "_N" + tileLat + ".rd5";
  }

  /** Picks a "good" start point — first trkpt is sometimes a snap to nowhere; use the 2nd. */
  private static List<double[]> sampleTrkpts(Path gpx, int n) throws IOException {
    List<double[]> all = new ArrayList<>();
    String text = new String(Files.readAllBytes(gpx));
    Matcher m = TRKPT.matcher(text);
    while (m.find()) all.add(new double[]{Double.parseDouble(m.group(2)), Double.parseDouble(m.group(1))});
    if (all.size() < n) return new ArrayList<>();
    List<double[]> sampled = new ArrayList<>();
    double step = all.size() / (double) n;
    for (int i = 0; i < n; i++) {
      int idx = (int) Math.min(all.size() - 1, Math.round(i * step));
      sampled.add(all.get(idx));
    }
    return sampled;
  }

  static class ScenarioResult {
    final String file;
    final int km;
    final String error;
    final int routeMeters;
    final int worstHostile;
    final String hostileTags;
    final double hostileLat;
    final double hostileLon;
    ScenarioResult(String file, int km, String error, int routeMeters,
                   int worstHostile, String hostileTags, double hostileLat, double hostileLon) {
      this.file = file; this.km = km; this.error = error; this.routeMeters = routeMeters;
      this.worstHostile = worstHostile; this.hostileTags = hostileTags;
      this.hostileLat = hostileLat; this.hostileLon = hostileLon;
    }
    boolean passed() {
      return error == null
        && worstHostile <= RoundTripQualityGate.MAX_CONTIGUOUS_HOSTILE_METERS;
    }
  }

  @Test
  public void replayAggregate() throws IOException {
    Assume.assumeTrue("loop-tests opt-in", Boolean.getBoolean("loop.tests"));
    String dir = System.getProperty("gpx.dir");
    Assume.assumeTrue("must pass -Dgpx.dir=<path>", dir != null);
    File segDir = segDir();
    Assume.assumeTrue("segments4 dir not found", segDir != null);
    File pf = profileFile("fastbike");
    Assume.assumeTrue("fastbike profile not found", pf != null);

    Path root = Paths.get(dir);
    if (!Files.exists(root)) {
      // Tests run from brouter-core/ — try resolving against the project root.
      Path alt = Paths.get("..").resolve(dir).normalize();
      if (Files.exists(alt)) root = alt;
    }
    Assume.assumeTrue("gpx.dir not found: " + root, Files.exists(root));
    List<Path> gpxFiles = new ArrayList<>();
    Files.walk(root)
      .filter(p -> p.toString().toLowerCase(Locale.ROOT).endsWith(".gpx"))
      .forEach(gpxFiles::add);
    System.out.println("GpxReplayAggregateTest: scanning " + gpxFiles.size() + " GPX files under " + dir);

    List<ScenarioResult> results = new ArrayList<>();
    int processed = 0;
    long t0 = System.currentTimeMillis();
    for (Path gpx : gpxFiles) {
      processed++;
      List<double[]> sampled;
      try {
        sampled = sampleTrkpts(gpx, N_WAYPOINTS);
      } catch (IOException e) { continue; }
      if (sampled.isEmpty()) continue;

      double[] first = sampled.get(0);
      File segFile = new File(segDir, segmentFileFor(first[1], first[0]));
      if (!segFile.exists()) {
        results.add(new ScenarioResult(gpx.getFileName().toString(), -1,
          "no segment for " + segFile.getName(), 0, 0, null, 0, 0));
        continue;
      }

      List<OsmNodeNamed> wplist = new ArrayList<>();
      for (int i = 0; i < sampled.size(); i++) {
        double[] p = sampled.get(i);
        OsmNodeNamed n = new OsmNodeNamed();
        n.name = i == 0 ? "from" : i == sampled.size() - 1 ? "to" : "via" + i;
        n.ilon = (int) Math.round((p[0] + 180) * 1_000_000);
        n.ilat = (int) Math.round((p[1] + 90) * 1_000_000);
        wplist.add(n);
      }

      RoutingContext rctx = new RoutingContext();
      rctx.localFunction = pf.getAbsolutePath();
      RoutingEngine re = new RoutingEngine(null, null, segDir, wplist, rctx);
      re.quite = true;
      String err = null;
      OsmTrack track = null;
      try {
        re.doRun(0);
        err = re.getErrorMessage();
        track = re.getFoundTrack();
      } catch (RuntimeException ex) {
        err = ex.getClass().getSimpleName() + ": " + ex.getMessage();
      }

      int meters = track != null ? track.distance : 0;
      int worstH = 0;
      String htags = null;
      double hLat = 0, hLon = 0;
      if (track != null) {
        HostileStretch hs = RoundTripQualityGate.worstHostileStretchPaved(track);
        if (hs.isPresent()) {
          worstH = hs.meters;
          htags = hs.startTags;
          hLat = hs.startLat();
          hLon = hs.startLon();
        }
      }
      int km = parseKm(gpx.getFileName().toString());
      results.add(new ScenarioResult(gpx.getFileName().toString(), km, err, meters,
        worstH, htags, hLat, hLon));
      if (processed % 100 == 0) {
        System.out.println("  ..." + processed + "/" + gpxFiles.size() + " processed");
      }
    }
    long dt = System.currentTimeMillis() - t0;
    System.out.println("Processed " + processed + " files in " + dt + "ms");

    writeReport(dir, results);
  }

  private static int parseKm(String filename) {
    // e.g. "2026-05-24__202km__2980978686__Zum dicken Daniel.gpx"
    Matcher m = Pattern.compile("__(\\d+)km__").matcher(filename);
    return m.find() ? Integer.parseInt(m.group(1)) : -1;
  }

  private void writeReport(String dir, List<ScenarioResult> results) throws IOException {
    int total = results.size();
    int passed = 0;
    int routedButHostile = 0;
    int errored = 0;
    java.util.Map<String, Integer> tagCounts = new java.util.HashMap<>();
    for (ScenarioResult r : results) {
      if (r.passed()) passed++;
      else if (r.error == null) {
        routedButHostile++;
        if (r.hostileTags != null) {
          String key = canonicalizeTags(r.hostileTags);
          tagCounts.merge(key, 1, Integer::sum);
        }
      } else errored++;
    }
    Path outDir = Paths.get("docs/features/gpx-replay");
    Files.createDirectories(outDir);
    String stem = dir.replaceAll(".*/", "").replaceAll("[^a-zA-Z0-9_-]", "_");
    Path md = outDir.resolve(stem + "-aggregate.md");
    try (PrintWriter w = new PrintWriter(md.toFile())) {
      w.println("# GPX Replay Aggregate — " + dir);
      w.println();
      w.printf("Source: %d GPX files%n", total);
      w.printf("Generated: %s%n", new java.util.Date());
      w.println();
      w.println("## Summary");
      w.println();
      w.printf("| Outcome | Count | %% |%n");
      w.println("| --- | --- | --- |");
      w.printf("| **Passed** (clean route, no hostile over cap) | %d | %.1f%% |%n", passed, 100.0 * passed / total);
      w.printf("| Routed but hostile stretch over cap | %d | %.1f%% |%n", routedButHostile, 100.0 * routedButHostile / total);
      w.printf("| Failed to route | %d | %.1f%% |%n", errored, 100.0 * errored / total);
      w.println();

      if (!tagCounts.isEmpty()) {
        w.println("## Top hostile-tag patterns (failures only)");
        w.println();
        w.println("| Count | Canonical tags |");
        w.println("| --- | --- |");
        tagCounts.entrySet().stream()
          .sorted(java.util.Map.Entry.<String, Integer>comparingByValue().reversed())
          .limit(20)
          .forEach(e -> w.printf("| %d | `%s` |%n", e.getValue(), e.getKey()));
        w.println();
      }

      List<ScenarioResult> worstByHostile = new ArrayList<>(results);
      worstByHostile.sort(Comparator.comparingInt((ScenarioResult r) -> r.worstHostile).reversed());
      w.println("## Top 15 worst-hostile scenarios");
      w.println();
      w.println("| Hostile (m) | km | File | Location | Tags |");
      w.println("| --- | --- | --- | --- | --- |");
      for (ScenarioResult r : worstByHostile.subList(0, Math.min(15, worstByHostile.size()))) {
        if (r.worstHostile <= 0) continue;
        w.printf(Locale.ROOT, "| %d | %d | %s | https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=15/%.5f/%.5f | `%s` |%n",
          r.worstHostile, r.km, r.file, r.hostileLat, r.hostileLon, r.hostileLat, r.hostileLon,
          r.hostileTags == null ? "" : r.hostileTags.replace("|", "\\|"));
      }
    }
    System.out.println("Wrote " + md);
    System.out.printf("Aggregate: %d/%d passed (%.1f%%), %d routed-but-hostile, %d errored%n",
      passed, total, 100.0 * passed / total, routedButHostile, errored);
  }

  private static String canonicalizeTags(String tags) {
    // Strip reversedirection and noise; keep highway / surface / tracktype /
    // bicycle / access / route_bicycle.
    String[] keep = {"highway", "surface", "tracktype", "bicycle", "access", "route_bicycle_lcn", "route_bicycle_rcn", "route_bicycle_ncn", "foot"};
    StringBuilder sb = new StringBuilder();
    for (String tok : tags.split(" ")) {
      for (String k : keep) {
        if (tok.startsWith(k + "=")) { if (sb.length() > 0) sb.append(' '); sb.append(tok); break; }
      }
    }
    return sb.toString();
  }
}
