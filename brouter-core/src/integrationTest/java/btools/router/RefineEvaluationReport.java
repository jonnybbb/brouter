package btools.router;

import java.io.IOException;
import java.io.InputStream;
import java.lang.management.ManagementFactory;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Properties;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import btools.router.roundtrip.RefineDiagnostics;
import btools.router.roundtrip.RefineRouteSnapshot;

/** Machine-readable paired observations and reports computed from those observations. */
final class RefineEvaluationReport {
  private RefineEvaluationReport() {
  }

  static final class Cell {
    final String label;
    final String profile;
    final RefineDiagnostics diagnostics;
    final RefineRouteSnapshot before;
    final RefineRouteSnapshot after;
    final String reason;
    final long requestMs;
    final long addedMs;
    final boolean hasFinalRoute;
    final boolean failed;

    Cell(String label, String profile, RefineDiagnostics diagnostics, long requestMs,
         String failure, boolean hasFinalRoute) {
      this.label = label;
      this.profile = profile;
      this.diagnostics = diagnostics;
      this.before = diagnostics == null ? null : diagnostics.baseline;
      this.after = diagnostics == null ? null : diagnostics.result;
      this.reason = failure != null ? failure : diagnostics == null ? "missing_diagnostics" : diagnostics.refineReason;
      this.requestMs = requestMs;
      this.addedMs = diagnostics == null ? 0 : diagnostics.elapsedMs;
      this.hasFinalRoute = hasFinalRoute;
      this.failed = failure != null || !hasFinalRoute;
    }

    boolean eligible() {
      return diagnostics != null && diagnostics.eligible;
    }

    double gain() {
      if (failed || diagnostics == null || !diagnostics.refineApplied || diagnostics.refineTruncated
          || diagnostics.oracleCostPerMeterBefore <= 0 || diagnostics.oracleCostPerMeterAfter <= 0
          || !Double.isFinite(diagnostics.oracleCostPerMeterBefore)
          || !Double.isFinite(diagnostics.oracleCostPerMeterAfter)) return 0;
      return 100 * (diagnostics.oracleCostPerMeterBefore - diagnostics.oracleCostPerMeterAfter)
        / diagnostics.oracleCostPerMeterBefore;
    }
  }

  static final class Statistics {
    int cells;
    int eligible;
    int failed;
    int wins;
    int regressions;
    int lengthRegressions;
    int crossingRegressions;
    int gateRegressions;
    int costRegressions;
    int truncated;
    double medianGain;
    double meanGain;
    double rcsMedianDelta;
    long addedP90;
    long requestP90;
    final int latencyBar;

    Statistics(List<Cell> records, String profile, int latencyBar) {
      this.latencyBar = latencyBar;
      List<Double> gains = new ArrayList<>();
      List<Double> added = new ArrayList<>();
      List<Double> requests = new ArrayList<>();
      List<Double> baselineRcs = new ArrayList<>();
      List<Double> resultRcs = new ArrayList<>();
      for (Cell cell : records) {
        if (!profile.equals(cell.profile)) continue;
        cells++;
        if (cell.failed) failed++;
        if (cell.eligible()) eligible++;
        double gain = cell.gain();
        gains.add(gain);
        meanGain += gain;
        if (gain > 0) wins++;
        added.add((double) cell.addedMs);
        requests.add((double) cell.requestMs);
        RefineDiagnostics d = cell.diagnostics;
        if (d != null && d.refineTruncated) truncated++;
        RefineRouteSnapshot a = cell.before;
        RefineRouteSnapshot b = cell.after;
        boolean regression = cell.hasFinalRoute && (a == null || b == null);
        if (a != null && a.gateAccepted) {
          if (!cell.hasFinalRoute || b == null || !b.gateAccepted) {
            gateRegressions++;
            regression = true;
          }
          if (b != null) {
            baselineRcs.add(a.rcs);
            resultRcs.add(b.rcs);
            double target = d.requestedDistance;
            if (target <= 0 || Math.abs(b.distance / target - 1) > Math.abs(a.distance / target - 1) + 1e-9) {
              lengthRegressions++;
              regression = true;
            }
            if (b.crossings > a.crossings) {
              crossingRegressions++;
              regression = true;
            }
            if (b.scatterReuse > a.scatterReuse || b.rcs < a.rcs - 0.02 - 1e-9) regression = true;
            if (!d.refineApplied && a.geometrySignature != b.geometrySignature) regression = true;
            if (d.refineApplied && (d.oracleCostPerMeterBefore <= 0 || d.oracleCostPerMeterAfter <= 0
                || !Double.isFinite(d.oracleCostPerMeterAfter)
                || d.oracleCostPerMeterAfter > 0.995 * d.oracleCostPerMeterBefore)) {
              costRegressions++;
              regression = true;
            }
          }
        }
        if (regression) regressions++;
      }
      medianGain = median(gains);
      meanGain = cells == 0 ? 0 : meanGain / cells;
      rcsMedianDelta = median(resultRcs) - median(baselineRcs);
      addedP90 = (long) percentile90(added);
      requestP90 = (long) percentile90(requests);
    }

    boolean passes() {
      return cells > 0 && failed == 0 && medianGain >= 2 && regressions == 0 && rcsMedianDelta >= -1e-9
        && addedP90 <= latencyBar && (double) truncated / cells <= 0.05;
    }
  }

  static double median(List<Double> values) {
    if (values.isEmpty()) return 0;
    Collections.sort(values);
    int n = values.size();
    return n % 2 == 0 ? (values.get(n / 2 - 1) + values.get(n / 2)) / 2 : values.get(n / 2);
  }

  static double percentile90(List<Double> values) {
    if (values.isEmpty()) return 0;
    Collections.sort(values);
    return values.get(Math.max(0, (int) Math.ceil(values.size() * 0.9) - 1));
  }

  static void appendRecord(Path output, Cell c) throws IOException {
    Path file = output.resolve("cells.csv");
    if (!Files.exists(file)) {
      Files.writeString(file, "cell,profile,reason,has_final_route,failed,eligible,applied,truncated,request_ms,added_ms,"
        + "measurement_ms,oracle_before,oracle_after,official_gain_pct,direction,target_m,producing_tier,"
        + "baseline_signature,result_signature,baseline_distance,result_distance,baseline_rcs,result_rcs,"
        + "baseline_crossings,result_crossings,baseline_scatter,result_scatter,baseline_gate,result_gate,"
        + "baseline_historical_cost,result_historical_cost,baseline_pricing_method,result_pricing_method,"
        + "proposals,evaluations,invalid_proposals,legs_routed,cache_hits,finalizations,initialization_ms,"
        + "routing_ms,pricing_ms,snapping_ms,cleanup_ms,finalization_ms,timeout_operation,baseline_pricing_failure,raw_baseline_pricing_failure\n");
    }
    RefineDiagnostics d = c.diagnostics == null ? new RefineDiagnostics() : c.diagnostics;
    RefineRouteSnapshot a = c.before;
    RefineRouteSnapshot b = c.after;
    Object[] values = {c.label, c.profile, c.reason, c.hasFinalRoute, c.failed, c.eligible(), d.refineApplied,
      d.refineTruncated, c.requestMs, c.addedMs, d.measurementMs, d.oracleCostPerMeterBefore,
      d.oracleCostPerMeterAfter, c.gain(), d.resolvedDirection, d.requestedDistance, d.producingTier,
      a == null ? "" : Long.toHexString(a.geometrySignature), b == null ? "" : Long.toHexString(b.geometrySignature),
      a == null ? "" : a.distance, b == null ? "" : b.distance, a == null ? "" : a.rcs, b == null ? "" : b.rcs,
      a == null ? "" : a.crossings, b == null ? "" : b.crossings,
      a == null ? "" : a.scatterReuse, b == null ? "" : b.scatterReuse,
      a == null ? "" : a.gateAccepted, b == null ? "" : b.gateAccepted,
      a == null ? "" : a.historicalCost, b == null ? "" : b.historicalCost,
      d.baselinePricingMethod, d.candidatePricingMethod, d.proposals, d.evaluations, d.invalidProposals,
      d.legsRouted, d.cacheHits, d.finalizations, d.initializationMs, d.routingMs, d.pricingMs,
      d.snappingMs, d.cleanupMs, d.finalizationMs, d.timeoutOperation, d.baselinePricingFailure, d.rawBaselinePricingFailure};
    Files.writeString(file, csv(values), StandardOpenOption.APPEND);
    appendRideAnalysis(output, c);
    ensureHeader(output.resolve("rejections.csv"), "cell,reason,count\n");
    ensureHeader(output.resolve("evaluations.csv"), "cell,evaluation,operator,energy,accepted,feasible\n");
    for (Map.Entry<String, Integer> entry : d.rejectionCounts.entrySet()) {
      Files.writeString(output.resolve("rejections.csv"), csv(c.label, entry.getKey(), entry.getValue()),
        StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    }
    for (RefineDiagnostics.EvaluationTrace trace : d.traces) {
      Files.writeString(output.resolve("evaluations.csv"), csv(c.label, trace.evaluation, trace.operator,
        trace.energy, trace.accepted, trace.feasible), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    }
  }

  private static void appendRideAnalysis(Path output, Cell cell) throws IOException {
    Path file = output.resolve("ride-analysis.csv");
    ensureHeader(file, "cell,profile,stage,geometry_signature,analysis_version,analysis_ms,metric,value\n");
    RefineRouteSnapshot[] snapshots = {cell.before, cell.after};
    for (int i = 0; i < snapshots.length; i++) {
      RefineRouteSnapshot snapshot = snapshots[i];
      if (snapshot == null) continue;
      StringBuilder rows = new StringBuilder();
      for (Map.Entry<String, Double> entry : snapshot.ride.values().entrySet()) {
        rows.append(csv(cell.label, cell.profile, i == 0 ? "before" : "after",
          Long.toHexString(snapshot.geometrySignature), RouteRideAnalysis.VERSION,
          snapshot.ride.elapsedNanos / 1000000.0, entry.getKey(),
          Double.isFinite(entry.getValue()) ? entry.getValue() : ""));
      }
      Files.writeString(file, rows.toString(), StandardOpenOption.APPEND);
    }
  }

  private static void ensureHeader(Path file, String header) throws IOException {
    if (!Files.exists(file)) Files.writeString(file, header);
  }

  private static String csv(Object... values) {
    List<String> fields = new ArrayList<>();
    for (Object value : values) fields.add("\"" + String.valueOf(value == null ? "" : value).replace("\"", "\"\"") + "\"");
    return String.join(",", fields) + "\n";
  }

  static void writeSummary(Path output, List<Cell> records, boolean fullMatrix, int latencyBar) throws IOException {
    StringBuilder text = new StringBuilder("# Paired refinement evaluation\n\n");
    text.append(fullMatrix ? "Full 460-cell manifest.\n\n" : "Subset validation; no product acceptance decision.\n\n");
    text.append("Original and result snapshots come from one request at the refinement hook. "
      + "Skips, failures and truncations contribute zero improvement. "
      + "All cells remain in latency and truncation statistics. RCS uses the ranking score.\n\n");
    text.append("| Profile | Cells | Failed requests | Eligible | Wins | Median gain % | Mean gain % | Regressions | "
      + "Length | Crossings | Gates | Cost | RCS median delta | Added p90 ms | Request p90 ms | Truncated | Bars |\n");
    text.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n");
    for (String profile : new String[]{"gravel.brf", "fastbike.brf"}) {
      Statistics s = new Statistics(records, profile, latencyBar);
      text.append(String.format(Locale.ROOT,
        "| %s | %d | %d | %d | %d | %.4f | %.4f | %d | %d | %d | %d | %d | %.6f | %d | %d | %d | %s |%n",
        profile, s.cells, s.failed, s.eligible, s.wins, s.medianGain, s.meanGain, s.regressions, s.lengthRegressions,
        s.crossingRegressions, s.gateRegressions, s.costRegressions, s.rcsMedianDelta,
        s.addedP90, s.requestP90, s.truncated,
        fullMatrix && "gravel.brf".equals(profile) ? s.passes() ? "PASS" : "FAIL" : "not assessed"));
    }
    text.append("\nTimings in cells.csv include initialization, leg routing/retracking, continuous pricing, "
      + "proposal snapping, cleanup and full finalization. Component timings overlap: initialization and "
      + "finalization contain routing/pricing/cleanup. Measurement overhead is reported separately; "
      + "request time includes it. A successful run of the tests is not a claim of optimizer quality.\n");
    text.append("\nRide characteristics are recorded separately in ride-analysis.csv and ride-analysis.html. "
      + "They do not influence routing or the acceptance bars. Missing measurements are blank, not zero.\n");
    Files.writeString(output.resolve("summary.md"), text.toString());
    writeRideReport(output, records);
    RefineRideReview.write(output, records);
  }

  private static void writeRideReport(Path output, List<Cell> records) throws IOException {
    StringBuilder html = new StringBuilder("<!doctype html><html lang=\"en\"><meta charset=\"utf-8\">"
      + "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      + "<title>Route characteristics</title><style>body{font:16px system-ui;max-width:1100px;margin:40px auto;padding:0 20px;color:#21313c}"
      + "table{border-collapse:collapse;width:100%;margin:16px 0}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:right}"
      + "td:first-child,th:first-child{text-align:left}summary{cursor:pointer;padding:14px;background:#eef3f5}"
      + "details{margin:14px 0}p{line-height:1.5}small{color:#53636f}</style><h1>Route characteristics</h1>"
      + "<p>Original and result from the same request. These are observations, not an enjoyment score. "
      + "Traffic and scenery use mapped estimates. Unknown data is shown as unavailable. "
      + "A zero observed exposure must be read alongside its coverage percentage.</p>"
      + "<p>Moving time sums the original model estimates for verified surviving segments. It excludes stop delays and does not rerun the physics after cleanup. "
      + "Elevation uses complete 100 m windows; coverage excludes incomplete tails and missing elevations. "
      + "Sharp geometry bends are not necessarily junction turns. Paved does not mean smooth.</p>"
      + "<p><a href=\"ride-analysis.csv\">Download measurements</a> · <a href=\"manifest.properties\">Input manifest</a></p>");
    for (Cell cell : records) {
      html.append("<details><summary>").append(escape(cell.label)).append(" · ").append(escape(cell.reason)).append("</summary>");
      if (cell.before == null || cell.after == null) {
        html.append("<p>Paired measurements unavailable.</p></details>");
        continue;
      }
      html.append("<p>Geometry ").append(cell.before.geometrySignature == cell.after.geometrySignature ? "unchanged" : "changed")
        .append(". Analysis time: ").append(String.format(Locale.ROOT, "%.3f / %.3f ms", cell.before.ride.elapsedNanos / 1e6,
          cell.after.ride.elapsedNanos / 1e6)).append(".</p>");
      html.append("<table><thead><tr><th>Measurement</th><th>Before</th><th>After</th></tr></thead><tbody>");
      for (Map.Entry<String, Double> entry : cell.before.ride.values().entrySet()) {
        html.append("<tr><td>").append(escape(entry.getKey().replace('_', ' '))).append("</td><td>")
          .append(display(entry.getValue())).append("</td><td>")
          .append(display(cell.after.ride.value(entry.getKey()))).append("</td></tr>");
      }
      html.append("<tr><td>Self intersections</td><td>").append(cell.before.crossings).append("</td><td>")
        .append(cell.after.crossings).append("</td></tr></tbody></table></details>");
    }
    html.append("<p><small>Analysis version ").append(RouteRideAnalysis.VERSION)
      .append(". Access observations are explicit tags, not a legal-access determination. "
        + "No current closures, resupply availability or real-world scenery verification is included.</small></p></html>");
    Files.writeString(output.resolve("ride-analysis.html"), html.toString());
  }

  private static String display(double value) {
    return Double.isFinite(value) ? String.format(Locale.ROOT, "%.2f", value) : "unavailable";
  }

  private static String escape(String value) {
    return value == null ? "" : value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
      .replace("\"", "&quot;").replace("'", "&#39;");
  }

  static void writeManifest(Path output, Path root, Path segments, List<FullABEvaluationMatrixTest.CellSpec> cells,
                            String mode, int evals, int maxMs) throws Exception {
    Properties manifest = new Properties();
    manifest.setProperty("commit", git(root, "rev-parse", "HEAD").trim());
    manifest.setProperty("jvm", System.getProperty("java.version") + " " + System.getProperty("java.vendor"));
    manifest.setProperty("jvm.args", ManagementFactory.getRuntimeMXBean().getInputArguments().toString());
    manifest.setProperty("os", System.getProperty("os.name") + " " + System.getProperty("os.version") + " " + System.getProperty("os.arch"));
    manifest.setProperty("logical.cores", String.valueOf(Runtime.getRuntime().availableProcessors()));
    manifest.setProperty("max.heap.bytes", String.valueOf(Runtime.getRuntime().maxMemory()));
    manifest.setProperty("matrix.workers", "1; cells execute sequentially in one test class");
    manifest.setProperty("mode", mode);
    manifest.setProperty("evaluations", String.valueOf(evals));
    manifest.setProperty("max.ms", String.valueOf(maxMs));
    manifest.setProperty("variety.seed", "0");
    manifest.setProperty("operator.mix", "move=0.50,2opt=0.20,replace=0.15,insert=0.15; invalid alternatives fall back to move");
    manifest.setProperty("chains", "1");
    manifest.setProperty("top.k", "3");
    manifest.setProperty("max.proposals", String.valueOf(4 * evals));
    manifest.setProperty("annealing.temperature", "max(0.001,0.05*(1-evaluation/evaluations))");
    manifest.setProperty("strict.quality", "false");
    manifest.setProperty("measurement", "true");
    manifest.setProperty("profile.parameter.processUnusedTags", "1");
    manifest.setProperty("ride.analysis.version", RouteRideAnalysis.VERSION);
    manifest.setProperty("source.archive", "inputs.zip; code, build files and profiles, including untracked Java sources");
    manifest.setProperty("request.max.ms", "60000");
    manifest.setProperty("segments.directory", segments.toAbsolutePath().toString());
    manifest.setProperty("cell.count", String.valueOf(cells.size()));
    StringBuilder selection = new StringBuilder("cell,profile,algorithm,lon,lat,radius_m,target_label_m,direction\n");
    for (FullABEvaluationMatrixTest.CellSpec c : cells) {
      selection.append(csv(c.label, c.profileName, c.algorithm, c.region.lon, c.region.lat,
        c.searchRadius, c.targetDistanceMeters, c.direction));
    }
    Files.writeString(output.resolve("manifest-cells.csv"), selection.toString());
    Files.writeString(output.resolve("working-tree.patch"), git(root, "diff", "--binary", "HEAD"));
    List<Path> inputs = new ArrayList<>();
    try (Stream<Path> paths = Files.walk(root)) {
      inputs.addAll(paths.filter(Files::isRegularFile).filter(path -> {
        String relative = root.relativize(path).toString();
        return !relative.startsWith(".worktrees/") && !relative.contains("/build/")
          && !relative.startsWith(".git/") && ((relative.contains("/src/") && (relative.endsWith(".java") || relative.contains("/resources/")))
          || relative.startsWith("misc/profiles2/") || relative.endsWith(".gradle"));
      }).collect(Collectors.toList()));
    }
    try (ZipOutputStream archive = new ZipOutputStream(Files.newOutputStream(output.resolve("inputs.zip")))) {
      for (Path input : inputs) {
        archive.putNextEntry(new ZipEntry(root.relativize(input).toString()));
        Files.copy(input, archive);
        archive.closeEntry();
      }
    }
    if (Files.isDirectory(segments)) {
      try (Stream<Path> paths = Files.list(segments)) {
        inputs.addAll(paths.filter(path -> path.toString().endsWith(".rd5")).collect(Collectors.toList()));
      }
    }
    manifest.setProperty("segments.files", segmentFiles(segments));
    Collections.sort(inputs);
    for (Path input : inputs) manifest.setProperty("sha256." + root.relativize(input), digest(input));
    try (java.io.Writer writer = Files.newBufferedWriter(output.resolve("manifest.properties"))) {
      manifest.store(writer, "Frozen inputs before routing; no tile downloads occur in this evaluation");
    }
  }

  static void verifyManifest(Path output, Path root, Path segments) throws Exception {
    Properties properties = new Properties();
    try (java.io.Reader reader = Files.newBufferedReader(output.resolve("manifest.properties"))) {
      properties.load(reader);
    }
    for (String key : properties.stringPropertyNames()) {
      if (key.startsWith("sha256.")) {
        Path input = root.resolve(key.substring("sha256.".length()));
        if (!properties.getProperty(key).equals(digest(input))) {
          throw new IOException("Input changed during evaluation: " + input);
        }
      }
    }
    if (!properties.getProperty("segments.files").equals(segmentFiles(segments))) {
      throw new IOException("Segment files added or removed during evaluation");
    }
    if (!properties.getProperty("segments.directory").equals(segments.toAbsolutePath().toString())) {
      throw new IOException("Segment directory changed");
    }
  }

  private static String segmentFiles(Path segments) throws IOException {
    if (!Files.isDirectory(segments)) return "";
    try (Stream<Path> paths = Files.list(segments)) {
      return paths.filter(path -> path.toString().endsWith(".rd5"))
        .map(path -> path.getFileName().toString()).sorted().collect(Collectors.joining(","));
    }
  }

  private static String digest(Path path) throws Exception {
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    byte[] buffer = new byte[65536];
    try (InputStream input = Files.newInputStream(path)) {
      int size;
      while ((size = input.read(buffer)) != -1) digest.update(buffer, 0, size);
    }
    StringBuilder hex = new StringBuilder();
    for (byte value : digest.digest()) hex.append(String.format(Locale.ROOT, "%02x", value & 255));
    return hex.toString();
  }

  private static String git(Path root, String... args) throws Exception {
    List<String> command = new ArrayList<>();
    command.add("git");
    Collections.addAll(command, args);
    Process process = new ProcessBuilder(command).directory(root.toFile()).redirectErrorStream(true).start();
    String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    if (process.waitFor() != 0) throw new IOException("Cannot record git provenance: " + output);
    return output;
  }
}
