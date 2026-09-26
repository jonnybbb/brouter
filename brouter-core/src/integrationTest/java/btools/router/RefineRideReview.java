package btools.router;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import btools.router.roundtrip.RefineRouteSnapshot;

/** Neutral A/B exports and a systematic, reviewable sample with access to the entire corpus. */
final class RefineRideReview {
  private RefineRideReview() {
  }

  static void write(Path output, List<RefineEvaluationReport.Cell> records) throws IOException {
    List<RefineEvaluationReport.Cell> sorted = new ArrayList<>(records);
    sorted.sort(Comparator.comparing(cell -> cell.label));
    Set<String> strata = new HashSet<>();
    StringBuilder data = new StringBuilder("window.rideReview=[");
    Files.createDirectories(output.resolve("gpx"));
    for (int i = 0; i < sorted.size(); i++) {
      RefineEvaluationReport.Cell cell = sorted.get(i);
      boolean swap = Math.floorMod(cell.label.hashCode(), 2) == 1;
      RefineRouteSnapshot a = swap ? cell.after : cell.before;
      RefineRouteSnapshot b = swap ? cell.before : cell.after;
      String region = cell.label.replaceFirst("_[0-9]+km_.*$", "");
      String outcome = cell.failed ? "failed" : cell.gain() > 0 ? "accepted"
        : cell.diagnostics != null && cell.diagnostics.refineTruncated ? "truncated"
        : cell.reason != null && cell.reason.contains("unpriceable") ? "unpriceable"
        : cell.before == null || cell.after == null ? "unobserved" : "unchanged";
      boolean sample = strata.add(region + ":" + cell.profile + ":" + outcome);
      String filename = String.format(Locale.ROOT, "pair-%03d", i + 1);
      if (i > 0) data.append(',');
      data.append("{\"label\":").append(json(cell.label)).append(",\"region\":").append(json(region))
        .append(",\"profile\":").append(json(cell.profile)).append(",\"reason\":").append(json(cell.reason))
        .append(",\"outcome\":").append(json(outcome)).append(",\"sample\":").append(sample)
        .append(",\"aIsOriginal\":").append(!swap).append(",\"requestMs\":").append(cell.requestMs)
        .append(",\"addedMs\":").append(cell.addedMs).append(",\"gain\":").append(cell.gain())
        .append(",\"a\":").append(route(a, "gpx/" + filename + "-A.gpx"))
        .append(",\"b\":").append(route(b, "gpx/" + filename + "-B.gpx")).append('}');
      if (a != null) Files.writeString(output.resolve("gpx/" + filename + "-A.gpx"), gpx(a, "Route A"));
      if (b != null) Files.writeString(output.resolve("gpx/" + filename + "-B.gpx"), gpx(b, "Route B"));
    }
    Files.writeString(output.resolve("ride-review-data.js"), data.append("];\n").toString());
    try (InputStream template = RefineRideReview.class.getResourceAsStream("/refine-ride-review.html")) {
      if (template == null) throw new IOException("Missing ride review template");
      Files.copy(template, output.resolve("ride-review.html"), java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    }
  }

  private static String route(RefineRouteSnapshot snapshot, String gpx) {
    if (snapshot == null) return "null";
    StringBuilder out = new StringBuilder("{\"gpx\":").append(json(gpx)).append(",\"points\":[");
    for (int i = 0; i < snapshot.geometry.size(); i++) {
      if (i > 0) out.append(',');
      RefineRouteSnapshot.Point p = snapshot.geometry.get(i);
      out.append('[').append(p.ilon).append(',').append(p.ilat).append(',')
        .append(p.elevation == Short.MIN_VALUE ? "null" : Double.toString(p.elevation / 4.0)).append(']');
    }
    out.append("],\"metrics\":{");
    boolean first = true;
    for (Map.Entry<String, Double> metric : snapshot.ride.values().entrySet()) {
      if (!first) out.append(',');
      first = false;
      out.append(json(metric.getKey())).append(':').append(Double.isFinite(metric.getValue()) ? metric.getValue() : "null");
    }
    return out.append("}}").toString();
  }

  private static String gpx(RefineRouteSnapshot snapshot, String name) {
    StringBuilder xml = new StringBuilder("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      + "<gpx version=\"1.1\" creator=\"BRouter paired review\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
      + "<trk><name>").append(name).append("</name><trkseg>\n");
    for (RefineRouteSnapshot.Point p : snapshot.geometry) {
      xml.append(String.format(Locale.ROOT, "<trkpt lat=\"%.6f\" lon=\"%.6f\">",
        p.ilat / 1e6 - 90, p.ilon / 1e6 - 180));
      if (p.elevation != Short.MIN_VALUE) xml.append("<ele>").append(p.elevation / 4.0).append("</ele>");
      xml.append("</trkpt>\n");
    }
    return xml.append("</trkseg></trk></gpx>\n").toString();
  }

  private static String json(String text) {
    if (text == null) return "null";
    return '"' + text.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n")
      .replace("\r", "\\r").replace("\t", "\\t").replace("<", "\\u003c") + '"';
  }
}
