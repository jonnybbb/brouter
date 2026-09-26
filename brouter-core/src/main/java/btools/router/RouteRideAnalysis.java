package btools.router;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import btools.router.roundtrip.LoopQualityMetrics;
import btools.util.CheapRuler;

/** Descriptive measurements of surviving geometry. Never a routing or publication score. */
public final class RouteRideAnalysis {
  public static final String VERSION = "1";
  private static final double ELEVATION_WINDOW_METERS = 100;
  private static final double STEEP_PERCENT = 8;
  private final Map<String, Double> values;
  public final long elapsedNanos;

  private RouteRideAnalysis(Map<String, Double> values, long elapsedNanos) {
    this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
    this.elapsedNanos = elapsedNanos;
  }

  /** NaN means unavailable, not zero. Names carry units; see docs/route-ride-analysis.md. */
  public Map<String, Double> values() {
    return values;
  }

  public double value(String name) {
    Double value = values.get(name);
    if (value == null) throw new IllegalArgumentException("Unknown measurement: " + name);
    return value;
  }

  public static RouteRideAnalysis measure(OsmTrack track, double requestedDistance) {
    long start = System.nanoTime();
    if (track == null || track.nodes == null || track.nodes.size() < 2) {
      return new RouteRideAnalysis(Collections.emptyMap(), System.nanoTime() - start);
    }
    List<OsmPathElement> nodes = track.nodes;
    Map<String, Double> m = new LinkedHashMap<>();
    String[] totals = {"distance_m", "metadata_verified_m", "surface_known_m", "surface_paved_m",
      "surface_firm_unpaved_m", "surface_other_unpaved_m", "surface_other_m", "smoothness_known_m",
      "poor_smoothness_m", "traffic_estimate_known_m", "high_traffic_estimate_m", "low_traffic_estimate_m",
      "forest_estimate_known_m", "high_forest_estimate_m", "river_estimate_known_m", "high_river_estimate_m",
      "highway_known_m", "arterial_m", "steps_m", "explicit_dismount_m", "explicit_bicycle_no_private_m",
      "tagged_signal_encounters", "tagged_stop_encounters", "tagged_barrier_encounters",
      "node_tagged_encounters", "moving_time_covered_m", "partial_model_moving_seconds",
      "elevation_known_m", "grade_window_covered_m", "ascent_100m_m", "descent_100m_m",
      "steep_up_100m_m", "steep_down_100m_m", "final_quarter_ascent_100m_m", "max_up_grade_100m_pct",
      "max_down_grade_100m_pct", "sharp_geometry_bends", "geometry_uturns"};
    for (String name : totals) m.put(name, 0.0);
    Map<String, Map<String, String>> parsed = new HashMap<>();
    Run paved = new Run();
    Run firm = new Run();
    Run highTraffic = new Run();
    Run lowTraffic = new Run();
    double[] distances = new double[nodes.size()];
    for (int i = 1; i < nodes.size(); i++) {
      OsmPathElement a = nodes.get(i - 1);
      OsmPathElement b = nodes.get(i);
      double distance = CheapRuler.distance(a.getILon(), a.getILat(), b.getILon(), b.getILat());
      distances[i] = distances[i - 1] + distance;
      if (distance <= 0) continue;
      add(m, "distance_m", distance);
      MessageData message = b.message;
      boolean verified = message != null && message.describesSegment(a.getIdFromPos(), b.getIdFromPos());
      Map<String, String> tags = verified ? tags(message.wayKeyValues, parsed) : Collections.emptyMap();
      if (verified) add(m, "metadata_verified_m", distance);
      String surface = tags.get("surface");
      int surfaceClass = surfaceClass(surface);
      if (surfaceClass >= 0) {
        add(m, "surface_known_m", distance);
        add(m, new String[]{"surface_paved_m", "surface_firm_unpaved_m", "surface_other_unpaved_m", "surface_other_m"}[surfaceClass], distance);
      }
      paved.accept(surfaceClass == 0, distance);
      firm.accept(surfaceClass == 1, distance);
      String smoothness = tags.get("smoothness");
      if (oneOf(smoothness, "excellent", "good", "intermediate", "bad", "very_bad", "horrible", "very_horrible", "impassable")) {
        add(m, "smoothness_known_m", distance);
        if (oneOf(smoothness, "bad", "very_bad", "horrible", "very_horrible", "impassable")) add(m, "poor_smoothness_m", distance);
      }
      int traffic = estimate(tags.get("estimated_traffic_class"), 7);
      if (traffic > 0) add(m, "traffic_estimate_known_m", distance);
      boolean high = traffic >= 4;
      boolean low = traffic > 0 && traffic <= 2;
      if (high) add(m, "high_traffic_estimate_m", distance);
      if (low) add(m, "low_traffic_estimate_m", distance);
      highTraffic.accept(high, distance);
      lowTraffic.accept(low, distance);
      estimatedExposure(m, tags, "forest", distance);
      estimatedExposure(m, tags, "river", distance);
      String highway = tags.get("highway");
      if (highway != null && !highway.isEmpty()) add(m, "highway_known_m", distance);
      if (oneOf(highway, "primary", "primary_link", "secondary", "secondary_link", "trunk", "trunk_link", "motorway", "motorway_link")) {
        add(m, "arterial_m", distance);
      }
      if ("steps".equals(highway)) add(m, "steps_m", distance);
      if ("dismount".equals(tags.get("bicycle"))) add(m, "explicit_dismount_m", distance);
      if (oneOf(tags.get("bicycle"), "no", "private")) add(m, "explicit_bicycle_no_private_m", distance);
      if (verified) {
        Map<String, String> node = tags(message.nodeKeyValues, parsed);
        if (!node.isEmpty()) add(m, "node_tagged_encounters", 1);
        if ("traffic_signals".equals(node.get("highway"))) add(m, "tagged_signal_encounters", 1);
        if ("stop".equals(node.get("highway"))) add(m, "tagged_stop_encounters", 1);
        if (node.containsKey("barrier") && !"no".equals(node.get("barrier"))) add(m, "tagged_barrier_encounters", 1);
        float seconds = message.measuredMovingSeconds();
        if (Float.isFinite(seconds) && seconds > 0) {
          add(m, "moving_time_covered_m", distance);
          add(m, "partial_model_moving_seconds", seconds);
        }
      }
      if (a.getSElev() != Short.MIN_VALUE && b.getSElev() != Short.MIN_VALUE) add(m, "elevation_known_m", distance);
      if (i >= 2 && !nodes.get(i - 2).positionEquals(a)) {
        OsmPathElement previous = nodes.get(i - 2);
        double first = CheapRuler.getScaledBearing(previous.getILon(), previous.getILat(), a.getILon(), a.getILat());
        double second = CheapRuler.getScaledBearing(a.getILon(), a.getILat(), b.getILon(), b.getILat());
        double change = Math.abs((second - first + 540) % 360 - 180);
        if (change >= 90) add(m, "sharp_geometry_bends", 1);
        if (change >= 150) add(m, "geometry_uturns", 1);
      }
    }
    double total = m.get("distance_m");
    m.put("stored_distance_m", (double) track.distance);
    m.put("distance_error_pct", requestedDistance > 0 ? 100 * Math.abs(total / requestedDistance - 1) : Double.NaN);
    m.put("surface_unknown_m", total - m.get("surface_known_m"));
    m.put("metadata_unknown_m", total - m.get("metadata_verified_m"));
    m.put("longest_paved_m", paved.longest);
    m.put("longest_firm_unpaved_m", firm.longest);
    m.put("longest_high_traffic_estimate_m", highTraffic.longest);
    m.put("longest_low_traffic_estimate_m", lowTraffic.longest);
    m.put("model_moving_seconds", total > 0 && Math.abs(total - m.get("moving_time_covered_m")) < 0.001
      ? m.get("partial_model_moving_seconds") : Double.NaN);
    measureElevation(nodes, distances, m);
    int[] reuse = LoopQualityMetrics.reuseStemSplit(nodes);
    m.put("shared_access_reuse_geometry_m", (double) reuse[0]);
    m.put("interior_reuse_geometry_m", (double) reuse[1]);
    for (String field : new String[]{"metadata_verified", "surface_known", "smoothness_known", "traffic_estimate_known",
      "forest_estimate_known", "river_estimate_known", "highway_known", "moving_time_covered", "elevation_known", "grade_window_covered"}) {
      m.put(field + "_pct", total > 0 ? 100 * m.get(field + "_m") / total : Double.NaN);
    }
    unavailableWithoutCoverage(m, "surface_known_m", "surface_paved_m", "surface_firm_unpaved_m",
      "surface_other_unpaved_m", "surface_other_m", "longest_paved_m", "longest_firm_unpaved_m");
    unavailableWithoutCoverage(m, "smoothness_known_m", "poor_smoothness_m");
    unavailableWithoutCoverage(m, "traffic_estimate_known_m", "high_traffic_estimate_m", "low_traffic_estimate_m",
      "longest_high_traffic_estimate_m", "longest_low_traffic_estimate_m");
    unavailableWithoutCoverage(m, "forest_estimate_known_m", "high_forest_estimate_m");
    unavailableWithoutCoverage(m, "river_estimate_known_m", "high_river_estimate_m");
    unavailableWithoutCoverage(m, "highway_known_m", "arterial_m", "steps_m");
    return new RouteRideAnalysis(m, System.nanoTime() - start);
  }

  // Non-overlapping 100 m windows with interpolated endpoints. Missing elevations break a run.
  private static void measureElevation(List<OsmPathElement> nodes, double[] distances, Map<String, Double> m) {
    double accumulated = 0;
    double windowStartElevation = 0;
    boolean active = false;
    for (int i = 1; i < nodes.size(); i++) {
      OsmPathElement a = nodes.get(i - 1);
      OsmPathElement b = nodes.get(i);
      double length = distances[i] - distances[i - 1];
      if (a.getSElev() == Short.MIN_VALUE || b.getSElev() == Short.MIN_VALUE) {
        active = false;
        accumulated = 0;
        continue;
      }
      if (length <= 0) continue;
      double elevation = a.getElev();
      if (!active) {
        windowStartElevation = elevation;
        active = true;
      }
      double used = 0;
      while (used < length) {
        double take = Math.min(length - used, ELEVATION_WINDOW_METERS - accumulated);
        used += take;
        accumulated += take;
        elevation = a.getElev() + (b.getElev() - a.getElev()) * used / length;
        if (accumulated >= ELEVATION_WINDOW_METERS - 1e-7) {
          double delta = elevation - windowStartElevation;
          double grade = 100 * delta / ELEVATION_WINDOW_METERS;
          add(m, "grade_window_covered_m", ELEVATION_WINDOW_METERS);
          add(m, "ascent_100m_m", Math.max(0, delta));
          add(m, "descent_100m_m", Math.max(0, -delta));
          if (grade >= STEEP_PERCENT) add(m, "steep_up_100m_m", ELEVATION_WINDOW_METERS);
          if (grade <= -STEEP_PERCENT) add(m, "steep_down_100m_m", ELEVATION_WINDOW_METERS);
          m.put("max_up_grade_100m_pct", Math.max(m.get("max_up_grade_100m_pct"), grade));
          m.put("max_down_grade_100m_pct", Math.max(m.get("max_down_grade_100m_pct"), -grade));
          double end = distances[i - 1] + used;
          double finalQuarterPart = Math.min(ELEVATION_WINDOW_METERS,
            Math.max(0, end - distances[distances.length - 1] * 0.75));
          add(m, "final_quarter_ascent_100m_m", Math.max(0, delta) * finalQuarterPart / ELEVATION_WINDOW_METERS);
          accumulated = 0;
          windowStartElevation = elevation;
        }
      }
    }
    if (m.get("grade_window_covered_m") == 0) {
      for (String name : new String[]{"ascent_100m_m", "descent_100m_m", "steep_up_100m_m", "steep_down_100m_m",
        "final_quarter_ascent_100m_m", "max_up_grade_100m_pct", "max_down_grade_100m_pct"}) m.put(name, Double.NaN);
    }
  }

  private static void unavailableWithoutCoverage(Map<String, Double> m, String coverage, String... measurements) {
    if (m.get(coverage) == 0) {
      for (String name : measurements) m.put(name, Double.NaN);
    }
  }

  private static void estimatedExposure(Map<String, Double> m, Map<String, String> tags, String type, double distance) {
    int value = estimate(tags.get("estimated_" + type + "_class"), 6);
    if (value > 0) {
      add(m, type + "_estimate_known_m", distance);
      if (value >= 4) add(m, "high_" + type + "_estimate_m", distance);
    }
  }

  private static int estimate(String value, int maximum) {
    if (value == null || value.length() != 1) return -1;
    int number = value.charAt(0) - '0';
    return number >= 1 && number <= maximum ? number : -1;
  }

  private static int surfaceClass(String surface) {
    if (surface == null || surface.isEmpty()) return -1;
    if (oneOf(surface, "asphalt", "paved", "concrete", "concrete:lanes", "concrete:plates", "paving_stones", "sett", "cobblestone")) return 0;
    if (oneOf(surface, "fine_gravel", "compacted")) return 1;
    if (oneOf(surface, "unpaved", "gravel", "ground", "dirt", "earth", "grass", "mud", "sand", "clay", "pebblestone")) return 2;
    if (oneOf(surface, "wood", "metal", "grass_paver", "woodchips", "rock", "stone")) return 3;
    return -1;
  }

  private static Map<String, String> tags(String raw, Map<String, Map<String, String>> cache) {
    if (raw == null || raw.isEmpty()) return Collections.emptyMap();
    return cache.computeIfAbsent(raw, value -> {
      Map<String, String> result = new HashMap<>();
      for (String token : value.split("\\s+")) {
        int separator = token.indexOf('=');
        if (separator > 0 && separator < token.length() - 1) result.put(token.substring(0, separator), token.substring(separator + 1));
      }
      return result;
    });
  }

  private static boolean oneOf(String value, String... options) {
    for (String option : options) if (option.equals(value)) return true;
    return false;
  }

  private static void add(Map<String, Double> m, String name, double amount) {
    m.put(name, m.get(name) + amount);
  }

  private static final class Run {
    double current;
    double longest;

    void accept(boolean matches, double distance) {
      current = matches ? current + distance : 0;
      longest = Math.max(longest, current);
    }
  }
}
