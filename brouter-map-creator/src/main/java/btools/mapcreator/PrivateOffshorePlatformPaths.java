package btools.mapcreator;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Narrow admission policy based only on explicit, unencoded OSM source tags. */
final class PrivateOffshorePlatformPaths {
  private static final int MAX_PLATFORM_NODES = 100_000;
  private static final Set<String> PATHS = Set.of("path", "footway", "steps", "cycleway", "pedestrian");
  private static final Set<String> ACCESS_KEYS = Set.of("access", "foot", "bicycle", "horse",
    "vehicle", "motor_vehicle", "motorcar", "motorcycle", "moped", "mofa", "hgv", "psv", "bus", "ski");
  private final Map<Long, Boolean> platformNodes = new HashMap<>();
  private long previousNodeId = Long.MIN_VALUE;
  private boolean sourceOrderCertain = true;
  private boolean waysStarted;
  private boolean excludedAny;

  void observe(NodeData node) throws IOException {
    // Strict ordering proves uniqueness without storing every regional node ID.
    // Unknown ordering may retain paths, but must never justify removing them.
    if (waysStarted || node.nid <= previousNodeId) {
      if (excludedAny) {
        throw new IOException("OFFSHORE_PLATFORM_SOURCE_ORDER: ambiguous node source after a path exclusion");
      }
      if (sourceOrderCertain) {
        System.out.println("private-offshore-platform-path exclusion disabled: ambiguous source node order");
      }
      sourceOrderCertain = false;
      platformNodes.clear();
    }
    previousNodeId = node.nid;
    if (!sourceOrderCertain) return;
    boolean offshore = "offshore_platform".equals(node.getTag("man_made"));
    boolean platform = offshore || ("platform".equals(node.getTag("location"))
      && "platform".equals(node.getTag("seamark:type")));
    if (!platform || hasAccessOverride(node.getTagsOrNull())) return;
    if (platformNodes.size() >= MAX_PLATFORM_NODES) {
      throw new IOException("OFFSHORE_PLATFORM_LIMIT: too many explicitly tagged platform nodes");
    }
    platformNodes.put(node.nid, offshore && "private".equals(node.getTag("access")));
  }

  boolean excludes(WayData way) {
    waysStarted = true;
    if (!sourceOrderCertain) return false;
    String highway = way.getTag("highway");
    if (highway == null || !PATHS.contains(highway) || way.nodes.size() < 2
      || hasAccessOverride(way.getTagsOrNull())) return false;
    for (String key : List.of("route", "railway", "waterway")) {
      String value = way.getTag(key);
      if (value != null && !"no".equals(value)) return false;
    }
    boolean privateOffshore = false;
    boolean distinct = false;
    long first = way.nodes.get(0);
    for (int i = 0; i < way.nodes.size(); i++) {
      long id = way.nodes.get(i);
      Boolean restricted = platformNodes.get(id);
      if (restricted == null) return false;
      privateOffshore |= restricted;
      distinct |= id != first;
    }
    boolean excluded = privateOffshore && distinct;
    excludedAny |= excluded;
    return excluded;
  }

  private static boolean hasAccessOverride(Map<String, String> tags) {
    for (Map.Entry<String, String> entry : tags.entrySet()) {
      String key = entry.getKey();
      int suffix = key.indexOf(':');
      String base = suffix < 0 ? key : key.substring(0, suffix);
      if (key.endsWith(":conditional") || (ACCESS_KEYS.contains(base)
        && (suffix >= 0 || (!"no".equals(entry.getValue()) && !"private".equals(entry.getValue()))))) return true;
    }
    return false;
  }
}
