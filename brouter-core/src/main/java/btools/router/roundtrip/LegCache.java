package btools.router.roundtrip;

import java.util.HashMap;
import java.util.Map;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;

/**
 * Per-request, directional cache for raw routed legs (§4.7).
 * Keyed by directed edge (node1, node2 idFromPos) plus crosspoint for both endpoints.
 * Stores only complete successes, never timeouts or failures.
 * Returns defensive copies so cached tracks cannot be mutated by callers.
 */
public final class LegCache {

  /**
   * Immutable composite cache key for a directed leg between two matched waypoints.
   */
  public static final class LegKey {
    private final long fromNode1;
    private final long fromNode2;
    private final long fromCross;
    private final long toNode1;
    private final long toNode2;
    private final long toCross;

    public LegKey(MatchedWaypoint from, MatchedWaypoint to) {
      this.fromNode1 = (from != null && from.node1 != null) ? from.node1.getIdFromPos() : 0L;
      this.fromNode2 = (from != null && from.node2 != null) ? from.node2.getIdFromPos() : 0L;
      this.fromCross = (from != null && from.crosspoint != null) ? from.crosspoint.getIdFromPos() : 0L;
      this.toNode1 = (to != null && to.node1 != null) ? to.node1.getIdFromPos() : 0L;
      this.toNode2 = (to != null && to.node2 != null) ? to.node2.getIdFromPos() : 0L;
      this.toCross = (to != null && to.crosspoint != null) ? to.crosspoint.getIdFromPos() : 0L;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if (o == null || getClass() != o.getClass()) {
        return false;
      }
      LegKey other = (LegKey) o;
      return fromNode1 == other.fromNode1
          && fromNode2 == other.fromNode2
          && fromCross == other.fromCross
          && toNode1 == other.toNode1
          && toNode2 == other.toNode2
          && toCross == other.toCross;
    }

    @Override
    public int hashCode() {
      int h = 1;
      h = 31 * h + (int) (fromNode1 ^ (fromNode1 >>> 32));
      h = 31 * h + (int) (fromNode2 ^ (fromNode2 >>> 32));
      h = 31 * h + (int) (fromCross ^ (fromCross >>> 32));
      h = 31 * h + (int) (toNode1 ^ (toNode1 >>> 32));
      h = 31 * h + (int) (toNode2 ^ (toNode2 >>> 32));
      h = 31 * h + (int) (toCross ^ (toCross >>> 32));
      return h;
    }
  }

  private final Map<LegKey, OsmTrack> cache = new HashMap<>();
  private int hits = 0;
  private int misses = 0;
  private int seamMismatches = 0;

  /**
   * Look up a cached leg. Returns a deep copy of the cached track on hit, or null on miss.
   */
  public OsmTrack get(MatchedWaypoint from, MatchedWaypoint to) {
    if (from == null || to == null) {
      misses++;
      return null;
    }
    LegKey key = new LegKey(from, to);
    OsmTrack track = cache.get(key);
    if (track != null) {
      hits++;
      return copyTrack(track);
    }
    misses++;
    return null;
  }

  /**
   * Store a successful raw leg. Rejects null, empty, or single-node tracks.
   */
  public void put(MatchedWaypoint from, MatchedWaypoint to, OsmTrack track) {
    if (from == null || to == null || track == null || track.nodes == null || track.nodes.size() < 2) {
      return;
    }
    LegKey key = new LegKey(from, to);
    cache.put(key, copyTrack(track));
  }

  public boolean contains(MatchedWaypoint from, MatchedWaypoint to) {
    if (from == null || to == null) {
      return false;
    }
    return cache.containsKey(new LegKey(from, to));
  }

  public int size() {
    return cache.size();
  }

  public int getHits() {
    return hits;
  }

  public int getMisses() {
    return misses;
  }

  public double getHitRate() {
    int total = hits + misses;
    if (total == 0) {
      return 0.0;
    }
    return (double) hits / total;
  }

  public int getSeamMismatches() {
    return seamMismatches;
  }

  public void incrementSeamMismatches() {
    seamMismatches++;
  }

  /**
   * Check seam identity between two waypoint references representing the same junction (§4.7).
   * Verifies directed edge (node1, node2) and crosspoint identity.
   */
  public static boolean isSeamValid(MatchedWaypoint w1, MatchedWaypoint w2) {
    if (w1 == null || w2 == null) {
      return false;
    }
    long c1 = (w1.crosspoint != null) ? w1.crosspoint.getIdFromPos() : 0L;
    long c2 = (w2.crosspoint != null) ? w2.crosspoint.getIdFromPos() : 0L;
    long n1A = (w1.node1 != null) ? w1.node1.getIdFromPos() : 0L;
    long n1B = (w2.node1 != null) ? w2.node1.getIdFromPos() : 0L;
    long n2A = (w1.node2 != null) ? w1.node2.getIdFromPos() : 0L;
    long n2B = (w2.node2 != null) ? w2.node2.getIdFromPos() : 0L;
    return c1 == c2 && n1A == n1B && n2A == n2B;
  }

  /**
   * Check geometry continuity between the end of leg1 and start of leg2.
   */
  public static boolean isTrackSeamValid(OsmTrack leg1, OsmTrack leg2) {
    if (leg1 == null || leg2 == null || leg1.nodes == null || leg2.nodes == null
        || leg1.nodes.isEmpty() || leg2.nodes.isEmpty()) {
      return false;
    }
    OsmPathElement last = leg1.nodes.get(leg1.nodes.size() - 1);
    OsmPathElement first = leg2.nodes.get(0);
    return last.getILon() == first.getILon() && last.getILat() == first.getILat();
  }

  public static OsmTrack copyTrack(OsmTrack track) {
    if (track == null) {
      return null;
    }
    OsmTrack copy = new OsmTrack();
    copy.cost = track.cost;
    copy.distance = track.distance;
    if (track.nodes != null) {
      for (OsmPathElement pe : track.nodes) {
        copy.nodes.add(OsmPathElement.create(pe.getILon(), pe.getILat(), pe.getSElev(), null));
      }
    }
    return copy;
  }
}
