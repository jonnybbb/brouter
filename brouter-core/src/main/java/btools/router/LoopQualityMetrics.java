package btools.router;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.util.CheapAngleMeter;
import btools.util.CheapRuler;

/**
 * Quality metrics for evaluating round-trip/loop routes.
 * Computes road reuse percentage, distance accuracy ratio,
 * direction adherence, continuity, compactness, cost and closure
 * from an OsmTrack.
 */
public final class LoopQualityMetrics {

  private final double roadReusePercent;
  private final double distanceRatio;
  private final double directionDeltaDegrees;
  private final int actualDistanceMeters;
  private final int requestedDistanceMeters;
  private final double continuityScore;
  private final int maxGapMeters;
  private final int totalGapMeters;
  private final double compactnessScore;
  private final double averageCostPerMeter;
  private final int closureDistanceMeters;
  /** Number of local out-and-back "spur" detours (see {@link #computeSpurInfo}). */
  private final int spurCount;
  /** Arc length (m) of the longest detected spur, 0 if none. */
  private final int worstSpurMeters;

  /**
   * Reconstruct a metrics instance from primitive fields. Used by the
   * test-time on-disk cache in {@code LoopQualityTest} to rehydrate results
   * across Gradle test forks without depending on Java serialisation. Not
   * part of the routing engine's normal write path — for new metrics
   * always use {@link #compute(OsmTrack, int, double)}.
   */
  public static LoopQualityMetrics fromFields(double roadReusePercent, double distanceRatio,
                                              double directionDeltaDegrees, int actualDistanceMeters,
                                              int requestedDistanceMeters, double continuityScore,
                                              int maxGapMeters, int totalGapMeters,
                                              double compactnessScore, double averageCostPerMeter,
                                              int closureDistanceMeters, int spurCount,
                                              int worstSpurMeters) {
    return new LoopQualityMetrics(roadReusePercent, distanceRatio, directionDeltaDegrees,
      actualDistanceMeters, requestedDistanceMeters, continuityScore, maxGapMeters, totalGapMeters,
      compactnessScore, averageCostPerMeter, closureDistanceMeters, spurCount, worstSpurMeters);
  }

  private LoopQualityMetrics(double roadReusePercent, double distanceRatio,
                             double directionDeltaDegrees, int actualDistanceMeters,
                             int requestedDistanceMeters, double continuityScore,
                             int maxGapMeters, int totalGapMeters,
                             double compactnessScore, double averageCostPerMeter,
                             int closureDistanceMeters, int spurCount, int worstSpurMeters) {
    this.roadReusePercent = roadReusePercent;
    this.distanceRatio = distanceRatio;
    this.directionDeltaDegrees = directionDeltaDegrees;
    this.actualDistanceMeters = actualDistanceMeters;
    this.requestedDistanceMeters = requestedDistanceMeters;
    this.continuityScore = continuityScore;
    this.maxGapMeters = maxGapMeters;
    this.totalGapMeters = totalGapMeters;
    this.compactnessScore = compactnessScore;
    this.averageCostPerMeter = averageCostPerMeter;
    this.closureDistanceMeters = closureDistanceMeters;
    this.spurCount = spurCount;
    this.worstSpurMeters = worstSpurMeters;
  }

  /**
   * Compute quality metrics for a round-trip track.
   *
   * @param track                  the routed track
   * @param requestedDistanceMeters the requested round-trip distance
   * @param requestedDirectionDeg  the requested start direction in degrees [0, 360)
   * @return computed metrics
   */
  public static LoopQualityMetrics compute(OsmTrack track, int requestedDistanceMeters,
                                           double requestedDirectionDeg) {
    List<OsmPathElement> nodes = track.nodes;

    double reusePercent = computeRoadReusePercent(nodes);
    double distRatio = (requestedDistanceMeters > 0)
      ? (double) track.distance / requestedDistanceMeters
      : 0.0;
    double dirDelta = computeDirectionDelta(nodes, requestedDirectionDeg);

    // Continuity: detect beeline gaps (consecutive points >100m apart)
    int[] gapInfo = computeGapInfo(nodes);
    int maxGap = gapInfo[0];
    int totalGap = gapInfo[1];
    double contScore = (track.distance > 0)
      ? Math.max(0.0, Math.min(1.0, 1.0 - (double) totalGap / track.distance))
      : 1.0;

    // Compactness: convex hull area vs ideal circle area
    double compact = computeCompactnessScore(nodes, track.distance);

    // Average cost per meter
    double avgCostPerMeter = (track.distance > 0)
      ? (double) track.cost / track.distance
      : 0.0;

    // Closure distance
    int closureDist = 0;
    if (nodes.size() >= 2) {
      OsmPathElement first = nodes.get(0);
      OsmPathElement last = nodes.get(nodes.size() - 1);
      closureDist = first.calcDistance(last);
    }

    int[] spurInfo = computeSpurInfo(nodes);

    return new LoopQualityMetrics(reusePercent, distRatio, dirDelta,
      track.distance, requestedDistanceMeters, contScore, maxGap, totalGap,
      compact, avgCostPerMeter, closureDist, spurInfo[0], spurInfo[1]);
  }

  /** Spur detector: a near-revisit must be this close spatially (m) to count. */
  static final double SPUR_EPS_METERS = 60.0;
  /** Lower arc-gap bound (m): closer revisits are local weaving, not an out-and-back. */
  static final double SPUR_MIN_ARC_GAP = 600.0;
  /** Upper arc-gap bound (m): keeps the detection LOCAL so the loop's own closure
   *  (start≈end over the full perimeter) and broad legitimate lobes are not flagged. */
  static final double SPUR_MAX_ARC_GAP = 6000.0;

  /**
   * Detect local out-and-back "spur" detours — the beeline back-and-forth the loop
   * planner occasionally emits (e.g. a waypoint reachable only via a dead-end corridor,
   * so the leg out and the leg back form a thin antenna).
   *
   * <p>A spur is a <em>local near-revisit</em>: the route returns within
   * {@link #SPUR_EPS_METERS} of a point it already passed between {@link #SPUR_MIN_ARC_GAP}
   * and {@link #SPUR_MAX_ARC_GAP} of riding earlier. The arc-gap band is the discriminator:
   * <ul>
   *   <li>the lower bound rejects switchback hairpins (adjacent arms are only a few hundred
   *       metres of road apart) and trivial local weaving;</li>
   *   <li>the upper bound keeps it LOCAL, so the loop's own closure and broad fat lobes
   *       (whose arms are far apart) are not flagged.</li>
   * </ul>
   * Validated against a labelled example (Berlin gravel_E iso_greedy): flags exactly the
   * eastern Wegendorf out-and-back and not the acceptable adjacent fat lobe. Reporting /
   * gate signal only — not part of the production selection formula.
   *
   * @return int[]{spurCount, worstSpurArcMeters}
   */
  static int[] computeSpurInfo(List<OsmPathElement> nodes) {
    int n = nodes.size();
    if (n < 4) return new int[]{0, 0};

    double[] cum = new double[n];
    for (int i = 1; i < n; i++) {
      cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    }

    int spurCount = 0;
    int worstArc = 0;
    int i = 0;
    while (i < n) {
      // Farthest forward node within the arc-gap band that the route returns near.
      int bestJ = -1;
      OsmPathElement a = nodes.get(i);
      for (int j = i + 1; j < n; j++) {
        double gap = cum[j] - cum[i];
        if (gap > SPUR_MAX_ARC_GAP) break;
        if (gap >= SPUR_MIN_ARC_GAP && a.calcDistance(nodes.get(j)) <= SPUR_EPS_METERS) {
          bestJ = j;
        }
      }
      if (bestJ >= 0) {
        spurCount++;
        int arc = (int) Math.round(cum[bestJ] - cum[i]);
        if (arc > worstArc) worstArc = arc;
        i = bestJ; // non-overlapping spans
      } else {
        i++;
      }
    }
    return new int[]{spurCount, worstArc};
  }

  /**
   * Generalised near-revisit scan: the spans where a track returns within
   * {@code epsMeters} of a point it already passed between {@code minArcMeters} and
   * {@code maxArcMeters} of riding earlier. This is the same primitive as
   * {@link #computeSpurInfo} but (a) parameterised and (b) returns the actual
   * {@code [i, j]} node-index spans rather than a count, so a caller can map a span
   * back to the leg(s) it falls in.
   *
   * <p>Used by the round-trip planner's elastic anti-reuse retry to decide whether a
   * shipped leg (or the assembled loop) contains a penalty-induced out-and-back or a
   * small sub-loop that rejoins near an earlier point. The detector is deliberately
   * liberal: it is only a <em>trigger</em> for a re-route-and-compare, so a false
   * positive costs one wasted re-route, never a wrong output.
   *
   * <p><b>Cap rationale:</b> {@link #computeSpurInfo} uses {@link #SPUR_MAX_ARC_GAP}
   * (6 km), which <em>misses</em> the motivating Basel→Ettingen out-and-back (≈7.3 km).
   * Callers targeting that class of detour pass a larger {@code maxArcMeters}
   * (≈10 km). Spans are non-overlapping (after a hit, the scan resumes at {@code j}).
   */
  public static List<int[]> nearRevisitSpans(List<OsmPathElement> nodes,
                                             double epsMeters, double minArcMeters,
                                             double maxArcMeters) {
    List<int[]> spans = new ArrayList<>();
    int n = nodes == null ? 0 : nodes.size();
    if (n < 4) return spans;

    double[] cum = new double[n];
    for (int i = 1; i < n; i++) {
      cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    }

    int i = 0;
    while (i < n) {
      int bestJ = -1;
      OsmPathElement a = nodes.get(i);
      for (int j = i + 1; j < n; j++) {
        double gap = cum[j] - cum[i];
        if (gap > maxArcMeters) break;
        if (gap >= minArcMeters && a.calcDistance(nodes.get(j)) <= epsMeters) {
          bestJ = j;
        }
      }
      if (bestJ >= 0) {
        spans.add(new int[]{i, bestJ});
        i = bestJ; // non-overlapping spans
      } else {
        i++;
      }
    }
    return spans;
  }

  /**
   * True if {@code nodes} contains at least one near-revisit span under the given
   * thresholds. Convenience wrapper over {@link #nearRevisitSpans}.
   */
  public static boolean hasNearRevisit(List<OsmPathElement> nodes,
                                       double epsMeters, double minArcMeters,
                                       double maxArcMeters) {
    return !nearRevisitSpans(nodes, epsMeters, minArcMeters, maxArcMeters).isEmpty();
  }

  /**
   * Length (m) of the longest single edge that is BOTH long and <em>null-way</em>
   * (no {@code wayKeyValues}) — the beeline fingerprint.
   *
   * <p>The discriminator is a <b>conjunction</b>, established empirically:
   * <ul>
   *   <li><b>long + tagged</b> = a real rural road (e.g. an 800m {@code highway=service}
   *       span with no intermediate OSM node) — legitimate;</li>
   *   <li><b>short + null</b> = harmless undetailed geometry (shape nodes survive, so it
   *       follows the real road shape) — pervasive on gravel because the metadata-detail
   *       gate is paved-only, and visually fine;</li>
   *   <li><b>long + null</b> = a beeline: a straight cut with no road shape AND no tags,
   *       which is what {@link RoutingEngine#retrackForDetail} ships when it cannot snap a
   *       straight expansion-guide onto the real road and falls back to the raw track.</li>
   * </ul>
   * Measuring the longest single null-way edge (not a contiguous sum) isolates the beeline
   * from the pervasive-but-harmless undetailed-road population. Reporting / gate signal only.
   */
  public static int maxSingleNullEdgeMeters(OsmTrack track) {
    if (track == null || track.nodes == null) return 0;
    List<OsmPathElement> nodes = track.nodes;
    int max = 0;
    for (int i = 1; i < nodes.size(); i++) {
      OsmPathElement curr = nodes.get(i);
      String tags = (curr.message != null) ? curr.message.wayKeyValues : null;
      boolean nullWay = (tags == null) || tags.isEmpty();
      if (nullWay) {
        int seg = nodes.get(i - 1).calcDistance(curr);
        if (seg > max) max = seg;
      }
    }
    return max;
  }

  /**
   * The beeline-in-dead-end fingerprint: the longest single null-way edge (m) that lies
   * <em>inside a detected spur span</em> (see {@link #computeSpurInfo}). This is the
   * <b>intersection</b> of the two signals that survives all the evidence:
   * <ul>
   *   <li>a long null-way edge ALONE over-fires — on gravel the metadata-detail gate is
   *       paved-only, so even real long rural roads ship undetailed (null + long);</li>
   *   <li>a near-revisit spur ALONE over-fires (~29%) — most out-and-backs are legitimate;</li>
   *   <li>their intersection is precise: a straight untagged cut <em>within</em> a near-revisit
   *       is the dead-end beeline — the route left the road, shot straight to a dead-end pinned
   *       waypoint and came back, which {@link RoutingEngine#retrackForDetail} could not snap to
   *       a real way (so it shipped raw + untagged). A real long road is traversed once, not
   *       inside a near-revisit, so it is excluded.</li>
   * </ul>
   * @return the longest single null-way edge within any spur span (m), 0 if none.
   */
  public static int beelineInSpurMeters(OsmTrack track) {
    if (track == null || track.nodes == null) return 0;
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    if (n < 4) return 0;
    double[] cum = new double[n];
    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    int worst = 0;
    int i = 0;
    while (i < n) {
      int bestJ = -1;
      OsmPathElement a = nodes.get(i);
      for (int j = i + 1; j < n; j++) {
        double gap = cum[j] - cum[i];
        if (gap > SPUR_MAX_ARC_GAP) break;
        if (gap >= SPUR_MIN_ARC_GAP && a.calcDistance(nodes.get(j)) <= SPUR_EPS_METERS) bestJ = j;
      }
      if (bestJ >= 0) {
        // longest single null-way edge within [i, bestJ]
        for (int k = i + 1; k <= bestJ; k++) {
          OsmPathElement curr = nodes.get(k);
          String tags = (curr.message != null) ? curr.message.wayKeyValues : null;
          if (tags == null || tags.isEmpty()) {
            int seg = nodes.get(k - 1).calcDistance(curr);
            if (seg > worst) worst = seg;
          }
        }
        i = bestJ;
      } else {
        i++;
      }
    }
    return worst;
  }

  /**
   * Compute the percentage of the track that reuses the same road edges.
   * An edge is identified by the unordered pair of consecutive node positions.
   * If an edge appears N times, (N-1) traversals count as reuse.
   */
  static double computeRoadReusePercent(List<OsmPathElement> nodes) {
    if (nodes.size() < 2) return 0.0;

    // Spatial corridor overlap (a parallel return on a different way) unioned
    // with edge-identity retrace, so this metric — consumed by RouteChoiceScore
    // (W_REUSE) and the composite score — sees same-corridor-back that edge
    // identity is blind to, and cannot drift from ReuseClassifier's gate which
    // uses the same primitive.
    boolean[] spatialOverlap = CorridorOverlapIndex.computeEdgeOverlap(nodes);

    Map<Long, int[]> edgeCounts = new HashMap<>();
    double totalDistance = 0;
    double reusedDistance = 0;

    for (int i = 1; i < nodes.size(); i++) {
      OsmPathElement a = nodes.get(i - 1);
      OsmPathElement b = nodes.get(i);
      int segDist = a.calcDistance(b);
      totalDistance += segDist;

      // Mixed undirected edge key (matches GreedyRoundTripPlanner.edgeKey). The
      // old `lo*31 + hi` form is a trivially-collidable linear combination of
      // packed (ilon<<32)|ilat ids, which could fold a genuinely-new edge into
      // an existing bucket and inflate the reuse percentage.
      long idA = a.getIdFromPos();
      long idB = b.getIdFromPos();
      long lo = Math.min(idA, idB);
      long hi = Math.max(idA, idB);
      long edgeKey = lo ^ (hi * 0x9E3779B97F4A7C15L);

      int[] entry = edgeCounts.get(edgeKey);
      boolean identityReuse = entry != null;
      if (entry == null) {
        edgeCounts.put(edgeKey, new int[]{1, segDist});
      } else {
        entry[0]++;
      }
      // Second/subsequent identity traversal OR a spatial parallel corridor.
      boolean spatial = (i - 1) < spatialOverlap.length && spatialOverlap[i - 1];
      if (identityReuse || spatial) {
        reusedDistance += segDist;
      }
    }

    return (totalDistance > 0) ? (reusedDistance / totalDistance) * 100.0 : 0.0;
  }

  /** Profile-ideal cost-per-meter (cycleway/tertiary on fastbike). Score plateaus at 1.0 below this. */
  private static final double IDEAL_COST_PER_METER = 1.5;
  /** Cost-per-meter at which a route is on roads the profile actively dislikes; score = 0. */
  private static final double ZERO_SCORE_COST_PER_METER = 4.0;

  /**
   * Score profile-match: how well the route's cost-per-meter matches a typical
   * profile-preferred road. 1.0 at cost/m ≤ {@link #IDEAL_COST_PER_METER}, decays
   * linearly to 0.0 at {@link #ZERO_SCORE_COST_PER_METER}+. Returns 0.0 if cost
   * data is missing.
   */
  static double computeCostMatchScore(double avgCostPerMeter) {
    if (avgCostPerMeter <= 0) return 0.0;
    return Math.max(0.0, Math.min(1.0,
      (ZERO_SCORE_COST_PER_METER - avgCostPerMeter) / (ZERO_SCORE_COST_PER_METER - IDEAL_COST_PER_METER)));
  }

  /**
   * Compute the angular delta between the requested direction and the
   * actual principal axis of the track. Uses the bearing from start to
   * the farthest point on the track.
   *
   * For round-trip loops, initial heading is ambiguous: clockwise vs
   * counter-clockwise traversals of the same loop differ by ~180° in
   * initial bearing. Measuring from start to farthest point captures
   * the loop's principal axis, which is invariant to traversal direction.
   * For open tracks, the farthest point is typically the endpoint, so
   * this remains equivalent to overall-heading.
   */
  static double computeDirectionDelta(List<OsmPathElement> nodes,
                                      double requestedDirectionDeg) {
    if (nodes.size() < 2) return 0.0;

    OsmPathElement first = nodes.get(0);
    OsmPathElement farthest = first;
    int maxDist = 0;
    for (OsmPathElement n : nodes) {
      int d = first.calcDistance(n);
      if (d > maxDist) {
        maxDist = d;
        farthest = n;
      }
    }
    if (maxDist == 0) return 0.0;

    double actualDirection = CheapAngleMeter.getDirection(
      first.getILon(), first.getILat(),
      farthest.getILon(), farthest.getILat());

    return CheapAngleMeter.getDifferenceFromDirection(requestedDirectionDeg, actualDirection);
  }

  /** A beeline shorter than this is a waypoint-snap connector, not a routing gap. */
  private static final int BEELINE_MIN_METERS = 500;

  /** Pseudo-tag BRouter writes on synthetic beeline links (see OsmPath.addAddionalPenalty). */
  private static final String BEELINE_MARKER = "direct_segment";
  private static final String FERRY_MARKER = "route=ferry";

  /**
   * Compute gap information for consecutive track points.
   * A "gap" is a segment that does not follow a real road:
   * <ol>
   *   <li>a ferry segment (carries the {@code route=ferry} tag), or</li>
   *   <li>a synthetic beeline/shortcut longer than {@code BEELINE_MIN_METERS}.</li>
   * </ol>
   * The discriminator is BRouter's own marker, not segment length or tag-absence:
   * a beeline link carries the pseudo-tag {@code direct_segment=N}
   * ({@link OsmPath#addAddionalPenalty}, {@code descriptionBitmap == null} branch),
   * while real road links carry their actual way tags (and only in detail mode).
   * Matching on the {@code direct_segment} marker therefore detects real beelines of
   * any length and never penalizes a legitimate long road segment (rural straight,
   * bridge, sparse-node highway), which never carries that marker. The length floor
   * skips trivial waypoint-snap connectors.
   *
   * @return int[]{maxGapMeters, totalGapMeters}
   */
  static int[] computeGapInfo(List<OsmPathElement> nodes) {
    int maxGap = 0;
    int totalGap = 0;

    for (int i = 1; i < nodes.size(); i++) {
      OsmPathElement prev = nodes.get(i - 1);
      OsmPathElement curr = nodes.get(i);
      int dist = prev.calcDistance(curr);

      String tags = (curr.message != null) ? curr.message.wayKeyValues : null;
      boolean isFerry = tags != null && tags.contains(FERRY_MARKER);
      boolean isBeeline = tags != null && tags.contains(BEELINE_MARKER);

      // Ferries are gaps at any length; beelines only once they exceed the
      // connector threshold (a short direct segment is just a waypoint snap).
      boolean isGap = isFerry || (isBeeline && dist > BEELINE_MIN_METERS);

      if (isGap) {
        totalGap += dist;
        if (dist > maxGap) {
          maxGap = dist;
        }
      }
    }
    return new int[]{maxGap, totalGap};
  }

  /**
   * Compute the compactness score of the track shape.
   * Uses the ratio of the convex hull area of track points to the area
   * of a circle with the same circumference as the track distance.
   * A perfect circle scores 1.0, an out-and-back line scores ~0.
   *
   * @param nodes    track points
   * @param distance total track distance in meters
   * @return compactness score clamped to [0, 1]
   */
  static double computeCompactnessScore(List<OsmPathElement> nodes, int distance) {
    if (nodes.size() < 3 || distance <= 0) return 0.0;

    // Subsample for performance (at most ~1000 points, minimum step of 1)
    List<double[]> points = new ArrayList<>();
    long centerILatSum = 0;
    int step = Math.max(1, nodes.size() / 1000);
    for (int i = 0; i < nodes.size(); i += step) {
      OsmPathElement n = nodes.get(i);
      centerILatSum += n.getILat();
      points.add(new double[]{n.getILon(), n.getILat()});
    }
    // Always include the last point
    OsmPathElement last = nodes.get(nodes.size() - 1);
    if (points.isEmpty() || points.get(points.size() - 1)[0] != last.getILon()
      || points.get(points.size() - 1)[1] != last.getILat()) {
      points.add(new double[]{last.getILon(), last.getILat()});
    }

    if (points.size() < 3) return 0.0;

    // Get meter scales at the center latitude
    int centerILat = (int) (centerILatSum / points.size());
    double[] kxky = CheapRuler.getLonLatToMeterScales(centerILat);
    double kx = kxky[0]; // ilon units to meters
    double ky = kxky[1]; // ilat units to meters

    // Convert to meter coordinates
    double[] xs = new double[points.size()];
    double[] ys = new double[points.size()];
    for (int i = 0; i < points.size(); i++) {
      xs[i] = points.get(i)[0] * kx;
      ys[i] = points.get(i)[1] * ky;
    }

    double hullArea = convexHullArea(xs, ys);

    // Area of a circle with circumference = distance
    // circumference = 2*PI*r => r = distance / (2*PI)
    // area = PI * r^2 = PI * (distance / (2*PI))^2 = distance^2 / (4*PI)
    double circleArea = (double) distance * distance / (4.0 * Math.PI);

    if (circleArea <= 0) return 0.0;
    return Math.max(0.0, Math.min(1.0, hullArea / circleArea));
  }

  /**
   * Compute the area of the convex hull of a set of 2D points using
   * Andrew's monotone chain algorithm. O(n log n).
   *
   * @param xs x coordinates
   * @param ys y coordinates
   * @return area of the convex hull
   */
  static double convexHullArea(double[] xs, double[] ys) {
    int n = xs.length;
    if (n < 3) return 0.0;

    // Create index array and sort by x, then y
    Integer[] idx = new Integer[n];
    for (int i = 0; i < n; i++) idx[i] = i;
    Arrays.sort(idx, new Comparator<>() {
      @Override
      public int compare(Integer a, Integer b) {
        int cx = Double.compare(xs[a], xs[b]);
        return cx != 0 ? cx : Double.compare(ys[a], ys[b]);
      }
    });

    // Build lower hull
    int[] hull = new int[2 * n];
    int k = 0;
    for (int i = 0; i < n; i++) {
      while (k >= 2 && cross(xs, ys, hull[k - 2], hull[k - 1], idx[i]) <= 0) {
        k--;
      }
      hull[k++] = idx[i];
    }
    // Build upper hull
    int lower = k + 1;
    for (int i = n - 2; i >= 0; i--) {
      while (k >= lower && cross(xs, ys, hull[k - 2], hull[k - 1], idx[i]) <= 0) {
        k--;
      }
      hull[k++] = idx[i];
    }
    // k-1 is the hull size (last point == first point)
    if (k < 4) return 0.0; // need at least 3 distinct hull vertices for non-zero area

    // Shoelace formula for area
    double area = 0;
    for (int i = 0; i < k - 1; i++) {
      int j = (i + 1) % (k - 1);
      area += xs[hull[i]] * ys[hull[j]];
      area -= xs[hull[j]] * ys[hull[i]];
    }
    return Math.abs(area) / 2.0;
  }

  /** Cross product of vectors OA and OB where O=hull[o], A=hull[a], B=idx[b] */
  private static double cross(double[] xs, double[] ys, int o, int a, int b) {
    return (xs[a] - xs[o]) * (ys[b] - ys[o])
      - (ys[a] - ys[o]) * (xs[b] - xs[o]);
  }

  /**
   * Compute a weighted composite quality score in [0, 1]. Higher is better.
   *
   * <p>Weights are aligned with what cyclists actually care about for a round-trip:
   * <ul>
   *   <li><b>25% distance</b> — total length close to target</li>
   *   <li><b>20% reuse</b> — no out-and-back / road retracing</li>
   *   <li><b>20% continuity</b> — no synthetic beelines, with maxGap penalty baked in</li>
   *   <li><b>20% profile match</b> — average cost/m matches profile-preferred roads</li>
   *   <li><b>10% compactness</b> — convex hull area vs ideal-circle area</li>
   *   <li><b>5% direction</b> — soft hint: user direction may not be reachable</li>
   * </ul>
   *
   * <p>Direction is intentionally low-weighted: in asymmetric terrain (valleys,
   * coastlines) the user's requested direction may not be achievable on real
   * roads, and the farthest-node-bearing signal degrades to ~random. Closure is
   * not scored — round-trips close by construction; bad closure should be a hard
   * gate, not a soft signal.
   *
   * <p><b>Reporting / test use only — NOT the production selection formula.</b>
   * Production candidate ranking uses {@link RouteChoiceScore#score}, which applies
   * different weights (it scores closure at 10% and adds shape penalties, splitting
   * continuity/cost differently). This method has no production callers; it backs
   * {@code toString()}, the loop-quality report generator, and integration-test
   * gates. Keep the two in mind when tuning: changing weights here does not change
   * which route production selects, and vice-versa.
   */
  public double compositeScore() {
    double distScore = 1.0 - Math.min(1.0, Math.abs(distanceRatio - 1.0) / 0.5);
    double reuseScore = 1.0 - Math.min(1.0, roadReusePercent / 50.0);
    double dirScore = 1.0 - Math.min(1.0, directionDeltaDegrees / 180.0);
    // continuity already penalises totalGap; weight maxGap on top so a single
    // 2km beeline scores worse than two 1km beelines of the same total length.
    double maxGapScore = 1.0 - Math.min(1.0, maxGapMeters / 1500.0);
    double continuityWithMaxGap = 0.75 * continuityScore + 0.25 * maxGapScore;
    double costMatchScore = computeCostMatchScore(averageCostPerMeter);

    return 0.25 * distScore
      + 0.20 * reuseScore
      + 0.20 * continuityWithMaxGap
      + 0.20 * costMatchScore
      + 0.10 * compactnessScore
      + 0.05 * dirScore;
  }

  public double getRoadReusePercent() {
    return roadReusePercent;
  }

  public double getDistanceRatio() {
    return distanceRatio;
  }

  public double getDirectionDeltaDegrees() {
    return directionDeltaDegrees;
  }

  public int getActualDistanceMeters() {
    return actualDistanceMeters;
  }

  public int getRequestedDistanceMeters() {
    return requestedDistanceMeters;
  }

  public double getContinuityScore() {
    return continuityScore;
  }

  public int getMaxGapMeters() {
    return maxGapMeters;
  }

  public int getTotalGapMeters() {
    return totalGapMeters;
  }

  public double getCompactnessScore() {
    return compactnessScore;
  }

  public double getAverageCostPerMeter() {
    return averageCostPerMeter;
  }

  public int getClosureDistanceMeters() {
    return closureDistanceMeters;
  }

  public int getSpurCount() {
    return spurCount;
  }

  public int getWorstSpurMeters() {
    return worstSpurMeters;
  }

  @Override
  public String toString() {
    return String.format(
      "LoopQualityMetrics[reuse=%.1f%%, distRatio=%.2f (%dm/%dm), dirDelta=%.1f°, " +
        "continuity=%.2f (maxGap=%dm, totalGap=%dm), compactness=%.2f, " +
        "cost/m=%.1f, closure=%dm, spurs=%d (worst=%dm), composite=%.2f]",
      roadReusePercent, distanceRatio, actualDistanceMeters, requestedDistanceMeters,
      directionDeltaDegrees, continuityScore, maxGapMeters, totalGapMeters,
      compactnessScore, averageCostPerMeter, closureDistanceMeters, spurCount, worstSpurMeters,
      compositeScore());
  }
}
