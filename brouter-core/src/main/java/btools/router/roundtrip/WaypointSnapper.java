package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.router.OsmNodeNamed;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;
import btools.util.CheapAngleMeter;
import btools.util.CheapRuler;

/**
 * Round-trip waypoint snapping, validation, and reachability probing,
 * extracted from RoutingEngine: road/profile-aware snaps for generated and
 * user vias, the ferry/hostile usability rules, the densified-via arc bulges
 * and their pinned-bulge repair, the circle-path validation pass, and the
 * bearing-grid reachability probes that feed ISOCHRONE and FAST placement.
 * Reaches the engine only through {@link RoundTripEngineOps}.
 */
public final class WaypointSnapper {

  private final RoundTripEngineOps ops;

  public WaypointSnapper(RoundTripEngineOps ops) {
    this.ops = ops;
  }

  /**
   * Ferry/profile usability of a retained probe match — the same rule every
   * snap-validation site applies (selection owns it; placement re-checks).
   */
  public FastPlacementOps.SnapUsability snapUsability(MatchedWaypoint m) {
    if (!isRoadSnap(m)) {
      return FastPlacementOps.SnapUsability.FERRY_LIKE;
    }
    if (snapCandidateCostFactor(m) > snapRejectCostFactorForProfile()) {
      return FastPlacementOps.SnapUsability.PROFILE_HOSTILE;
    }
    return FastPlacementOps.SnapUsability.OK;
  }

  // A snap whose matched edge is longer than this is ferry-like (sparse nodes
  // spanning km over water), not a road edge — skip it when snapping waypoints.
  private static final int FERRY_LIKE_EDGE_METERS = 1500;

  // Display/log label stamped on generated arc-densification "bulge" waypoints.
  // It is NOT load-bearing: post-route spur cleanup targets generated points via
  // the typed MatchedWaypoint.generated flag, not this name.
  private static final String DENSIFIED_VIA_WAYPOINT_NAME = "dvia";

  /**
   * Hard ceiling for the active profile's costfactor at a waypoint snap. A
   * candidate's BEST road is REJECTED entirely if its costfactor exceeds this —
   * better to drop the waypoint (route around the area) than commit to a
   * profile-hostile road that the user will hit as a surprise mid-tour. A
   * fastbike snap to a track grade-5 (costfactor 30) is far worse for the rider
   * than skipping that waypoint and routing elsewhere.
   *
   * <p>Per-profile thresholds — fastbike rejects high-costfactor (≥5) tracks/unpaved;
   * gravel allows moderate roughness; mtb is lenient because trails are its target.
   */
  private static final double SNAP_REJECT_COSTFACTOR_FASTBIKE = 5.0;

  private static final double SNAP_REJECT_COSTFACTOR_GRAVEL = 8.0;

  private static final double SNAP_REJECT_COSTFACTOR_DEFAULT = 10.0;

  /**
   * Via-arc densification. Insert one generated "bulge" waypoint per
   * perimeter leg of the anchor cycle [start, via1, ..., viaN, (back to start)],
   * offset outward from the anchor centroid so each leg follows the loop perimeter
   * instead of cutting the chord. User anchors stay hard constraints.
   *
   * <p>Safety: a bulge is kept ONLY if it snaps to a profile-compatible way (cost-factor
   * at or below the profile's snap-reject threshold, and not a ferry-like edge). Where
   * "outward" is only profile-hostile or off-network (e.g. a road bike in sparse/alpine
   * terrain), the bulge is dropped and that leg reverts to its baseline shortest-path
   * form — so densification never forces the route onto a hostile road, which was the
   * lost-route failure mode of the unguarded version.
   * Returns the densified, ordered waypoint chain (without the closing start copy).
   */
  public List<OsmNodeNamed> densifyViaArcs(List<OsmNodeNamed> anchors, double searchRadius, double snapDist) {
    long sumLon = 0;
    long sumLat = 0;
    for (OsmNodeNamed a : anchors) {
      sumLon += a.ilon;
      sumLat += a.ilat;
    }
    int cLon = (int) (sumLon / anchors.size());
    int cLat = (int) (sumLat / anchors.size());

    double alpha = ops.routingContext().explicitViaDensifyAlpha;
    int n = anchors.size();

    // 1. Build one candidate bulge per leg (null when degenerate, e.g. a 1-via
    //    out-and-back where the leg midpoint coincides with the centroid).
    //    candIndexForLeg maps each leg to its slot in the compacted candidates
    //    list (or -1), so acceptance can be tracked by index rather than by
    //    node identity/coordinate.
    OsmNodeNamed[] bulges = new OsmNodeNamed[n];
    int[] candIndexForLeg = new int[n];
    List<OsmNodeNamed> candidates = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      OsmNodeNamed bulge = makeArcBulge(anchors.get(i), anchors.get((i + 1) % n), cLon, cLat, alpha, searchRadius);
      bulges[i] = bulge;
      if (bulge != null) {
        candIndexForLeg[i] = candidates.size();
        candidates.add(bulge);
      } else {
        candIndexForLeg[i] = -1;
      }
    }

    // 2. Profile-aware snap (batched): accepted[c] flags candidate c BY INDEX, so
    //    two bulges that snap to the same road node are never conflated (which a
    //    coordinate-keyed set would, since OsmNode equals/hashCode key on lon/lat).
    boolean[] accepted = snapBulgesProfileAware(candidates, snapDist);

    // 3. Assemble the densified chain, keeping only accepted bulges.
    List<OsmNodeNamed> out = new ArrayList<>();
    int added = 0;
    int dropped = 0;
    for (int i = 0; i < n; i++) {
      out.add(anchors.get(i));
      if (candIndexForLeg[i] >= 0 && accepted[candIndexForLeg[i]]) {
        out.add(bulges[i]);
        added++;
      } else {
        dropped++;
      }
    }
    ops.logInfo("explicit-via arc densification: +" + added + " arc point(s), " + dropped
      + " dropped (degenerate/hostile/off-network), alpha=" + alpha);
    return out;
  }

  /**
   * A snap match is usable only if it resolved to a real road edge: both endpoints
   * present and the matched edge short enough to be a road rather than a ferry-like
   * span (see {@link #FERRY_LIKE_EDGE_METERS}).
   */
  private static boolean isRoadSnap(MatchedWaypoint m) {
    return m.crosspoint != null && m.node1 != null && m.node2 != null
      && m.node1.calcDistance(m.node2) <= FERRY_LIKE_EDGE_METERS;
  }

  /**
   * Shared batch road-snap primitive: reset the node cache, wrap each point in a
   * {@link MatchedWaypoint}, and run the matcher once. Returns the matched list
   * (same size and order as {@code points}; each entry's crosspoint/node1/node2
   * populated where a road was found within {@code maxSnapDist}), or {@code null}
   * if the matcher threw (callers treat that as "nothing matched"). Used by the
   * profile-aware start/bulge snaps. {@link #snapWaypointsToRoad} keeps its own
   * copy of this pattern because it additionally mutates the input waypoints and
   * returns per-point booleans rather than the MatchedWaypoint objects.
   */
  private List<MatchedWaypoint> batchMatchToRoads(List<OsmNode> points, double maxSnapDist, String nameTag) {
    ops.resetCache(false);
    List<MatchedWaypoint> mwps = new ArrayList<>(points.size());
    for (OsmNode p : points) {
      MatchedWaypoint mwp = new MatchedWaypoint();
      mwp.waypoint = new OsmNode(p.ilon, p.ilat);
      mwp.name = nameTag;
      mwps.add(mwp);
    }
    try {
      ops.matchWaypointsToNodes(mwps, maxSnapDist);
    } catch (Exception e) {
      return null;
    }
    return mwps;
  }

  /**
   * Profile-aware point match for planner-generated round-trip vias. The plain
   * nearest-road match commits the via to whatever way is closest — for
   * fastbike that can be a grade2 track in a pocket whose only escape is more
   * track, which pins a junk bulge into the loop (see
   * {@link #repairViaPinnedBulges}). When (and only when) that nearest snap is
   * profile-hostile ({@link #VIA_SNAP_HOSTILE_COSTFACTOR}), probe rings
   * ({@link #VIA_SNAP_PROBE_RINGS}) are evaluated and the via moves to the
   * best-scoring road ({@code costFactor*1000 + distance}) if it improves the
   * cost factor by {@link #VIA_SNAP_MIN_IMPROVEMENT}; otherwise the plain
   * nearest match is returned unchanged. Returns null when nothing matches.
   */
  public MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist) {
    // Loop-scale relocation bound (see VIA_RELOCATION_LOOP_FRACTION). When no
    // round-trip scale is known (0), fall back to the absolute maxSnapDist.
    double relocationCap = ops.roundTripSearchRadius() > 0
      ? Math.min(maxSnapDist, VIA_RELOCATION_LOOP_FRACTION * ops.roundTripSearchRadius())
      : maxSnapDist;
    OsmNode orig = new OsmNode(ilon, ilat);
    // Match the plain point FIRST and probe the rings only when it is
    // actually hostile. The rings exist purely for the hostile-snap escape;
    // matching all 17 points up front paid ~16 extra road matches per routed
    // candidate (~240 candidate snaps per plan) on the common non-hostile
    // path. Skipping the orig entry in the ring loop below is
    // behavior-identical: its score (origCf*1000 + snapDist) can never beat
    // the incumbent-initialized bestScore of origCf*1000.
    List<OsmNode> plainPoint = new ArrayList<>(1);
    plainPoint.add(new OsmNode(ilon, ilat));
    List<MatchedWaypoint> plainMatch = batchMatchToRoads(plainPoint, maxSnapDist, name);
    if (plainMatch == null) return null;

    MatchedWaypoint origMatch = isRoadSnap(plainMatch.get(0)) ? plainMatch.get(0) : null;
    double origCf = origMatch != null ? snapCandidateCostFactor(origMatch) : Double.MAX_VALUE;

    MatchedWaypoint best = origMatch;
    double bestCf = origCf;
    if (origMatch == null || origCf >= VIA_SNAP_HOSTILE_COSTFACTOR) {
      List<OsmNode> points = new ArrayList<>();
      for (double ring : VIA_SNAP_PROBE_RINGS) {
        if (ring > relocationCap) continue; // ring would only yield over-cap candidates
        for (double bearing = 0; bearing < 360; bearing += 45) {
          int[] p = CheapRuler.destination(ilon, ilat, ring, bearing);
          points.add(new OsmNode(p[0], p[1]));
        }
      }
      List<MatchedWaypoint> mwps = points.isEmpty()
        ? new ArrayList<>() : batchMatchToRoads(points, maxSnapDist, name);
      if (mwps == null) mwps = new ArrayList<>(); // probe failure: keep the plain match
      // KNOWN INCONSISTENCY, kept for now: the incumbent is scored WITHOUT its
      // snap-distance term while the probe alternatives pay costFactor*1000 +
      // distance (the sibling start-snapper scores the incumbent with the full
      // formula). The omission gives the hostile original a stickiness bonus
      // equal to its own snap distance, which can block a relocation the
      // improvement guard below would keep. Risk of the consistent formula:
      // it loosens incumbent stickiness while relocation distance has no bound
      // relative to loop scale (rings {300,700} + maxSnapDist are absolute, so
      // a small loop can have a via pulled >1km sideways). Change this only
      // together with a loop-scale relocation bound, validated on a GREEN
      // matrix baseline (a 2026-06-11 attempt was reverted during a red
      // baseline; its regression evidence was misattributed and is unproven).
      double bestScore = origMatch != null ? origCf * 1000.0 : Double.MAX_VALUE;
      for (MatchedWaypoint m : mwps) {
        if (!isRoadSnap(m)) continue;
        // Hard loop-scale bound on the actual displacement (ring filtering
        // above is necessary but not sufficient: a ring point can snap to a
        // road far beyond the cap). The incumbent (m == origMatch) is exempt —
        // staying put is never a relocation.
        if (m != origMatch && orig.calcDistance(m.crosspoint) > relocationCap) continue;
        double costFactor = snapCandidateCostFactor(m);
        double score = costFactor * 1000.0 + orig.calcDistance(m.crosspoint);
        if (score < bestScore) {
          bestScore = score;
          best = m;
          bestCf = costFactor;
        }
      }
      // Accept the relocation only when it is a substantial improvement, not a
      // lateral move among comparably hostile roads.
      if (origMatch != null && best != origMatch && bestCf > origCf * VIA_SNAP_MIN_IMPROVEMENT) {
        best = origMatch;
      }
    }
    if (best != null) {
      best.name = name;
      // Re-anchor the waypoint on the original position so downstream radius /
      // catching-range checks measure from the planner's intended target.
      best.waypoint = orig;
      best.radius = orig.calcDistance(best.crosspoint);
    }
    return best;
  }

  /**
   * Build a single arc-following waypoint for the leg {@code a -> b}: the chord
   * midpoint pushed outward (away from the loop centroid) by {@code alpha} of the
   * chord length. Returns null for legs too short to bulge or a degenerate centroid.
   */
  private OsmNodeNamed makeArcBulge(OsmNodeNamed a, OsmNodeNamed b, int cLon, int cLat,
                                    double alpha, double searchRadius) {
    int midLon = (int) (((long) a.ilon + b.ilon) / 2);
    int midLat = (int) (((long) a.ilat + b.ilat) / 2);
    double chord = CheapRuler.distance(a.ilon, a.ilat, b.ilon, b.ilat);
    if (chord < 50) {
      return null;
    }
    if (midLon == cLon && midLat == cLat) {
      return null; // midpoint coincides with the loop centroid — no outward direction
    }
    // Outward bearing = centroid -> midpoint (latitude-scaled compass degrees, [0,360)).
    double bearing = CheapRuler.getScaledBearing(cLon, cLat, midLon, midLat);
    double offset = Math.min(alpha * chord, searchRadius);
    int[] p = CheapRuler.destination(midLon, midLat, offset, bearing);
    OsmNodeNamed bulge = new OsmNodeNamed(new OsmNode(p[0], p[1]));
    bulge.name = DENSIFIED_VIA_WAYPOINT_NAME; // display/log label only
    bulge.generated = true; // the load-bearing "this is a generated bulge" signal
    return bulge;
  }

  /**
   * Filter round-trip waypoints that matched to bad positions.
   * Removes intermediate waypoints (named "rt*") where:
   * - The snap distance (waypoint to crosspoint) exceeds maxSnapDistance
   * - Consecutive matched positions are too close together
   * Never removes the first or last waypoint (start/end of loop).
   * Preserves at least one intermediate waypoint.
   */
  public void filterRoundTripWaypoints(List<MatchedWaypoint> waypoints) {
    double maxSnapDistance = ops.roundTripSearchRadius() * 0.5;
    double minWaypointDistance = ops.roundTripSearchRadius() / 10.0;
    // Max edge length between node1 and node2 for a valid road match.
    // Ferry routes have sparse nodes with multi-km edges; road segments are typically < 1km.
    int maxSegmentLength = 1500;

    // Count intermediate round-trip waypoints
    int rtCount = 0;
    for (int i = 1; i < waypoints.size() - 1; i++) {
      if (waypoints.get(i).name != null && waypoints.get(i).name.startsWith("rt")) {
        rtCount++;
      }
    }

    // Remove waypoints matched to ferry-like segments (very long edges)
    for (int i = waypoints.size() - 2; i >= 1; i--) {
      MatchedWaypoint mwp = waypoints.get(i);
      if (mwp.name == null || !mwp.name.startsWith("rt")) continue;
      if (rtCount <= 1) break;

      if (mwp.node1 != null && mwp.node2 != null) {
        int segLen = mwp.node1.calcDistance(mwp.node2);
        if (segLen > maxSegmentLength) {
          ops.logInfo("filterRoundTrip: removing " + mwp.name + " matched to long segment (" + segLen + "m, likely ferry)");
          waypoints.remove(i);
          rtCount--;
        }
      }
    }

    // Remove waypoints that snapped too far
    for (int i = waypoints.size() - 2; i >= 1; i--) {
      MatchedWaypoint mwp = waypoints.get(i);
      if (mwp.name == null || !mwp.name.startsWith("rt")) continue;
      if (rtCount <= 1) break; // preserve at least one intermediate waypoint

      if (mwp.radius > maxSnapDistance) {
        ops.logInfo("filterRoundTrip: removing " + mwp.name + " snap=" + (int) mwp.radius + "m > max=" + (int) maxSnapDistance + "m");
        waypoints.remove(i);
        rtCount--;
      }
    }

    if (rtCount <= 1) return;

    // Remove consecutive round-trip waypoints that matched too close together
    for (int i = waypoints.size() - 2; i >= 2; i--) {
      if (rtCount <= 1) break;
      MatchedWaypoint curr = waypoints.get(i);
      MatchedWaypoint prev = waypoints.get(i - 1);
      if (curr.name == null || !curr.name.startsWith("rt")) continue;
      if (prev.name == null || !prev.name.startsWith("rt")) continue;
      // crosspoint is nullable (MatchedWaypoint.crosspoint); a not-yet-matched
      // rt waypoint would NPE here. Skip the too-close test rather than crash.
      if (curr.crosspoint == null || prev.crosspoint == null) continue;

      double dist = curr.crosspoint.calcDistance(prev.crosspoint);
      if (dist < minWaypointDistance) {
        ops.logInfo("filterRoundTrip: removing " + curr.name + " too close to " + prev.name + " dist=" + (int) dist + "m");
        waypoints.remove(i);
        rtCount--;
      }
    }
  }

  /**
   * Snap intermediate roundtrip waypoints to the nearest intersection node.
   * When a waypoint is matched to the middle of an edge (crosspoint between
   * node1 and node2), routing to that mid-edge point can create small
   * out-and-back tails. By moving the crosspoint to the closer of node1/node2
   * (a real intersection), we avoid these tails entirely.
   * This is analogous to GraphHopper's getBaseNode() approach.
   * Only applied to generated roundtrip waypoints (rt*), not user waypoints
   * or the start/end points.
   */
  public void snapToIntersection(List<MatchedWaypoint> waypoints) {
    for (int i = 1; i < waypoints.size() - 1; i++) {
      MatchedWaypoint mwp = waypoints.get(i);
      if (mwp.name == null || !mwp.name.startsWith("rt")) continue;
      if (mwp.node1 == null || mwp.node2 == null || mwp.crosspoint == null) continue;

      int distToNode1 = mwp.crosspoint.calcDistance(mwp.node1);
      int distToNode2 = mwp.crosspoint.calcDistance(mwp.node2);
      OsmNode closerNode = distToNode1 <= distToNode2 ? mwp.node1 : mwp.node2;

      ops.logInfo("snapToIntersection: " + mwp.name + " moved crosspoint "
        + (distToNode1 <= distToNode2 ? distToNode1 : distToNode2) + "m to nearest intersection");
      mwp.crosspoint = new OsmNode(closerNode.ilon, closerNode.ilat);
    }
  }

  /**
   * Snap a single waypoint to the nearest road within {@code maxSnapDist}.
   * Returns true and rewrites the waypoint coordinates to the matched
   * crosspoint when a match is found; returns false otherwise (waypoint left
   * untouched). Used by round-trip code paths to ensure generated points are
   * close enough to a road that final matchWaypointsToNodes (250m catching
   * range) does not fall back to dynamic beeline insertion.
   */
  public boolean snapWaypointToRoad(OsmNodeNamed wp, double maxSnapDist, String logTag) {
    return snapWaypointsToRoad(Collections.singletonList(wp), maxSnapDist, logTag).get(0);
  }

  /**
   * Per-profile snap-rejection threshold from the {@code SNAP_REJECT_COSTFACTOR_*}
   * constants. Resolves the active profile by filename so a fastbike user
   * doesn't end up snapped to a grade-5 track even when the engine has run a
   * different profile previously on the same JVM.
   */
  private double snapRejectCostFactorForProfile() {
    String name = profileNameForLog();
    if (name == null) return SNAP_REJECT_COSTFACTOR_DEFAULT;
    String lower = name.toLowerCase();
    if (lower.contains("fastbike")) return SNAP_REJECT_COSTFACTOR_FASTBIKE;
    if (lower.contains("gravel")) return SNAP_REJECT_COSTFACTOR_GRAVEL;
    return SNAP_REJECT_COSTFACTOR_DEFAULT;
  }

  /** Filename of the active profile (lower-cased, no extension) — for logging + threshold lookup. */
  private String profileNameForLog() {
    String path = ops.routingContext() == null ? null : ops.routingContext().localFunction;
    if (path == null) return null;
    int slash = Math.max(path.lastIndexOf('/'), path.lastIndexOf('\\'));
    int dot = path.lastIndexOf('.');
    int start = slash + 1;
    int end = dot > start ? dot : path.length();
    return path.substring(start, end);
  }

  /**
   * Evaluate the active profile's costfactor for the road segment a candidate
   * waypoint matched against (used by round-trip via-snapping to prefer
   * profile-liked roads). Returns {@code 1.0f} on any failure to resolve the
   * tags — that's the neutral value (preserves the geometric-only score).
   *
   * <p>The score comes straight from {@code mwp.wayDescription}: the waypoint
   * matcher records the matched way's tag bytes at match time (the trailing
   * way-description arg of {@code WaypointMatcher.start}). This method, reached
   * only on the round-trip via-snap path, evaluates those tags through the
   * profile's way context. {@code wayDescription} is null only when the match
   * carried no way tags, in which case the neutral {@code 1.0f} is returned.
   */
  private float snapCandidateCostFactor(MatchedWaypoint mwp) {
    // Evaluate the tag description the waypoint matcher captured from the
    // matched way. The previous implementation walked node1's links to find
    // the way — but the matcher's node1/node2 are coordinate-only copies that
    // are never hollow and carry no links, so that walk silently returned the
    // 1.0 fallback for every match and all "profile-aware" snap scoring was
    // inert. The matcher now records the way description at match time.
    if (mwp == null || mwp.wayDescription == null) return 1.0f;
    try {
      ops.routingContext().expctxWay.evaluate(false, mwp.wayDescription);
      return ops.routingContext().expctxWay.getCostfactor();
    } catch (RuntimeException ignored) {
      return 1.0f;
    }
  }

  /**
   * One probe skeleton for both grids — the legacy 24-bearing × 3-radii sweep
   * and the optimized FAST 2-radii grid — so fixes to the snap rule, matcher
   * error handling, or start-probe convention cannot drift between the paths
   * (the two used to be near-verbatim forks kept alive for A/B comparison).
   */
  private ProbeResult probeDirections(OsmNodeNamed start, double searchRadius,
                                      double[] bearings, double[] distFactors,
                                      boolean retainMatches, String logLabel) {
    ops.resetCache(false);
    double maxSnapDist = Math.min(searchRadius * 0.3, 2000);
    int probesPerDirection = distFactors.length;

    List<MatchedWaypoint> allProbes = new ArrayList<>();
    // Include the start point itself to ensure its segment is loaded.
    MatchedWaypoint startProbe = new MatchedWaypoint();
    startProbe.waypoint = new OsmNode(start.ilon, start.ilat);
    startProbe.name = "probe_start";
    allProbes.add(startProbe);

    for (int d = 0; d < bearings.length; d++) {
      for (double df : distFactors) {
        int[] pos = CheapRuler.destination(start.ilon, start.ilat, searchRadius * df, bearings[d]);
        MatchedWaypoint mwp = new MatchedWaypoint();
        mwp.waypoint = new OsmNode(pos[0], pos[1]);
        mwp.name = "probe_" + d + "_" + (int) (df * 100);
        allProbes.add(mwp);
      }
    }

    try {
      ops.matchWaypointsToNodes(allProbes, maxSnapDist);
    } catch (Exception e) {
      ops.logInfo("reachability probe failed: " + e.getMessage());
      return null;
    }

    int probeOffset = 1; // start probe is at index 0
    double[] viable = new double[bearings.length];
    int viableCount = 0;
    List<ProbeDirection> scored = new ArrayList<>();
    double snapRejectThreshold = retainMatches
      ? snapRejectCostFactorForProfile() : Double.POSITIVE_INFINITY;
    for (int d = 0; d < bearings.length; d++) {
      ProbeDirection pd = selectProbeDirection(bearings[d], allProbes,
        probeOffset + d * probesPerDirection, probesPerDirection,
        retainMatches, maxSnapDist, snapRejectThreshold);
      if (pd == null) continue;
      viable[viableCount++] = bearings[d];
      scored.add(pd);
    }

    ops.logInfo(logLabel + ": " + viableCount + "/" + bearings.length + " bearings snapped");
    if (viableCount == 0) return null;
    // allProbes.get(0) is the matched start (has node1/node2/crosspoint when
    // the start is on a road) — used by the islanded-via guard.
    return new ProbeResult(Arrays.copyOf(viable, viableCount), scored,
      retainMatches ? allProbes.get(0) : null);
  }

  /**
   * Summarize one bearing's probe matches. The optimized FAST path counts and
   * retains only road/profile-compatible matches, so a rejected nearer edge
   * cannot hide a usable match from the other probe radius. The legacy probe
   * keeps its historical raw-snap count and does not retain a match.
   */
  ProbeDirection selectProbeDirection(double direction, List<MatchedWaypoint> probes,
                                      int firstIndex, int count, boolean retainMatches,
                                      double maxSnapDist, double snapRejectThreshold) {
    int successCount = 0;
    MatchedWaypoint best = null;
    for (int i = firstIndex; i < firstIndex + count; i++) {
      MatchedWaypoint mwp = probes.get(i);
      if (mwp.crosspoint == null || mwp.radius > maxSnapDist) continue;
      if (retainMatches && (!isRoadSnap(mwp)
          || snapCandidateCostFactor(mwp) > snapRejectThreshold)) {
        continue;
      }
      successCount++;
      if (retainMatches && (best == null || mwp.radius < best.radius)) {
        best = mwp;
      }
    }
    return successCount == 0 ? null
      : new ProbeDirection(direction, successCount, best);
  }

  /**
   * FAST-tier reachability probe (optimization ideas 1 + 2). Trims the grid to
   * 12 bearings × 2 radii (vs the legacy 24 × 3) AND retains the best road match
   * per direction, so {@code FastWaypointPlanner#placeWaypointsFromProbeMatches}
   * can reuse those already-snapped nodes as vias — replacing both the geometric
   * placement and the ~21-candidate-per-via re-matching in
   * {@link #validateAndAdjustWaypoints}.
   */
  public ProbeResult probeReachableDirectionsFast(OsmNodeNamed start, double searchRadius, double[] bearings) {
    // 2 radii per bearing (vs the legacy 3) + best-match retention so
    // placeWaypointsFromProbeMatches can reuse the snapped nodes as vias.
    return probeDirections(start, searchRadius, bearings,
      new double[]{0.85, 1.15}, true, "reachability probe (fast)");
  }

  public ProbeResult probeReachableDirections(OsmNodeNamed start, double searchRadius) {
    // Legacy sweep: 24 bearings every 15°, radii {0.7, 1.0, 1.3}·R — the
    // ISOCHRONE path and the -Droundtrip.fast.optimized=false A/B baseline.
    return probeDirections(start, searchRadius, PlacementGeometry.fullCircleBearings(24),
      new double[]{0.7, 1.0, 1.3}, false, "reachability probe");
  }

  /**
   * Snap the start (and closing) waypoint to the nearest road.
   * Round-trip waypoints use the user's raw click position which may be
   * far from any road. If the distance exceeds waypointCatchingRange (250m),
   * the routing engine inserts a straight-line beeline instead of a road segment.
   * This method matches the start to the road network and updates its position.
   */
  public void snapStartToRoad(List<OsmNodeNamed> waypoints, double searchRadius) {
    if (waypoints.size() < 2) return;
    OsmNodeNamed start = waypoints.get(0);
    if (snapWaypointToRoad(start, Math.min(searchRadius * 0.3, 2000), "snapStartToRoad")) {
      // Also snap the closing waypoint (last in list) which mirrors the start
      OsmNodeNamed closing = waypoints.get(waypoints.size() - 1);
      if ("to".equals(closing.name)) {
        closing.ilon = start.ilon;
        closing.ilat = start.ilat;
      }
    }
  }

  /**
   * Batch variant of {@link #snapWaypointToRoad}. One nodesCache reset and one
   * matchWaypointsToNodes call for the whole list — avoids reallocating the
   * cache per waypoint when many points need snapping (e.g. user via points).
   * Returns a parallel list of booleans indicating which waypoints matched.
   */
  public List<Boolean> snapWaypointsToRoad(List<OsmNodeNamed> wps, double maxSnapDist, String logTag) {
    ops.resetCache(false);
    List<MatchedWaypoint> mwpList = new ArrayList<>(wps.size());
    for (OsmNodeNamed wp : wps) {
      MatchedWaypoint mwp = new MatchedWaypoint();
      mwp.waypoint = new OsmNode(wp.ilon, wp.ilat);
      mwp.name = (wp.name == null ? "wp" : wp.name) + "_snap";
      mwpList.add(mwp);
    }
    try {
      ops.matchWaypointsToNodes(mwpList, maxSnapDist);
    } catch (Exception e) {
      ops.logInfo(logTag + ": match failed, leaving " + wps.size() + " waypoint(s) unsnapped: " + e.getMessage());
      List<Boolean> all = new ArrayList<>(wps.size());
      for (int i = 0; i < wps.size(); i++) all.add(false);
      return all;
    }
    List<Boolean> matched = new ArrayList<>(wps.size());
    for (int i = 0; i < wps.size(); i++) {
      OsmNodeNamed wp = wps.get(i);
      MatchedWaypoint mwp = mwpList.get(i);
      if (mwp.crosspoint == null) {
        matched.add(false);
        continue;
      }
      int snapDist = wp.calcDistance(mwp.crosspoint);
      if (snapDist > 0) {
        ops.logInfo(logTag + ": moved " + (wp.name == null ? "wp" : wp.name) + " "
          + snapDist + "m to nearest road");
        wp.ilon = mwp.crosspoint.getILon();
        wp.ilat = mwp.crosspoint.getILat();
      }
      matched.add(true);
    }
    return matched;
  }

  /**
   * Detect and repair wide-mouth bulges pinned at generated round-trip vias.
   *
   * <p>The planner can place a via in a pocket whose only escape is over roads
   * the profile penalizes (Basel reference: via on a grade2 field track 750m
   * west of a parallel asphalt cycle route — the loop pays ~11× the through
   * cost to touch it). The resulting detour has no ≤50m pinch point, so
   * {@code RoutingEngine#removeMicroDetours}' teardrop band never sees it; and it encloses
   * real area (petal compactness ~0.9), so a thin-only filter can't separate
   * it from scenic petals. What does separate it is price: the span rides
   * roads at ≥{@link #BULGE_MIN_COST_FACTOR}× the track's average cost/m.
   *
   * <p>Because the mouth is wide, the span cannot simply be deleted (that
   * would leave an off-road beeline). Instead the mouth is re-routed: a local
   * {@code RoutingEngine#findTrack} between the two mouth nodes, accepted only when the
   * connector meaningfully shortens the span
   * ({@link #BULGE_CONNECTOR_MAX_ARC_FRACTION}, {@link #BULGE_MIN_SAVED_M})
   * and undercuts its cost/m ({@link #BULGE_CONNECTOR_COST_ADVANTAGE}) — when
   * the network offers no better connector, the bulge was forced, and it stays.
   */
  public void repairViaPinnedBulges(OsmTrack track, List<MatchedWaypoint> waypoints) {
    List<OsmPathElement> nodes = track.nodes;
    if (nodes == null || nodes.size() < 10 || waypoints == null || waypoints.size() < 3) {
      return;
    }
    long totalDist = 0;
    for (int k = 1; k < nodes.size(); k++) {
      totalDist += nodes.get(k - 1).calcDistance(nodes.get(k));
    }
    int arcCap = (int) Math.min(VIA_TEARDROP_MAX_ARC_M, VIA_TEARDROP_MAX_ARC_FRAC * totalDist);
    double trackCostPerM = spanCostPerMeter(nodes, 0, nodes.size() - 1);

    for (int wi = 1; wi < waypoints.size() - 1; wi++) {
      MatchedWaypoint via = waypoints.get(wi);
      if (!isGeneratedRoundTripWaypoint(via)) continue;
      int v = via.indexInTrack;
      if (v <= 0 || v >= nodes.size() - 1) continue;
      int lo = Math.max(0, waypoints.get(wi - 1).indexInTrack + 1);
      int hi = Math.min(nodes.size() - 1, waypoints.get(wi + 1).indexInTrack - 1);
      int[] span = findViaPinnedBulgeSpan(nodes, v, lo, hi, arcCap);
      if (span == null) continue;
      double spanCpm = spanCostPerMeter(nodes, span[0], span[1]);
      if (trackCostPerM > 0 && spanCpm < BULGE_MIN_COST_FACTOR * trackCostPerM) {
        ops.logInfo("repairViaPinnedBulges: " + via.name + " span " + span[0] + "-" + span[1]
          + " kept as petal (span " + spanCpm + " vs track " + trackCostPerM + " cost/m)");
        continue; // priced like the rest of the loop — a petal, not an artifact
      }
      OsmPathElement ni = nodes.get(span[0]);
      OsmPathElement nj = nodes.get(span[1]);
      double arc = 0;
      for (int k = span[0] + 1; k <= span[1]; k++) {
        arc += nodes.get(k - 1).calcDistance(nodes.get(k));
      }
      double crowFly = CheapRuler.distance(ni.getILon(), ni.getILat(), nj.getILon(), nj.getILat());

      List<OsmNode> mouthPts = new ArrayList<>();
      mouthPts.add(new OsmNode(ni.getILon(), ni.getILat()));
      mouthPts.add(new OsmNode(nj.getILon(), nj.getILat()));
      List<MatchedWaypoint> mouth = batchMatchToRoads(mouthPts, 100.0, "bulge_repair");
      if (mouth == null || !isRoadSnap(mouth.get(0)) || !isRoadSnap(mouth.get(1))) {
        ops.logInfo("repairViaPinnedBulges: " + via.name + " mouth snap failed");
        continue;
      }

      OsmTrack connector = null;
      try {
        connector = ops.findTrackUnguided("bulge-repair", mouth.get(0), mouth.get(1));
      } catch (RuntimeException e) {
        ops.logInfo("repairViaPinnedBulges: connector routing failed (" + e.getMessage() + ")");
      }
      if (connector == null || connector.nodes == null || connector.nodes.size() < 2) {
        ops.logInfo("repairViaPinnedBulges: " + via.name + " no connector route");
        continue;
      }
      double connCpm = connector.distance > 0
        ? (double) connector.cost / connector.distance : Double.MAX_VALUE;
      if (connector.distance > arc * BULGE_CONNECTOR_MAX_ARC_FRACTION
          || arc - connector.distance < BULGE_MIN_SAVED_M
          || connCpm * BULGE_CONNECTOR_COST_ADVANTAGE > spanCpm) {
        ops.logInfo(String.format(Locale.US,
          "repairViaPinnedBulges: %s connector rejected (dist=%d crowFly=%.0f arc=%.0f connCpm=%.2f spanCpm=%.2f)",
          via.name, connector.distance, crowFly, arc, connCpm, spanCpm));
        continue;
      }

      // Splice: replace the span interior with the connector's nodes, trimming
      // connector endpoints that duplicate the mouth nodes.
      List<OsmPathElement> conn = connector.nodes;
      int cs = 0;
      int ce = conn.size();
      while (cs < ce && conn.get(cs).calcDistance(ni) <= 2) cs++;
      while (ce > cs && conn.get(ce - 1).calcDistance(nj) <= 2) ce--;
      List<OsmPathElement> interior = new ArrayList<>(conn.subList(cs, ce));
      int removedNodes = span[1] - span[0] - 1;
      // Crossing guard: the connector is routed without sight of the rest of
      // the loop, so it can cut transversely across the outbound or return —
      // trading a fat bulge for a user-visible self-crossing (measured: AUTO
      // fastbike crossings +49% before this guard). Splice the node list
      // first, compare self-intersections, and revert if the count rose;
      // waypoint indices are only adjusted after acceptance.
      int crossingsBefore = RoundTripQualityGate.countSelfIntersections(track);
      List<OsmPathElement> oldInterior = new ArrayList<>(nodes.subList(span[0] + 1, span[1]));
      nodes.subList(span[0] + 1, span[1]).clear();
      nodes.addAll(span[0] + 1, interior);
      int crossingsAfter = RoundTripQualityGate.countSelfIntersections(track);
      if (crossingsAfter > crossingsBefore) {
        nodes.subList(span[0] + 1, span[0] + 1 + interior.size()).clear();
        nodes.addAll(span[0] + 1, oldInterior);
        ops.logInfo("repairViaPinnedBulges: " + via.name + " connector rejected (would add "
          + (crossingsAfter - crossingsBefore) + " self-crossing(s))");
        continue;
      }
      adjustWaypointIndices(waypoints, span[0], span[1] - 1, removedNodes - interior.size());

      ops.logInfo(String.format(Locale.US,
        "repairViaPinnedBulges: at %s replaced %.0fm bulge (mouth %.0fm, span %.2f cost/m vs track %.2f) with %dm connector (%.2f cost/m)",
        via.name, arc, crowFly, spanCpm, trackCostPerM, connector.distance, connCpm));
    }
  }

  /**
   * Validate round-trip waypoints against segment data before routing.
   * For each generated waypoint, checks if profile-compatible ways exist
   * nearby by matching against the routing segments. Waypoints that fall
   * in unreachable areas (water, no compatible ways) are relocated by
   * trying alternative positions at varied angles and distances from the
   * start point. Unmatched waypoints are removed if enough remain.
   *
   * The matching is profile-aware: during segment decoding, only ways
   * with accessType >= 2 (compatible with the current routing profile)
   * are considered.
   */
  public void validateAndAdjustWaypoints(List<OsmNodeNamed> waypoints, double searchRadius) {
    ops.resetCache(false);
    OsmNodeNamed start = waypoints.get(0);
    double maxSnapDist = Math.min(searchRadius * 0.3, 2000);

    double[] angleOffsets = {0, -15, 15, -30, 30};
    double[] distFactors = {1.0, 0.8, 1.2, 0.6, 1.4};

    int intermediateCount = waypoints.size() - 2;
    List<List<MatchedWaypoint>> candidateGroups = new ArrayList<>();
    List<MatchedWaypoint> allCandidates = new ArrayList<>();

    for (int i = 1; i <= intermediateCount; i++) {
      OsmNodeNamed wp = waypoints.get(i);
      double bearing = CheapRuler.getScaledBearing(start.ilon, start.ilat, wp.ilon, wp.ilat);
      double dist = CheapRuler.distance(start.ilon, start.ilat, wp.ilon, wp.ilat);

      List<MatchedWaypoint> group = new ArrayList<>();

      for (double da : angleOffsets) {
        for (double df : distFactors) {
          // skip extreme combinations to limit candidate count
          if (Math.abs(da) > 15 && Math.abs(df - 1.0) > 0.25) continue;

          int ilon, ilat;
          if (da == 0 && df == 1.0) {
            ilon = wp.ilon;
            ilat = wp.ilat;
          } else {
            int[] pos = CheapRuler.destination(start.ilon, start.ilat, dist * df, bearing + da);
            ilon = pos[0];
            ilat = pos[1];
          }

          MatchedWaypoint mwp = new MatchedWaypoint();
          mwp.waypoint = new OsmNode(ilon, ilat);
          mwp.name = wp.name + "_c" + group.size();
          group.add(mwp);
          allCandidates.add(mwp);
        }
      }

      candidateGroups.add(group);
    }

    // Match all candidates at once — profile-aware via segment decoding.
    // Guard like the sibling callers (batchMatchToRoads, snapWaypointsToRoad): a
    // cache/segment-decode failure must not abort the whole round trip. On error,
    // leave the waypoints at their generated positions and let the downstream
    // final matchWaypointsToNodes handle them, rather than throwing out of here
    // into doRoundTrip's catch and failing the request outright.
    try {
      ops.matchWaypointsToNodes(allCandidates, maxSnapDist);
    } catch (Exception e) {
      ops.logInfo("validateAndAdjustWaypoints: candidate match failed ("
        + e.getClass().getSimpleName() + "), keeping generated waypoint positions");
      return;
    }

    // Pick the best candidate for each waypoint or remove it.
    // Use direction-aware scoring: prefer roads perpendicular to the travel
    // direction (the route naturally crosses them) over parallel roads
    // (which often require a detour to reach).
    int minWaypoints = PlacementGeometry.MIN_ROUNDTRIP_VIAS;
    int remaining = intermediateCount;

    for (int i = intermediateCount; i >= 1; i--) {
      List<MatchedWaypoint> group = candidateGroups.get(i - 1);

      // Travel bearing: direction from previous to next waypoint
      OsmNodeNamed prev = waypoints.get(i - 1);
      OsmNodeNamed next = waypoints.get(i + 1);
      double travelBearing = CheapAngleMeter.getDirection(prev.ilon, prev.ilat, next.ilon, next.ilat);

      MatchedWaypoint best = null;
      double bestScore = Double.MAX_VALUE;
      double bestCostFactor = 1.0;
      double snapRejectThreshold = snapRejectCostFactorForProfile();
      for (MatchedWaypoint mwp : group) {
        if (mwp.crosspoint == null) continue;

        // node1/node2 are dereferenced just below for the road bearing; skip any
        // match missing either endpoint rather than NPE.
        if (mwp.node1 == null || mwp.node2 == null) continue;

        // Skip matches to ferry-like segments (very long edges between node1/node2).
        // Ferry routes have sparse nodes spanning several km over water; road edges are < 1km.
        if (mwp.node1.calcDistance(mwp.node2) > FERRY_LIKE_EDGE_METERS) {
          continue;
        }

        double snapDist = mwp.radius;

        // Road bearing at snap point
        double roadBearing = CheapAngleMeter.getDirection(
          mwp.node1.ilon, mwp.node1.ilat, mwp.node2.ilon, mwp.node2.ilat);

        // Angle between road and travel direction (0-90°, road is bidirectional)
        double angleDiff = CheapAngleMeter.getDifferenceFromDirection(roadBearing, travelBearing);
        if (angleDiff > 90) angleDiff = 180 - angleDiff;

        // parallelFactor: 1.0 for parallel, 0.0 for perpendicular
        double parallelFactor = 1.0 - angleDiff / 90.0;

        // Penalize parallel roads: effective snap distance increases up to 50%.
        // This favors roads the route would naturally cross without detour.
        double score = snapDist * (1.0 + 0.5 * parallelFactor * parallelFactor);

        // Profile-aware penalty: prefer roads the active profile actually likes.
        // Without this, fastbike happily snaps to a 50m forest track when there's
        // a 200m paved road just past it — and then the routing engine is forced
        // through the track because the waypoint is committed.
        double costFactor = snapCandidateCostFactor(mwp);
        score *= (1.0 + SNAP_PROFILE_COST_WEIGHT * (costFactor - 1.0));

        if (score < bestScore) {
          best = mwp;
          bestScore = score;
          bestCostFactor = costFactor;
        }
      }
      // Reject the snap entirely if the BEST road we could find is too profile-
      // hostile (e.g. fastbike forced onto a grade-5 track). Better to drop this
      // waypoint and route around than ship the user onto a road their profile
      // would have actively avoided — that's the surprise-mid-tour pain.
      if (best != null && bestCostFactor > snapRejectThreshold) {
        ops.logInfo("validateWaypoints: rejecting profile-hostile snap for "
          + waypoints.get(i).name + " (costfactor=" + String.format("%.1f", bestCostFactor)
          + " > " + snapRejectThreshold + " for " + profileNameForLog() + ")");
        best = null;
      }

      OsmNodeNamed wp = waypoints.get(i);
      if (best != null && best.radius <= maxSnapDist) {
        // Use crosspoint, not waypoint: keeps the point within the 250m
        // catching range used by final matchWaypointsToNodes (avoids beeline).
        if (wp.ilon != best.crosspoint.getILon() || wp.ilat != best.crosspoint.getILat()) {
          ops.logInfo("validateWaypoints: relocated " + wp.name + " snap=" + (int) best.radius + "m");
          wp.ilon = best.crosspoint.getILon();
          wp.ilat = best.crosspoint.getILat();
        }
      } else if (remaining > minWaypoints) {
        ops.logInfo("validateWaypoints: removing unreachable " + wp.name
          + " (best=" + (best == null ? "none" : (int) best.radius + "m") + ")");
        waypoints.remove(i);
        remaining--;
      } else {
        ops.logInfo("validateWaypoints: keeping marginal " + wp.name + " (min waypoint count reached)");
      }
    }
    ops.logInfo("validateWaypoints: " + remaining + "/" + intermediateCount + " waypoints validated");
  }

  /**
   * Relocation triggers only when the plain nearest snap is meaningfully
   * profile-hostile. Below this cost factor the original snap is fine and the
   * cached graph-native leg (if any) stays valid — relocating on marginal wins
   * (cf 1.2 → 1.1) was measured to fire on most candidates, paying an extra
   * leg Dijkstra each (~6× shard runtime) and churning routes for no artifact
   * being prevented.
   */
  private static final double VIA_SNAP_HOSTILE_COSTFACTOR = 2.0;

  /**
   * Loop-scale relocation bound: a via may relocate at most this fraction of
   * the round-trip search radius away from the planner's intended position.
   * The probe rings ({@link #VIA_SNAP_PROBE_RINGS}) and {@code maxSnapDist}
   * are absolute, so without this bound a small loop can have a via pulled
   * sideways by a distance comparable to the whole loop radius — measured on
   * the r=1000m fixture (after the 8802e75b metadata-consistency fix made
   * snap cost factors real): the GREEDY dir90 loop flipped clean →
   * 19%-retraced OUT_AND_BACK. At production radii (≥8000m) the cap reaches
   * {@code maxSnapDist} and behaviour is unchanged — the junk-pocket bulges
   * this relocation prevents are a long-loop phenomenon.
   */
  private static final double VIA_RELOCATION_LOOP_FRACTION = 0.25;

  /**
   * Batch-snap candidate bulge points, accepting a bulge ONLY if it matches a
   * profile-compatible road and is not a ferry-like long edge. Accepted bulges are
   * moved to their crosspoint. Returns a boolean[] parallel to {@code bulges}:
   * {@code accepted[i]} is true iff {@code bulges.get(i)} was accepted — indexed,
   * not coordinate-keyed, so two bulges snapping to the same node stay distinct.
   *
   * <p>The cost-factor ceiling is {@code RoutingContext#explicitViaDensifyMaxCostFactor},
   * intentionally stricter than the lenient user-snap reject threshold in
   * {@link #validateAndAdjustWaypoints}: a bulge is an optional nicety, so a
   * fastbike facing only tracks drops it and reverts to its baseline leg instead
   * of being forced onto a hostile detour.
   */
  private boolean[] snapBulgesProfileAware(List<OsmNodeNamed> bulges, double maxSnapDist) {
    boolean[] accepted = new boolean[bulges.size()];
    if (bulges.isEmpty()) {
      return accepted;
    }
    List<OsmNode> points = new ArrayList<>(bulges.size());
    for (OsmNodeNamed bulge : bulges) {
      points.add(new OsmNode(bulge.ilon, bulge.ilat));
    }
    List<MatchedWaypoint> mwps = batchMatchToRoads(points, maxSnapDist, "dvia_snap");
    if (mwps == null) {
      return accepted; // match failure — all legs revert to baseline
    }
    double rejectThreshold = ops.routingContext().explicitViaDensifyMaxCostFactor;
    for (int i = 0; i < bulges.size(); i++) {
      MatchedWaypoint mwp = mwps.get(i);
      if (!isRoadSnap(mwp)) {
        continue;
      }
      if (snapCandidateCostFactor(mwp) > rejectThreshold) {
        continue; // not a near-ideal road for this profile — drop, leg reverts to baseline
      }
      OsmNodeNamed bulge = bulges.get(i);
      bulge.ilon = mwp.crosspoint.getILon();
      bulge.ilat = mwp.crosspoint.getILat();
      accepted[i] = true;
    }
    return accepted;
  }

  /**
   * Weight on the profile costfactor in waypoint-snap scoring (line ~1218 of this
   * file). Combined with the geometric score as {@code geom × (1 + W × (costfactor - 1))}.
   *
   * <p>Empirical calibration with W=0.5:
   * <ul>
   *   <li>50m grade5 track (fastbike costfactor 30) score = 50 × (1 + 0.5 × 29) = 775,
   *       loses to 200m tertiary (costfactor 1, score 200) — correct: fastbike won't
   *       snap to a forest track when there's a paved road nearby.</li>
   *   <li>50m cycleway (costfactor 1.3) score = 50 × 1.15 = 57.5 still beats a
   *       200m tertiary (score 200) — correct: short snap to a cyclist-preferred
   *       road remains the better pick.</li>
   * </ul>
   * Falls back to 1.0 (no penalty) when the link can't be resolved.
   */
  private static final double SNAP_PROFILE_COST_WEIGHT = 0.5;

  /**
   * Probe-ring radii for {@link #profileAwareMatchPoint}. The inner ring covers
   * "good road just past the nearest track" cases; the outer ring covers
   * pocket escapes (the Basel reference via sat 750m from the parallel asphalt
   * cycle route, with nothing but grade2 track in between).
   */
  private static final double[] VIA_SNAP_PROBE_RINGS = {300, 700};

  /** ... and only when the alternative is substantially better than the
   *  original (multiplicative improvement), not a lateral move. */
  private static final double VIA_SNAP_MIN_IMPROVEMENT = 0.6;

  /**
   * Junk filter: the span's average cost/m must be at least this multiple of
   * the whole track's average. A deliberate scenic petal rides roads the
   * profile likes (span cost/m ≈ track average) and is kept; only overpriced
   * detours — the profile got dragged onto roads it penalizes — qualify.
   * Deliberately loose (the Basel reference span measures 1.38× because
   * {@link #spanCostPerMeter} skips the leg-reset edge); the connector
   * cost-advantage check below is the decisive guard.
   */
  private static final double BULGE_MIN_COST_FACTOR = 1.2;

  /**
   * Repair acceptance: the connector must be at most this fraction of the
   * bulge arc. This is an efficiency criterion, not a straightness one — a
   * winding connector on profile-friendly roads still beats a junk-road bulge
   * (Basel via3: 839m connector at 1.44 cost/m vs 3231m span at 3.92). When
   * the shortest connector is comparable to the bulge itself, the bulge was
   * network-forced, not via-pinned — keep it.
   */
  private static final double BULGE_CONNECTOR_MAX_ARC_FRACTION = 0.6;

  /** Repair acceptance: minimum arc length the splice must save. */
  private static final int BULGE_MIN_SAVED_M = 400;

  /** Repair acceptance: connector cost/m × this must still undercut the span's
   *  cost/m, so the repair is a genuine quality win, not a lateral move. */
  private static final double BULGE_CONNECTOR_COST_ADVANTAGE = 1.3;

  /**
   * Via-pinned teardrop band: a detour pinned at a generated via (the planner
   * placed a waypoint in a one-way-out pocket; the anti-reuse penalty then
   * shapes the escape into a thin offset loop instead of an exact retrace the
   * symmetric spur remover could strip) may be removed up to this arc length —
   * well beyond the plain micro-detour cap.
   */
  public static final int VIA_TEARDROP_MAX_ARC_M = 4000;

  /** ... bounded relative to the whole track, so short loops never lose a large share. */
  public static final double VIA_TEARDROP_MAX_ARC_FRAC = 0.15;

  /**
   * Profile-aware start snap for explicit-via round trips. The plain nearest-road snap
   * ({@link #snapWaypointToRoad}) matches the nearest <em>accessible</em> way, which for a
   * road bike can be a track right next to a paved road — starting the loop on unpaved.
   * This evaluates the original click plus a small ring of nearby positions and moves the
   * start to the most profile-compatible road (lowest cost-factor), tie-broken by proximity,
   * within {@code maxSnapDist}. The original position is always a candidate, so the start is
   * never moved unless a clearly more compatible road is genuinely nearby — and never moved
   * further than {@code maxSnapDist}.
   */
  public void snapStartProfileAware(OsmNodeNamed start, double maxSnapDist) {
    int origLon = start.ilon;
    int origLat = start.ilat;
    OsmNode orig = new OsmNode(origLon, origLat);

    List<OsmNode> points = new ArrayList<>();
    points.add(new OsmNode(origLon, origLat)); // index 0 = original = plain nearest-road snap
    double ring = Math.min(maxSnapDist, 300);
    for (double bearing = 0; bearing < 360; bearing += 45) {
      int[] p = CheapRuler.destination(origLon, origLat, ring, bearing);
      points.add(new OsmNode(p[0], p[1]));
    }

    List<MatchedWaypoint> mwps = batchMatchToRoads(points, maxSnapDist, "start_snap");
    if (mwps == null) {
      return; // leave start as-is; downstream matchWaypointsToNodes handles it
    }

    MatchedWaypoint best = null;
    double bestScore = Double.MAX_VALUE;
    double bestCostFactor = 1.0;
    for (MatchedWaypoint m : mwps) {
      if (!isRoadSnap(m)) {
        continue;
      }
      int distFromOrig = orig.calcDistance(m.crosspoint);
      if (distFromOrig > maxSnapDist) {
        continue; // keep the relocation bounded
      }
      double costFactor = snapCandidateCostFactor(m);
      // Strongly prefer a low-cost (profile-liked) road; tie-break toward the original click.
      double score = costFactor * 1000.0 + distFromOrig;
      if (score < bestScore) {
        bestScore = score;
        best = m;
        bestCostFactor = costFactor;
      }
    }

    if (best != null) {
      int moved = orig.calcDistance(best.crosspoint);
      if (moved > 0) {
        ops.logInfo("snapStart: profile-aware start snap (" + moved + "m, costfactor "
          + String.format("%.1f", bestCostFactor) + ")");
      }
      start.ilon = best.crosspoint.getILon();
      start.ilat = best.crosspoint.getILat();
    }
  }

  /**
   * Whether this waypoint is engine-generated (greedy planner vias carry the
   * {@code generated} flag; the WAYPOINT algorithm's points use the "rt" name
   * prefix). User-supplied waypoints match neither — they express route
   * intent and must never be repaired away.
   */
  public static boolean isGeneratedRoundTripWaypoint(MatchedWaypoint wp) {
    return wp.generated || (wp.name != null && wp.name.startsWith("rt"));
  }

  /**
   * Adjust waypoint track indices after removing a range of nodes.
   * Waypoints beyond the removed range are shifted back; waypoints
   * inside the range are clamped to rangeStart.
   */
  public static void adjustWaypointIndices(List<MatchedWaypoint> waypoints, int rangeStart, int rangeEnd, int removeCount) {
    for (MatchedWaypoint mwp : waypoints) {
      if (mwp.indexInTrack > rangeEnd) {
        mwp.indexInTrack -= removeCount;
      } else if (mwp.indexInTrack > rangeStart) {
        mwp.indexInTrack = rangeStart;
      }
    }
  }

  /**
   * Find a low-progress span pinned at a generated via: the pair (i, j) with
   * i &lt; viaIdx &lt; j maximizing {@code arc - BULGE_MIN_PROGRESS_RATIO * crowFly}
   * (the "excess length" of the detour), subject to {@code BULGE_MIN_ARC_M ≤ arc
   * ≤ maxArcM}. Unlike removeMicroDetours this needs no ≤50m pinch point — it
   * catches wide-mouth bulges whose outbound and return legs never come close
   * (e.g. parallel field lanes 400m apart). Returns {@code {i, j}} or null.
   */
  static int[] findViaPinnedBulgeSpan(List<OsmPathElement> nodes, int viaIdx,
                                      int loIdx, int hiIdx, int maxArcM) {
    if (nodes == null || loIdx < 0 || hiIdx >= nodes.size()
        || viaIdx <= loIdx || viaIdx >= hiIdx) {
      return null;
    }
    double[] cum = new double[hiIdx - loIdx + 1];
    for (int k = loIdx + 1; k <= hiIdx; k++) {
      cum[k - loIdx] = cum[k - loIdx - 1] + nodes.get(k - 1).calcDistance(nodes.get(k));
    }
    double bestBenefit = 0;
    int bi = -1;
    int bj = -1;
    for (int i = viaIdx - 1; i >= loIdx; i--) {
      if (cum[viaIdx - loIdx] - cum[i - loIdx] > maxArcM) break;
      for (int j = viaIdx + 1; j <= hiIdx; j++) {
        double arc = cum[j - loIdx] - cum[i - loIdx];
        if (arc > maxArcM) break;
        if (arc < BULGE_MIN_ARC_M) continue;
        double crowFly = CheapRuler.distance(
          nodes.get(i).getILon(), nodes.get(i).getILat(),
          nodes.get(j).getILon(), nodes.get(j).getILat());
        double benefit = arc - BULGE_MIN_PROGRESS_RATIO * crowFly;
        if (benefit > bestBenefit) {
          bestBenefit = benefit;
          bi = i;
          bj = j;
        }
      }
    }
    return bi < 0 ? null : new int[]{bi, bj};
  }

  /**
   * Average cost per meter over a node span, summing per-edge cost deltas.
   * Node costs are cumulative per routed leg and reset to 0 at leg joins in
   * planner-merged tracks; a negative delta marks such a reset and that edge's
   * cost is skipped (slightly underestimates the span — conservative for the
   * junk filter, which requires the span to be expensive).
   */
  public static double spanCostPerMeter(List<OsmPathElement> nodes, int fromIdx, int toIdx) {
    double cost = 0;
    double dist = 0;
    for (int k = fromIdx + 1; k <= toIdx; k++) {
      int dc = nodes.get(k).cost - nodes.get(k - 1).cost;
      if (dc > 0) cost += dc;
      dist += nodes.get(k - 1).calcDistance(nodes.get(k));
    }
    return dist > 0 ? cost / dist : Double.MAX_VALUE;
  }

  /** Spans shorter than this are left to removeMicroDetours' teardrop band. */
  private static final int BULGE_MIN_ARC_M = 600;

  /**
   * Geometric trigger for a via-pinned bulge: the route arc across the span
   * must be at least this multiple of the mouth's crow-fly distance. The Basel
   * reference artifact (1.85km of grade2 track for 410m of progress) measures
   * 4.5; 3.0 keeps margin against firing on ordinary loop curvature near a via.
   */
  private static final double BULGE_MIN_PROGRESS_RATIO = 3.0;
}
