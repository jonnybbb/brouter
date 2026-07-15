package btools.router.roundtrip;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.RoutingEngine;

import java.util.List;

/**
 * Narrow seam between {@link FastWaypointPlanner} and the routing engine:
 * coarse delegates to the engine's shared probe/snap/island/circle
 * primitives. Deliberately small (see {@code .agents/fast-waypoint-planner-spec.md})
 * — it must NOT expose the node cache, {@code findTrack}, deadlines, or
 * mutable engine fields. The production adapter is
 * {@link RoutingEngine#fastPlacementOps()}; tests use a deterministic fake.
 */
public interface FastPlacementOps {

  /** Usability verdict for a retained probe match, in check order. */
  enum SnapUsability { OK, FERRY_LIKE, PROFILE_HOSTILE }

  /**
   * FAST-tier reachability probe: snap the bearing grid to roads and retain
   * the best usable match per direction. Hides the engine's cache reset and
   * waypoint matcher as ONE operation
   * (delegates to {@code probeReachableDirectionsFast}).
   */
  ProbeResult probe(OsmNodeNamed start, double searchRadius, double[] bearings);

  /**
   * Ferry/profile usability of a match, the same rule every snap-validation
   * site applies. Selection ({@code selectProbeDirection}, retain mode) is the
   * rule's owner; the planner re-checks committed vias as belt-and-braces.
   */
  SnapUsability snapUsability(MatchedWaypoint m);

  /**
   * Island guard: {@code false} only when {@code via} sits on a small road
   * component that cannot reach the start. The engine-field save/restore
   * around the bounded search stays inside the adapter.
   */
  boolean isViaReachable(MatchedWaypoint via, MatchedWaypoint startMatch);

  /**
   * Geometric circle fallback, validated: append circle vias + closing point
   * to {@code skeleton} and run the shared matching pass on them (delegates to
   * {@code buildPointsFromCircle} + {@code validateAndAdjustWaypoints}). Keeping
   * validation behind this op means the caller never decides whether the
   * planner's skeleton needs another pass.
   */
  void circleFallbackValidated(List<OsmNodeNamed> skeleton, double direction,
                               double searchRadius, int targetPoints);

  /** Engine log line; a no-op in {@code quite} child engines (AUTO candidates). */
  void log(String msg);
}
