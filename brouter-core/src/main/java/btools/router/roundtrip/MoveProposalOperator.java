package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

/**
 * MOVE proposal operator for refinement (§4.4).
 * Mutates one non-start, non-final via with a Gaussian displacement.
 * Validates skeleton constraints before and after snapping.
 */
public final class MoveProposalOperator {

  /**
   * Result of a MOVE proposal attempt.
   */
  public static final class MoveProposal {
    private final boolean feasible;
    private final String reason;
    private final int movedViaIndex;
    private final RefineSkeleton mutatedSkeleton;
    private final MatchedWaypoint snappedWaypoint;

    public MoveProposal(boolean feasible, String reason, int movedViaIndex,
                        RefineSkeleton mutatedSkeleton, MatchedWaypoint snappedWaypoint) {
      this.feasible = feasible;
      this.reason = reason;
      this.movedViaIndex = movedViaIndex;
      this.mutatedSkeleton = mutatedSkeleton;
      this.snappedWaypoint = snappedWaypoint;
    }

    public static MoveProposal invalid(String reason) {
      return new MoveProposal(false, reason, -1, null, null);
    }

    public boolean isFeasible() {
      return feasible;
    }

    public String getReason() {
      return reason;
    }

    public int getMovedViaIndex() {
      return movedViaIndex;
    }

    public RefineSkeleton getMutatedSkeleton() {
      return mutatedSkeleton;
    }

    public MatchedWaypoint getSnappedWaypoint() {
      return snappedWaypoint;
    }
  }

  private final RoundTripEngineOps ops;
  private final RefineConfig config;
  private final Map<Long, MatchedWaypoint> snapCache = new HashMap<>();

  public MoveProposalOperator(RoundTripEngineOps ops, RefineConfig config) {
    this.ops = ops;
    this.config = config != null ? config : new RefineConfig();
  }

  /**
   * Generate one MOVE proposal from the current skeleton, checked against originalSkeleton.
   */
  public MoveProposal proposeMove(RefineSkeleton currentSkeleton, RefineSkeleton originalSkeleton,
                                  SplitmixRandom rng, double searchRadius, double requestedDistance) {
    if (currentSkeleton == null || originalSkeleton == null) {
      return MoveProposal.invalid("skeleton_null");
    }

    List<MatchedWaypoint> currentVias = currentSkeleton.getVias();
    List<MatchedWaypoint> origVias = originalSkeleton.getVias();
    int m = currentVias.size();
    if (m < 2) {
      // Final via is frozen; need at least one non-final intermediate via to move
      return MoveProposal.invalid("too_few_vias_to_move");
    }

    // Pick one non-final via uniformly: indices [0, m - 2]
    int viaIdx = rng.nextInt(m - 1);
    MatchedWaypoint curVia = currentVias.get(viaIdx);
    MatchedWaypoint origVia = origVias.get(viaIdx);

    // Compute mean via spacing in skeleton
    double totalSpacing = 0;
    List<MatchedWaypoint> allWps = currentSkeleton.getWaypoints();
    for (int i = 0; i < allWps.size() - 1; i++) {
      MatchedWaypoint a = allWps.get(i);
      MatchedWaypoint b = allWps.get(i + 1);
      totalSpacing += CheapRuler.distance(a.crosspoint.ilon, a.crosspoint.ilat,
        b.crosspoint.ilon, b.crosspoint.ilat);
    }
    double meanSpacing = totalSpacing / (allWps.size() - 1);
    double sigma = Math.max(150.0, 0.5 * meanSpacing);

    // Generate Gaussian offset
    double dx = rng.nextGaussian() * sigma;
    double dy = rng.nextGaussian() * sigma;

    double[] scale = CheapRuler.getLonLatToMeterScales(curVia.crosspoint.ilat);
    int dilon = (int) (dx / scale[0]);
    int dilat = (int) (dy / scale[1]);

    int rawTargetIlon = curVia.crosspoint.ilon + dilon;
    int rawTargetIlat = curVia.crosspoint.ilat + dilat;

    // Pre-snap displacement check from original position
    double preDisp = CheapRuler.distance(origVia.crosspoint.ilon, origVia.crosspoint.ilat,
      rawTargetIlon, rawTargetIlat);
    double maxDisp = config.maxDisplacementFraction * searchRadius;
    if (preDisp > maxDisp) {
      return MoveProposal.invalid("pre_snap_displacement_exceeded");
    }

    // Pre-snap radius bound from start
    MatchedWaypoint startWp = currentSkeleton.getStartWp();
    double preRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat,
      rawTargetIlon, rawTargetIlat);
    double radiusBound = config.radiusBoundFactor * (requestedDistance > 0 ? requestedDistance : 2 * Math.PI * searchRadius);
    if (preRadius > radiusBound) {
      return MoveProposal.invalid("pre_snap_radius_bound_exceeded");
    }

    // Snap target coordinates
    MatchedWaypoint snapped = snap(rawTargetIlon, rawTargetIlat, curVia.name);
    if (snapped == null || snapped.crosspoint == null || snapped.crosspoint.ilon == 0) {
      return MoveProposal.invalid("snap_failed");
    }

    // Post-snap no-op check
    if (LegCache.isSeamValid(curVia, snapped)) {
      return MoveProposal.invalid("noop_snap");
    }

    // Post-snap displacement check from original position
    double postDisp = CheapRuler.distance(origVia.crosspoint.ilon, origVia.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (postDisp > maxDisp) {
      return MoveProposal.invalid("post_snap_displacement_exceeded");
    }

    // Post-snap radius check
    double postRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (postRadius > radiusBound) {
      return MoveProposal.invalid("post_snap_radius_bound_exceeded");
    }

    // Post-snap adjacent via spacing: >= 300m
    MatchedWaypoint prevWp = (viaIdx == 0) ? startWp : currentVias.get(viaIdx - 1);
    MatchedWaypoint nextWp = currentVias.get(viaIdx + 1); // viaIdx < m - 1, so viaIdx + 1 is valid

    double prevDist = CheapRuler.distance(prevWp.crosspoint.ilon, prevWp.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (prevDist < config.minViaSpacingMeters) {
      return MoveProposal.invalid("adjacent_spacing_under_min_prev");
    }

    double nextDist = CheapRuler.distance(snapped.crosspoint.ilon, snapped.crosspoint.ilat,
      nextWp.crosspoint.ilon, nextWp.crosspoint.ilat);
    if (nextDist < config.minViaSpacingMeters) {
      return MoveProposal.invalid("adjacent_spacing_under_min_next");
    }

    // Assemble mutated skeleton
    List<MatchedWaypoint> newVias = new ArrayList<>(currentVias.size());
    for (int i = 0; i < currentVias.size(); i++) {
      if (i == viaIdx) {
        newVias.add(RefineSkeleton.copyWaypoint(snapped));
      } else {
        newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(i)));
      }
    }
    RefineSkeleton mutated = new RefineSkeleton(startWp, newVias, currentSkeleton.getEndWp());

    return new MoveProposal(true, "feasible", viaIdx, mutated, snapped);
  }

  private MatchedWaypoint snap(int ilon, int ilat, String name) {
    long snapKey = (((long) (ilon / 1000)) << 32) | ((ilat / 1000) & 0xffffffffL);
    MatchedWaypoint cached = snapCache.get(snapKey);
    if (cached != null) {
      return RefineSkeleton.copyWaypoint(cached);
    }

    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.name = name != null ? name : "refine_via";
    mwp.waypoint = new OsmNode(ilon, ilat);
    mwp.crosspoint = new OsmNode();
    mwp.node1 = new OsmNode();
    mwp.node2 = new OsmNode();

    List<MatchedWaypoint> list = new ArrayList<>(1);
    list.add(mwp);
    ops.matchWaypointsToNodes(list, 2000.0);

    if (mwp.crosspoint != null && mwp.crosspoint.ilon != 0) {
      snapCache.put(snapKey, RefineSkeleton.copyWaypoint(mwp));
      return mwp;
    }
    return null;
  }
}
