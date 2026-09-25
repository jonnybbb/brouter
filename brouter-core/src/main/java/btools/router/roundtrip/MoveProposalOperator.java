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
   * Result of a proposal attempt (§4.4, §5).
   */
  public static final class MoveProposal {
    private final boolean feasible;
    private final String reason;
    private final String operator;
    private final int movedViaIndex;
    private final RefineSkeleton mutatedSkeleton;
    private final MatchedWaypoint snappedWaypoint;

    public MoveProposal(boolean feasible, String reason, int movedViaIndex,
                        RefineSkeleton mutatedSkeleton, MatchedWaypoint snappedWaypoint) {
      this(feasible, reason, "MOVE", movedViaIndex, mutatedSkeleton, snappedWaypoint);
    }

    public MoveProposal(boolean feasible, String reason, String operator, int movedViaIndex,
                        RefineSkeleton mutatedSkeleton, MatchedWaypoint snappedWaypoint) {
      this.feasible = feasible;
      this.reason = reason;
      this.operator = operator != null ? operator : "MOVE";
      this.movedViaIndex = movedViaIndex;
      this.mutatedSkeleton = mutatedSkeleton;
      this.snappedWaypoint = snappedWaypoint;
    }

    public static MoveProposal invalid(String reason) {
      return new MoveProposal(false, reason, "MOVE", -1, null, null);
    }

    public static MoveProposal invalid(String operator, String reason) {
      return new MoveProposal(false, reason, operator, -1, null, null);
    }

    public boolean isFeasible() {
      return feasible;
    }

    public String getReason() {
      return reason;
    }

    public String getOperator() {
      return operator;
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
    MatchedWaypoint origVia = findOriginalAnchor(curVia, origVias);

    // Compute mean via spacing in skeleton
    double totalSpacing = 0;
    List<MatchedWaypoint> allWps = currentSkeleton.getWaypoints();
    for (int i = 0; i < allWps.size() - 1; i++) {
      MatchedWaypoint a = allWps.get(i);
      MatchedWaypoint b = allWps.get(i + 1);
      totalSpacing += CheapRuler.distance(a.crosspoint.ilon, a.crosspoint.ilat,
        b.crosspoint.ilon, b.crosspoint.ilat);
    }
    double maxDisp = config.maxDisplacementFraction * searchRadius;
    double meanSpacing = totalSpacing / (allWps.size() - 1);
    double maxSigma = 0.5 * maxDisp;
    double sigma = Math.min(maxSigma, Math.max(30.0, 0.25 * meanSpacing));
    if (sigma < 10.0) {
      sigma = 10.0;
    }

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

    return new MoveProposal(true, "feasible", "MOVE", viaIdx, mutated, snapped);
  }

  /**
   * REPLACE operator (§4.4): Replaces one intermediate via with an alternative in the corridor.
   */
  public MoveProposal proposeReplace(RefineSkeleton currentSkeleton, RefineSkeleton originalSkeleton,
                                     SplitmixRandom rng, double searchRadius, double requestedDistance) {
    if (currentSkeleton == null || originalSkeleton == null) {
      return MoveProposal.invalid("REPLACE", "skeleton_null");
    }

    List<MatchedWaypoint> currentVias = currentSkeleton.getVias();
    List<MatchedWaypoint> origVias = originalSkeleton.getVias();
    int m = currentVias.size();
    if (m < 2) {
      return MoveProposal.invalid("REPLACE", "too_few_vias_to_replace");
    }

    // Pick one non-final via uniformly: indices [0, m - 2]
    int viaIdx = rng.nextInt(m - 1);
    MatchedWaypoint curVia = currentVias.get(viaIdx);
    MatchedWaypoint origVia = findOriginalAnchor(curVia, origVias);

    // Compute an alternative point in the corridor:
    // Sample an offset between 0.4 * maxDisp (0.10 * searchRadius) and maxDisp (0.25 * searchRadius)
    double maxDisp = config.maxDisplacementFraction * searchRadius;
    double minDisp = 0.4 * maxDisp;
    double dist = minDisp + rng.nextDouble() * (maxDisp - minDisp);
    double angle = rng.nextDouble() * 2.0 * Math.PI;

    double dx = dist * Math.cos(angle);
    double dy = dist * Math.sin(angle);

    double[] scale = CheapRuler.getLonLatToMeterScales(curVia.crosspoint.ilat);
    int dilon = (int) (dx / scale[0]);
    int dilat = (int) (dy / scale[1]);

    int rawTargetIlon = curVia.crosspoint.ilon + dilon;
    int rawTargetIlat = curVia.crosspoint.ilat + dilat;

    // Pre-snap displacement check from original position
    double preDisp = CheapRuler.distance(origVia.crosspoint.ilon, origVia.crosspoint.ilat,
      rawTargetIlon, rawTargetIlat);
    if (preDisp > maxDisp) {
      return MoveProposal.invalid("REPLACE", "pre_snap_displacement_exceeded");
    }

    // Pre-snap radius bound from start
    MatchedWaypoint startWp = currentSkeleton.getStartWp();
    double preRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat,
      rawTargetIlon, rawTargetIlat);
    double radiusBound = config.radiusBoundFactor * (requestedDistance > 0 ? requestedDistance
        : 2 * Math.PI * searchRadius);
    if (preRadius > radiusBound) {
      return MoveProposal.invalid("REPLACE", "pre_snap_radius_bound_exceeded");
    }

    // Snap target coordinates
    MatchedWaypoint snapped = snap(rawTargetIlon, rawTargetIlat, curVia.name);
    if (snapped == null || snapped.crosspoint == null || snapped.crosspoint.ilon == 0) {
      return MoveProposal.invalid("REPLACE", "snap_failed");
    }

    // Post-snap no-op check
    if (LegCache.isSeamValid(curVia, snapped)) {
      return MoveProposal.invalid("REPLACE", "noop_snap");
    }

    // Post-snap displacement check from original position
    double postDisp = CheapRuler.distance(origVia.crosspoint.ilon, origVia.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (postDisp > maxDisp) {
      return MoveProposal.invalid("REPLACE", "post_snap_displacement_exceeded");
    }

    // Post-snap radius check
    double postRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (postRadius > radiusBound) {
      return MoveProposal.invalid("REPLACE", "post_snap_radius_bound_exceeded");
    }

    // Post-snap adjacent via spacing: >= 300m
    MatchedWaypoint prevWp = (viaIdx == 0) ? startWp : currentVias.get(viaIdx - 1);
    MatchedWaypoint nextWp = currentVias.get(viaIdx + 1);

    double prevDist = CheapRuler.distance(prevWp.crosspoint.ilon, prevWp.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (prevDist < config.minViaSpacingMeters) {
      return MoveProposal.invalid("REPLACE", "adjacent_spacing_under_min_prev");
    }

    double nextDist = CheapRuler.distance(snapped.crosspoint.ilon, snapped.crosspoint.ilat,
      nextWp.crosspoint.ilon, nextWp.crosspoint.ilat);
    if (nextDist < config.minViaSpacingMeters) {
      return MoveProposal.invalid("REPLACE", "adjacent_spacing_under_min_next");
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
    return new MoveProposal(true, "feasible", "REPLACE", viaIdx, mutated, snapped);
  }

  /**
   * 2-OPT operator (§4.4, §5): Reverses an intermediate sub-chain of non-final vias.
   * Frozen final via and closing leg are preserved.
   */
  public MoveProposal propose2Opt(RefineSkeleton currentSkeleton) {
    return propose2Opt(currentSkeleton, new SplitmixRandom(42L));
  }

  /**
   * 2-OPT operator (§4.4, §5): Reverses an intermediate sub-chain of non-final vias with RNG.
   * Frozen final via and closing leg are preserved.
   */
  public MoveProposal propose2Opt(RefineSkeleton currentSkeleton, SplitmixRandom rng) {
    if (currentSkeleton == null) {
      return MoveProposal.invalid("2-OPT", "skeleton_null");
    }

    List<MatchedWaypoint> currentVias = currentSkeleton.getVias();
    int m = currentVias.size();
    // Non-final vias are at indices 0..m-2 (m-1 is frozen final via!).
    // We need at least 2 non-final vias to reverse (i < j <= m-2), so m >= 3.
    if (m < 3) {
      return MoveProposal.invalid("2-OPT", "too_few_vias_for_2opt");
    }

    // Pick i in [0, m - 3] and j in [i + 1, m - 2]
    int i = rng.nextInt(m - 2);
    int j = (i + 1) + rng.nextInt(m - 1 - (i + 1));

    MatchedWaypoint startWp = currentSkeleton.getStartWp();
    MatchedWaypoint wpBeforeI = (i == 0) ? startWp : currentVias.get(i - 1);
    MatchedWaypoint viaJ = currentVias.get(j);
    MatchedWaypoint viaI = currentVias.get(i);
    MatchedWaypoint wpAfterJ = currentVias.get(j + 1); // safe because j <= m - 2 < m - 1

    // Check adjacent spacing after reversal:
    // New connection 1: wpBeforeI -> viaJ
    double d1 = CheapRuler.distance(wpBeforeI.crosspoint.ilon, wpBeforeI.crosspoint.ilat,
      viaJ.crosspoint.ilon, viaJ.crosspoint.ilat);
    if (d1 < config.minViaSpacingMeters) {
      return MoveProposal.invalid("2-OPT", "spacing_under_min_entry");
    }

    // New connection 2: viaI -> wpAfterJ
    double d2 = CheapRuler.distance(viaI.crosspoint.ilon, viaI.crosspoint.ilat,
      wpAfterJ.crosspoint.ilon, wpAfterJ.crosspoint.ilat);
    if (d2 < config.minViaSpacingMeters) {
      return MoveProposal.invalid("2-OPT", "spacing_under_min_exit");
    }

    // Build new vias list with sublist [i..j] reversed
    List<MatchedWaypoint> newVias = new ArrayList<>(m);
    for (int k = 0; k < i; k++) {
      newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(k)));
    }
    for (int k = j; k >= i; k--) {
      newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(k)));
    }
    for (int k = j + 1; k < m; k++) {
      newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(k)));
    }

    RefineSkeleton mutated = new RefineSkeleton(startWp, newVias, currentSkeleton.getEndWp());
    return new MoveProposal(true, "feasible", "2-OPT", -1, mutated, null);
  }

  /**
   * INSERT operator (§4.4, §5): Inserts a via between two distant intermediate vias.
   * Frozen final via and closing leg are preserved (never inserted on closing leg).
   */
  public MoveProposal proposeInsert(RefineSkeleton currentSkeleton, SplitmixRandom rng,
                                    double searchRadius, double requestedDistance) {
    if (currentSkeleton == null) {
      return MoveProposal.invalid("INSERT", "skeleton_null");
    }

    List<MatchedWaypoint> currentVias = currentSkeleton.getVias();
    int m = currentVias.size();
    if (m < 1) {
      return MoveProposal.invalid("INSERT", "too_few_vias_to_insert");
    }
    if (m >= 16) {
      return MoveProposal.invalid("INSERT", "max_vias_exceeded");
    }

    // Rule (§4.4, ADR-0002): NEVER insert on the closing leg!
    // The closing leg is from final via (currentVias.get(m - 1)) to endWp.
    // Intermediate segments are:
    // seg 0: startWp -> currentVias.get(0)
    // seg k (1 <= k < m): currentVias.get(k - 1) -> currentVias.get(k)
    // So there are m segments (0 to m - 1), all before or ending at the final via.
    MatchedWaypoint startWp = currentSkeleton.getStartWp();
    List<Integer> eligibleSegs = new ArrayList<>();
    for (int k = 0; k < m; k++) {
      MatchedWaypoint from = (k == 0) ? startWp : currentVias.get(k - 1);
      MatchedWaypoint to = currentVias.get(k);
      double dist = CheapRuler.distance(from.crosspoint.ilon, from.crosspoint.ilat,
        to.crosspoint.ilon, to.crosspoint.ilat);
      if (dist >= 2.0 * config.minViaSpacingMeters) { // >= 600m
        eligibleSegs.add(k);
      }
    }

    if (eligibleSegs.isEmpty()) {
      return MoveProposal.invalid("INSERT", "no_segment_long_enough");
    }

    int chosenSeg = eligibleSegs.get(rng.nextInt(eligibleSegs.size()));
    MatchedWaypoint from = (chosenSeg == 0) ? startWp : currentVias.get(chosenSeg - 1);
    MatchedWaypoint to = currentVias.get(chosenSeg);

    // Midpoint between from and to
    int midLon = (from.crosspoint.ilon + to.crosspoint.ilon) / 2;
    int midLat = (from.crosspoint.ilat + to.crosspoint.ilat) / 2;

    // Small orthogonal perturbation (up to +- 100m perpendicular to segment)
    double[] scale = CheapRuler.getLonLatToMeterScales(midLat);
    double dLonMeters = (to.crosspoint.ilon - from.crosspoint.ilon) * scale[0];
    double dLatMeters = (to.crosspoint.ilat - from.crosspoint.ilat) * scale[1];
    double segLen = Math.sqrt(dLonMeters * dLonMeters + dLatMeters * dLatMeters);

    int rawLon;
    int rawLat;
    if (segLen > 1.0) {
      double perpOffset = (rng.nextDouble() * 200.0 - 100.0);
      double offX = -perpOffset * (dLatMeters / segLen);
      double offY = perpOffset * (dLonMeters / segLen);
      rawLon = midLon + (int) (offX / scale[0]);
      rawLat = midLat + (int) (offY / scale[1]);
    } else {
      rawLon = midLon;
      rawLat = midLat;
    }

    // Radius check from start
    double preRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat, rawLon, rawLat);
    double radiusBound = config.radiusBoundFactor * (requestedDistance > 0 ? requestedDistance
        : 2 * Math.PI * searchRadius);
    if (preRadius > radiusBound) {
      return MoveProposal.invalid("INSERT", "pre_snap_radius_bound_exceeded");
    }

    // Snap target
    String insertName = "refine_insert_" + chosenSeg + "_" + rawLon + "_" + rawLat;
    MatchedWaypoint snapped = snap(rawLon, rawLat, insertName);
    if (snapped == null || snapped.crosspoint == null || snapped.crosspoint.ilon == 0) {
      return MoveProposal.invalid("INSERT", "snap_failed");
    }

    // Spacing checks
    double dFrom = CheapRuler.distance(from.crosspoint.ilon, from.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (dFrom < config.minViaSpacingMeters) {
      return MoveProposal.invalid("INSERT", "spacing_under_min_from");
    }

    double dTo = CheapRuler.distance(snapped.crosspoint.ilon, snapped.crosspoint.ilat,
      to.crosspoint.ilon, to.crosspoint.ilat);
    if (dTo < config.minViaSpacingMeters) {
      return MoveProposal.invalid("INSERT", "spacing_under_min_to");
    }

    // Post-snap radius check
    double postRadius = CheapRuler.distance(startWp.crosspoint.ilon, startWp.crosspoint.ilat,
      snapped.crosspoint.ilon, snapped.crosspoint.ilat);
    if (postRadius > radiusBound) {
      return MoveProposal.invalid("INSERT", "post_snap_radius_bound_exceeded");
    }

    // Assemble new vias list with inserted via at index chosenSeg
    List<MatchedWaypoint> newVias = new ArrayList<>(m + 1);
    for (int k = 0; k < chosenSeg; k++) {
      newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(k)));
    }
    newVias.add(RefineSkeleton.copyWaypoint(snapped));
    for (int k = chosenSeg; k < m; k++) {
      newVias.add(RefineSkeleton.copyWaypoint(currentVias.get(k)));
    }

    RefineSkeleton mutated = new RefineSkeleton(startWp, newVias, currentSkeleton.getEndWp());
    return new MoveProposal(true, "feasible", "INSERT", chosenSeg, mutated, snapped);
  }

  /**
   * Unified proposal generator (§5): Mixes proposal operators according to weights.
   */
  public MoveProposal propose(RefineSkeleton currentSkeleton, RefineSkeleton originalSkeleton,
                              SplitmixRandom rng, double searchRadius, double requestedDistance) {
    // Mix: 50% MOVE, 20% 2-OPT, 15% REPLACE, 15% INSERT
    double roll = rng.nextDouble();
    if (roll < 0.50) {
      return proposeMove(currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);
    } else if (roll < 0.70) {
      MoveProposal p = propose2Opt(currentSkeleton, rng);
      if (p.isFeasible()) {
        return p;
      }
      return proposeMove(currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);
    } else if (roll < 0.85) {
      MoveProposal p = proposeReplace(currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);
      if (p.isFeasible()) {
        return p;
      }
      return proposeMove(currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);
    } else {
      MoveProposal p = proposeInsert(currentSkeleton, rng, searchRadius, requestedDistance);
      if (p.isFeasible()) {
        return p;
      }
      return proposeMove(currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);
    }
  }

  /**
   * Resolve original anchor for a via to bound cumulative displacement drift (§4.4).
   */
  public static MatchedWaypoint findOriginalAnchor(MatchedWaypoint curVia, List<MatchedWaypoint> origVias) {
    if (curVia == null) {
      return null;
    }
    // For inserted vias, extract initial position if encoded in name: refine_insert_<seg>_<lon>_<lat>
    if (curVia.name != null && curVia.name.startsWith("refine_insert_")) {
      String[] parts = curVia.name.split("_");
      if (parts.length >= 5) {
        try {
          int aLon = Integer.parseInt(parts[3]);
          int aLat = Integer.parseInt(parts[4]);
          MatchedWaypoint anchor = new MatchedWaypoint();
          anchor.name = curVia.name;
          anchor.crosspoint = new OsmNode(aLon, aLat);
          return anchor;
        } catch (NumberFormatException ignored) {
          // fall through to curVia
        }
      }
      return curVia;
    }
    // Match by name against original vias
    if (origVias != null && curVia.name != null) {
      for (MatchedWaypoint orig : origVias) {
        if (curVia.name.equals(orig.name)) {
          return orig;
        }
      }
    }
    // Fallback: return curVia itself if no match found
    return curVia;
  }

  private MatchedWaypoint snap(int ilon, int ilat, String name) {
    long snapKey = (((long) (ilon / 1000)) << 32) | ((ilat / 1000) & 0xffffffffL);
    MatchedWaypoint cached = snapCache.get(snapKey);
    if (cached != null) {
      MatchedWaypoint cp = RefineSkeleton.copyWaypoint(cached);
      cp.name = name != null ? name : "refine_via";
      return cp;
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
