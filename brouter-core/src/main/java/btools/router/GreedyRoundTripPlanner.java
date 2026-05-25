package btools.router;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.util.CheapAngleMeter;
import btools.util.CheapRuler;

/**
 * Greedy sub-route algorithm for cycling round-trip generation.
 * <p>
 * Follows the pattern from "Efficient Dijkstra-Based Greedy Algorithm for
 * Cycle-Route Planning" (CEUR-WS Vol-3885):
 * <ol>
 *   <li>Generate candidate waypoints at the target sub-route distance</li>
 *   <li>Score ALL candidates by air-distance heuristics (O(1) each, no routing)</li>
 *   <li>Rank candidates; route only the top pick via full Dijkstra</li>
 *   <li>If route fails, try the next-ranked candidate</li>
 *   <li>Compute ONE return path to start; check if loop closes within tolerance</li>
 *   <li>Repeat until loop closes or max steps exhausted</li>
 * </ol>
 * This gives 1-2 Dijkstra per step (sub-route + return) instead of N per step,
 * making the algorithm practical for real-time use.
 */
public class GreedyRoundTripPlanner {

  private static final int DEFAULT_SUB_ROUTE_COUNT = 5;
  private static final double DEFAULT_TOLERANCE = 0.05;
  private static final int DEFAULT_MAX_ATTEMPTS = 8;
  private static final double ROAD_INDIRECTNESS = 1.3;
  private static final long SUB_ROUTE_TIMEOUT_MS = 10000;
  /**
   * Whole-plan wall-clock ceiling. Worst-case per-sub-route timing
   * (subRouteCount × maxAttempts × MAX_ROUTE_ATTEMPTS × SUB_ROUTE_TIMEOUT_MS)
   * blows past 20 minutes; this is the safety net. Each timedFindTrack call
   * uses min(SUB_ROUTE_TIMEOUT_MS, deadline - now) so the planner stops
   * issuing new Dijkstras after the deadline.
   */
  private static final long DEFAULT_PLAN_DEADLINE_MS = 30_000;
  /** Minimum per-Dijkstra timeout. Below this it's cheaper to skip than try. */
  private static final long MIN_FIND_TRACK_MS = 250;
  /**
   * Backoff factors:
   *   - "no routable candidate found at this radius" — gentle shrink so we don't
   *     skip viable nearby radii after a few candidates fail to snap/route.
   *   - "route too long" — aggressive shrink, the radius really needs to come down.
   * Both clamp at MIN_LOCAL_RADIUS_M so we don't collapse to a degenerate 0m radius.
   */
  private static final double BACKOFF_FACTOR_NO_CANDIDATE = 0.8;
  private static final double BACKOFF_FACTOR_TOO_LONG = 0.5;
  private static final double MIN_LOCAL_RADIUS_M = 200;

  /**
   * Quality gates on bestFallback / forced-closure loops. A loop that's
   * grossly the wrong length, retraces > half itself, or doesn't close gets
   * downgraded — caller (RoutingEngine.doGreedyRoundTrip) treats a result
   * with fallbackReason starting with "rejected" as a planner failure and
   * falls through to WAYPOINT, rather than ship a low-quality loop as success.
   */
  static final String DEGRADED_FALLBACK_PREFIX = "rejected: ";
  private static final double FALLBACK_MIN_RATIO = 0.5;
  private static final double FALLBACK_MAX_RATIO = 1.8;
  private static final double FALLBACK_MAX_REUSE = 0.5;
  private static final int FALLBACK_MAX_CLOSURE_M = 400;
  // Max candidates to route per step (heuristic top-K, with angular spread).
  private static final int MAX_ROUTE_ATTEMPTS = 3;
  /** Raised cap on late steps or after a failed attempt, where extra exploration pays off. */
  private static final int MAX_ROUTE_ATTEMPTS_LATE = 5;
  /**
   * Min angular separation between routed candidates within a step. Top-K by raw
   * heuristic score is often spatially redundant in dense networks (two adjacent
   * road choices have similar scores); enforcing a 30° gap gives diverse routed
   * options instead of three picks in the same micro-direction.
   */
  private static final double MIN_ANGULAR_SEPARATION_DEG = 30.0;
  // Weight applied to cost-per-meter when picking among routed candidates.
  // Magnitude is similar to scorer.score() output; 0.5 keeps both signals relevant.
  static final double COST_PER_METER_WEIGHT = 0.5;
  // Multiplier applied to the air-distance return estimate when deciding
  // whether to skip the return Dijkstra. > 1 means we skip less aggressively.
  private static final double RETURN_SKIP_SAFETY = 1.5;

  private final RoutingEngine engine;
  private final CandidateScorer scorer;
  private final RoundTripCandidateProvider candidateProvider;

  private final int subRouteCount;
  private final double tolerance;
  private final int maxAttempts;

  public GreedyRoundTripPlanner(RoutingEngine engine) {
    this(engine, new RoundTripCandidateProvider.RadialCandidateProvider(),
      new CandidateScorer(), DEFAULT_SUB_ROUTE_COUNT, DEFAULT_TOLERANCE, DEFAULT_MAX_ATTEMPTS);
  }

  public GreedyRoundTripPlanner(RoutingEngine engine, RoundTripCandidateProvider provider) {
    this(engine, provider, new CandidateScorer(),
      DEFAULT_SUB_ROUTE_COUNT, DEFAULT_TOLERANCE, DEFAULT_MAX_ATTEMPTS);
  }

  /**
   * Configure scorer/sub-route count/tolerance/max-attempts. Uses the default
   * {@link RoundTripCandidateProvider.RadialCandidateProvider}; for QUALITY-tier
   * callers, prefer the 6-arg ctor with an {@link IsochroneCandidateProvider}.
   */
  public GreedyRoundTripPlanner(RoutingEngine engine, CandidateScorer scorer,
                                int subRouteCount, double tolerance, int maxAttempts) {
    this(engine, new RoundTripCandidateProvider.RadialCandidateProvider(),
      scorer, subRouteCount, tolerance, maxAttempts);
  }

  public GreedyRoundTripPlanner(RoutingEngine engine, RoundTripCandidateProvider provider,
                                CandidateScorer scorer, int subRouteCount, double tolerance,
                                int maxAttempts) {
    this.engine = engine;
    this.candidateProvider = provider;
    this.scorer = scorer;
    this.subRouteCount = subRouteCount;
    this.tolerance = tolerance;
    this.maxAttempts = maxAttempts;
  }

  /**
   * Plan a greedy round-trip loop.
   */
  public RoundTripResult plan(OsmNodeNamed start, double desiredDistance, double startDirection) {
    long planStart = System.currentTimeMillis();
    long deadline = planStart + DEFAULT_PLAN_DEADLINE_MS;
    RoundTripResult result = new RoundTripResult();
    double subTarget = desiredDistance / subRouteCount;
    Map<Long, Integer> visitedEdgeCounts = new HashMap<>();
    // Parallel map: first-visit cumulative distance from start, per edge. Lets
    // the reuse signal weight back-and-forth near start/end heavier than mid-
    // loop reuse — the start/end retraces are visible & annoying; mid-loop
    // reuse is often unavoidable in constrained terrain.
    Map<Long, Double> visitedEdgeFirstPos = new HashMap<>();
    List<OsmTrack> segments = new ArrayList<>();
    int totalAttempts = 0;
    double totalDistance = 0;
    int candidatesGenerated = 0;
    int candidatesRouted = 0;
    int returnChecksPerformed = 0;

    MatchedWaypoint startMwp = matchPoint(start.ilon, start.ilat, "greedy_start");
    if (startMwp == null) {
      result.setFallbackReason("start point not on road network");
      stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
      return result;
    }

    MatchedWaypoint currentMwp = startMwp;
    List<MatchedWaypoint> waypointStack = new ArrayList<>();
    waypointStack.add(startMwp);

    Snapshot bestFallback = null;

    DirectionPreference dirPref = DirectionPreference.ANY;
    if (startDirection >= 0) {
      dirPref = nearestDirectionPreference(startDirection);
    }

    double searchRadius = desiredDistance / 4.0;
    int prevIlon = -1;
    int prevIlat = -1;

    for (int step = 1; step <= subRouteCount; step++) {
      if (System.currentTimeMillis() >= deadline) {
        result.addDiagnostic("step " + step + ": planner deadline reached, stopping");
        break;
      }
      boolean candidateFound = false;
      double localRadius = subTarget;
      int currentIlon = currentMwp.crosspoint.getILon();
      int currentIlat = currentMwp.crosspoint.getILat();
      // Segments only change across steps — any tentative append is undone on retry.
      OsmTrack cachedRefTrack = segments.isEmpty() ? null : buildRefTrack(segments);

      for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        totalAttempts++;
        if (System.currentTimeMillis() >= deadline) break;

        double airRadius = localRadius / ROAD_INDIRECTNESS;

        // --- Phase 1: Generate candidates and score by heuristics (no routing) ---
        List<RoundTripCandidateProvider.CandidatePoint> candidates =
          candidateProvider.candidatesForStep(
            currentIlon, currentIlat, airRadius,
            step, subRouteCount,
            start.ilon, start.ilat,
            startDirection);
        candidatesGenerated += candidates.size();

        // Score using air-distance estimates — O(1) per candidate
        for (RoundTripCandidateProvider.CandidatePoint cp : candidates) {
          double airDistToCp = CheapRuler.distance(currentIlon, currentIlat, cp.ilon, cp.ilat);
          double estimatedRouteDist = airDistToCp * ROAD_INDIRECTNESS;
          double airDistToStart = CheapRuler.distance(cp.ilon, cp.ilat, start.ilon, start.ilat);
          double estimatedReturn = airDistToStart * ROAD_INDIRECTNESS;
          double distFromStart = airDistToStart;

          double distFromPrevious = (prevIlon >= 0)
            ? CheapRuler.distance(prevIlon, prevIlat, cp.ilon, cp.ilat) * ROAD_INDIRECTNESS
            : -1;

          cp.score = scorer.score(
            estimatedRouteDist, subTarget,
            totalDistance, estimatedReturn, desiredDistance,
            cp.bearing, dirPref,
            step, subRouteCount,
            0.0, // can't estimate visited ratio without routing
            distFromStart, searchRadius,
            distFromPrevious,
            cp.costFromStart, cp.bucketHits, cp.sourceContour);
        }

        // Rank by score (lowest = best)
        candidates.sort(Comparator.comparingDouble(c -> c.score));

        // --- Phase 2: Route top candidates, pick best by combined routed score ---
        // Heuristic score uses visitedEdgeRatio=0 since pre-routing can't know it.
        // Re-score with actual route distance and visited ratio so reuse-heavy
        // candidates lose to fresh ones at similar cost-per-meter.
        //
        // Pick top-K candidates with angular spread (≥ MIN_ANGULAR_SEPARATION_DEG
        // between picks) rather than just the top K by score — the top heuristic
        // picks are often spatially redundant in dense networks. Bump K from
        // MAX_ROUTE_ATTEMPTS to MAX_ROUTE_ATTEMPTS_LATE on late steps or after
        // an earlier failed attempt this step, where extra exploration pays off.
        int routeBudget = (step >= subRouteCount - 1 || attempt > 1)
          ? MAX_ROUTE_ATTEMPTS_LATE : MAX_ROUTE_ATTEMPTS;
        List<RoundTripCandidateProvider.CandidatePoint> toRoute =
          pickDiverseTopK(candidates, routeBudget);

        ScoredRoute accepted = null;
        double bestRoutedScore = Double.MAX_VALUE;
        int routeAttempts = toRoute.size();
        MatchedWaypoint fromMwp = currentMwp;

        for (int r = 0; r < routeAttempts; r++) {
          RoundTripCandidateProvider.CandidatePoint cp = toRoute.get(r);

          MatchedWaypoint toMwp = matchPoint(cp.ilon, cp.ilat, "greedy_to");
          if (toMwp == null) continue;

          // Snap distance from the geometric candidate to its routed-on-road
          // crosspoint. Reject candidates that snapped too far away.
          int snappedIlon = toMwp.crosspoint.getILon();
          int snappedIlat = toMwp.crosspoint.getILat();
          double snapDist = CheapRuler.distance(cp.ilon, cp.ilat, snappedIlon, snappedIlat);
          if (snapDist > airRadius * 0.5) continue;

          OsmTrack subTrack = timedFindTrack("greedy-sub", fromMwp, toMwp, cachedRefTrack, deadline);
          candidatesRouted++;
          if (subTrack == null || subTrack.distance == 0) continue;

          // Recompute scoring inputs from the SNAPPED endpoint (toMwp.crosspoint).
          // The router actually travels to that snapped location, not the raw
          // candidate point — so air-distance, bearing, return estimate, and the
          // overlong-route reject threshold should all reflect what was routed.
          double snappedAirDistFromCurrent = CheapRuler.distance(
            currentIlon, currentIlat, snappedIlon, snappedIlat);
          if (subTrack.distance > snappedAirDistFromCurrent * 3.0) continue;

          double actualVisitedRatio = computeTrackVisitedRatio(subTrack,
            visitedEdgeCounts, visitedEdgeFirstPos, totalDistance, desiredDistance);
          double airDistToStart = CheapRuler.distance(snappedIlon, snappedIlat, start.ilon, start.ilat);
          double estimatedReturn = airDistToStart * ROAD_INDIRECTNESS;
          double distFromPrevious = (prevIlon >= 0)
            ? CheapRuler.distance(prevIlon, prevIlat, snappedIlon, snappedIlat) * ROAD_INDIRECTNESS
            : -1;
          double snappedBearing = CheapRuler.getScaledBearing(
            currentIlon, currentIlat, snappedIlon, snappedIlat);

          double routedScorerScore = scorer.score(
            subTrack.distance, subTarget,
            totalDistance, estimatedReturn, desiredDistance,
            snappedBearing, dirPref,
            step, subRouteCount,
            actualVisitedRatio,
            airDistToStart, searchRadius,
            distFromPrevious,
            cp.costFromStart, cp.bucketHits, cp.sourceContour);

          double costPerMeter = (double) subTrack.cost / subTrack.distance;
          double routedScore = combinedRoutedScore(routedScorerScore, costPerMeter);

          if (routedScore < bestRoutedScore) {
            bestRoutedScore = routedScore;
            accepted = new ScoredRoute();
            accepted.track = subTrack;
            accepted.toMwp = toMwp;
            accepted.routeDistance = subTrack.distance;
            accepted.visitedRatio = actualVisitedRatio;
          }
        }

        if (accepted == null) {
          // No routable candidate at this radius — gentle shrink so we don't
          // jump past viable radii. The aggressive halving below applies only
          // when the route is too long.
          result.addDiagnostic("step " + step + " attempt " + attempt
            + ": no routable candidate at radius " + (int) localRadius);
          localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_NO_CANDIDATE);
          continue;
        }

        result.addDiagnostic("step " + step + ": routed " + (int) accepted.routeDistance
          + "m (target " + (int) subTarget + "m)"
          + ", reuse=" + String.format("%.1f%%", accepted.visitedRatio * 100));

        // --- Phase 3: Accept sub-route, advance position ---
        addVisitedEdges(accepted.track, visitedEdgeCounts, visitedEdgeFirstPos, totalDistance);
        segments.add(accepted.track);
        totalDistance += accepted.routeDistance;

        // Record previous waypoint position for next step's Silesian scoring.
        // Save old values so we can restore on undo.
        int savedPrevIlon = prevIlon;
        int savedPrevIlat = prevIlat;
        prevIlon = currentIlon;
        prevIlat = currentIlat;

        // Use actual track endpoint for next step
        OsmPathElement lastNode = accepted.track.nodes.get(accepted.track.nodes.size() - 1);
        MatchedWaypoint nextMwp = matchPoint(lastNode.getILon(), lastNode.getILat(), "greedy_next");
        currentMwp = (nextMwp != null) ? nextMwp : accepted.toMwp;
        waypointStack.add(currentMwp);

        // --- Phase 4: Check loop closure (ONE return Dijkstra per step) ---
        int curIlon = currentMwp.crosspoint.getILon();
        int curIlat = currentMwp.crosspoint.getILat();
        double airDistToStart = CheapRuler.distance(curIlon, curIlat, start.ilon, start.ilat);
        double minReturn = airDistToStart * ROAD_INDIRECTNESS;

        // Skip the return check only when closure is clearly out of reach AND
        // we still have multiple steps left. ROAD_INDIRECTNESS is a heuristic;
        // constrained networks can force much longer returns, so apply a safety
        // factor and never skip on the last two steps where closure matters.
        boolean isLateStep = step >= subRouteCount - 1;
        if (!isLateStep
          && totalDistance + minReturn * RETURN_SKIP_SAFETY < desiredDistance * (1 - tolerance)) {
          candidateFound = true;
          break;
        }

        // One Dijkstra: return path to start.
        OsmTrack returnTrack = timedFindTrack("greedy-return", currentMwp, startMwp,
          buildRefTrack(segments), deadline);
        returnChecksPerformed++;
        if (returnTrack != null && returnTrack.distance > 0) {
          double closedDistance = totalDistance + returnTrack.distance;
          double error = Math.abs(closedDistance - desiredDistance) / desiredDistance;

          // Snapshot now: later retries may mutate segments / waypointStack.
          if (bestFallback == null || error < bestFallback.error) {
            bestFallback = snapshotFallback(segments, returnTrack, waypointStack, error);
          }

          // Within tolerance → close the loop
          if (error <= tolerance) {
            addVisitedEdges(returnTrack, visitedEdgeCounts, visitedEdgeFirstPos, totalDistance);
            segments.add(returnTrack);
            OsmTrack finalTrack = mergeSegments(segments, null);
            populateResult(result, finalTrack, waypointStack, start, startMwp, segments, desiredDistance, startDirection);
            result.setTotalDistanceMeters((int) closedDistance);
            result.setWithinTolerance(true);
            result.setSubRoutesChosen(step);
            result.setAttemptsUsed(totalAttempts);
            result.addDiagnostic("loop closed at step " + step
              + ", total=" + (int) closedDistance + "m"
              + ", error=" + String.format("%.1f%%", error * 100));
            stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
            return result;
          }

          // Too long → undo sub-route, aggressively shrink radius, retry.
          if (closedDistance > desiredDistance * (1 + tolerance)) {
            result.addDiagnostic("step " + step + ": projected " + (int) closedDistance
              + "m exceeds desired " + (int) desiredDistance + "m, shrinking radius");
            segments.remove(segments.size() - 1);
            totalDistance -= accepted.routeDistance;
            removeVisitedEdges(accepted.track, visitedEdgeCounts, visitedEdgeFirstPos);
            waypointStack.remove(waypointStack.size() - 1);
            currentMwp = waypointStack.get(waypointStack.size() - 1);
            currentIlon = currentMwp.crosspoint.getILon();
            currentIlat = currentMwp.crosspoint.getILat();
            prevIlon = savedPrevIlon;
            prevIlat = savedPrevIlat;
            localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_TOO_LONG);
            continue;
          }

          // Between (1-tol) and (1+tol) but not within tol? → too short, continue
        }

        candidateFound = true;
        break;
      }

      if (!candidateFound) {
        result.addDiagnostic("step " + step + ": exhausted all " + maxAttempts + " attempts");
        break;
      }
    }

    if (bestFallback != null) {
      populateResult(result, bestFallback.track, bestFallback.waypointStack, start,
        startMwp, bestFallback.legTracks, desiredDistance, startDirection);
      result.setTotalDistanceMeters(bestFallback.track.distance);
      result.setWithinTolerance(false);
      String reject = qualityGateReason(bestFallback.track, desiredDistance);
      String reason = "best error=" + String.format("%.1f%%", bestFallback.error * 100);
      result.setFallbackReason(reject == null ? reason : DEGRADED_FALLBACK_PREFIX + reject + "; " + reason);
      result.setSubRoutesChosen(bestFallback.legTracks.size());
      result.setAttemptsUsed(totalAttempts);
      stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
      return result;
    }

    // Last resort: force-close. Allow up to 10s here even past the planner
    // deadline — without a closing leg the planner has nothing usable to return.
    if (!segments.isEmpty()) {
      long forceCloseDeadline = Math.max(deadline, System.currentTimeMillis() + SUB_ROUTE_TIMEOUT_MS);
      OsmTrack returnTrack = timedFindTrack("greedy-force-close",
        currentMwp, startMwp, buildRefTrack(segments), forceCloseDeadline);
      returnChecksPerformed++;
      if (returnTrack != null && returnTrack.distance > 0) {
        segments.add(returnTrack);
        OsmTrack finalTrack = mergeSegments(segments, null);
        populateResult(result, finalTrack, waypointStack, start, startMwp, segments, desiredDistance, startDirection);
        result.setTotalDistanceMeters(finalTrack.distance);
        result.setWithinTolerance(false);
        String reject = qualityGateReason(finalTrack, desiredDistance);
        result.setFallbackReason(reject == null ? "forced closure" : DEGRADED_FALLBACK_PREFIX + reject + "; forced closure");
        result.setSubRoutesChosen(segments.size());
        result.setAttemptsUsed(totalAttempts);
        stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
        return result;
      }
    }

    result.setFallbackReason("could not build any loop");
    stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
    return result;
  }

  /**
   * Returns {@code null} if {@code track} meets quality bars (distance ratio,
   * reuse, closure); otherwise returns a short human-readable reason. Used to
   * tag fallback / forced-closure results so the caller can choose to demote
   * them rather than ship as success.
   */
  private String qualityGateReason(OsmTrack track, double desiredDistance) {
    if (track == null || track.nodes == null || track.nodes.size() < 4) return "no track";
    double ratio = track.distance / desiredDistance;
    if (ratio < FALLBACK_MIN_RATIO || ratio > FALLBACK_MAX_RATIO) {
      return String.format(java.util.Locale.US, "ratio=%.2f outside [%.1f,%.1f]",
        ratio, FALLBACK_MIN_RATIO, FALLBACK_MAX_RATIO);
    }
    double reuse = finalTrackReuseRatio(track);
    if (reuse > FALLBACK_MAX_REUSE) {
      return String.format(java.util.Locale.US, "reuse=%.0f%% exceeds %.0f%%",
        reuse * 100, FALLBACK_MAX_REUSE * 100);
    }
    OsmPathElement first = track.nodes.get(0);
    OsmPathElement last = track.nodes.get(track.nodes.size() - 1);
    int closure = first.calcDistance(last);
    if (closure > FALLBACK_MAX_CLOSURE_M) {
      return "closure=" + closure + "m exceeds " + FALLBACK_MAX_CLOSURE_M + "m";
    }
    return null;
  }

  /**
   * Distance-share of the track on edges traversed more than once — matches
   * {@link LoopQualityMetrics#computeRoadReusePercent}'s definition (first
   * visit not reuse; subsequent visits ARE reuse). Self-contained (does not
   * depend on the planner's running visit counts); use this when evaluating
   * the FINAL loop, not when scoring per-step candidates.
   */
  static double finalTrackReuseRatio(OsmTrack track) {
    if (track == null || track.nodes == null || track.nodes.size() < 2) return 0.0;
    Map<Long, Integer> localCounts = new HashMap<>();
    double total = 0;
    double reused = 0;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = a.calcDistance(b);
      total += segLen;
      long key = edgeKey(a, b);
      int prev = localCounts.merge(key, 1, Integer::sum);
      if (prev > 1) reused += segLen;
    }
    return total > 0 ? reused / total : 0.0;
  }

  private static void stampTelemetry(RoundTripResult result, long planStart,
                                     int candidatesGenerated, int candidatesRouted,
                                     int returnChecksPerformed) {
    result.setCandidatesGenerated(candidatesGenerated);
    result.setCandidatesRouted(candidatesRouted);
    result.setReturnChecksPerformed(returnChecksPerformed);
    result.setRuntimeMillis(System.currentTimeMillis() - planStart);
  }

  private void populateResult(RoundTripResult result, OsmTrack track,
    List<MatchedWaypoint> waypointStack, OsmNodeNamed start,
    MatchedWaypoint startMwp, List<OsmTrack> segments,
    double desiredDistance, double startDirection) {
    result.setTrack(track);
    result.setLoopWaypoints(buildLoopWaypoints(waypointStack, start));
    result.setMatchedWaypoints(buildMatchedWaypoints(waypointStack, startMwp));
    result.setLegTracks(new ArrayList<>(segments));
    // Compute the full quality metrics once and surface them: the reuse ratio
    // drives the result field, and the complete metric set is recorded as a
    // diagnostic so API callers can inspect loop quality instead of it being
    // computed and discarded.
    if (track != null && track.nodes != null && track.nodes.size() >= 2) {
      LoopQualityMetrics metrics = LoopQualityMetrics.compute(track, (int) desiredDistance, startDirection);
      result.setReusedEdgeRatio(metrics.getRoadReusePercent() / 100.0);
      result.addDiagnostic("quality: " + metrics);
    } else {
      result.setReusedEdgeRatio(0.0);
    }
  }

  /**
   * Pick up to {@code k} candidates from the score-sorted list with angular
   * spread: walk in score order, accept each pick if it is at least
   * {@link #MIN_ANGULAR_SEPARATION_DEG} away from every previously picked
   * candidate's bearing. If diversity culling leaves fewer than {@code k},
   * back-fill with the next best-scored candidates regardless of spread so
   * we never under-budget.
   */
  static List<RoundTripCandidateProvider.CandidatePoint> pickDiverseTopK(
    List<RoundTripCandidateProvider.CandidatePoint> sorted, int k) {
    List<RoundTripCandidateProvider.CandidatePoint> picked = new ArrayList<>(k);
    for (RoundTripCandidateProvider.CandidatePoint cp : sorted) {
      if (picked.size() >= k) break;
      boolean farEnough = true;
      for (RoundTripCandidateProvider.CandidatePoint other : picked) {
        if (CheapAngleMeter.getDifferenceFromDirection(cp.bearing, other.bearing)
            < MIN_ANGULAR_SEPARATION_DEG) {
          farEnough = false;
          break;
        }
      }
      if (farEnough) picked.add(cp);
    }
    if (picked.size() < k) {
      for (RoundTripCandidateProvider.CandidatePoint cp : sorted) {
        if (picked.size() >= k) break;
        if (!picked.contains(cp)) picked.add(cp);
      }
    }
    return picked;
  }

  // --- Routing with timeout ---

  /**
   * Routes from→to with a per-call timeout = min(SUB_ROUTE_TIMEOUT_MS, deadline - now).
   * Returns {@code null} if the remaining budget is below {@link #MIN_FIND_TRACK_MS}.
   */
  private OsmTrack timedFindTrack(String name, MatchedWaypoint from, MatchedWaypoint to,
                                  OsmTrack refTrack, long deadline) {
    long now = System.currentTimeMillis();
    long remaining = deadline - now;
    if (remaining < MIN_FIND_TRACK_MS) {
      engine.logInfo(name + ": deadline exceeded, skipping (remaining " + remaining + "ms)");
      return null;
    }
    long budget = Math.min(SUB_ROUTE_TIMEOUT_MS, remaining);
    long savedStartTime = engine.startTime;
    long savedMaxRunningTime = engine.maxRunningTime;
    try {
      engine.startTime = now;
      engine.maxRunningTime = budget;
      return engine.findTrack(name, from, to, null, refTrack, false);
    } catch (IllegalArgumentException e) {
      engine.logInfo(name + ": no track (" + e.getMessage() + ")");
      return null;
    } finally {
      engine.startTime = savedStartTime;
      engine.maxRunningTime = savedMaxRunningTime;
    }
  }

  // --- Waypoint matching ---

  private MatchedWaypoint matchPoint(int ilon, int ilat, String name) {
    try {
      engine.resetCache(false);
      MatchedWaypoint mwp = new MatchedWaypoint();
      mwp.waypoint = new OsmNode(ilon, ilat);
      mwp.name = name;
      List<MatchedWaypoint> mwpList = new ArrayList<>();
      mwpList.add(mwp);
      engine.nodesCache.matchWaypointsToNodes(mwpList, 2000, engine.islandNodePairs);
      if (mwp.crosspoint == null || mwp.node1 == null || mwp.node2 == null) {
        return null;
      }
      return mwp;
    } catch (Exception e) {
      engine.logInfo("matchPoint(" + name + ") failed: " + e.getMessage());
      return null;
    }
  }

  // --- Track management ---

  private OsmTrack buildRefTrack(List<OsmTrack> segments) {
    if (segments.isEmpty()) return null;
    return mergeSegments(segments, null);
  }

  private OsmTrack mergeSegments(List<OsmTrack> segments, OsmTrack finalSegment) {
    OsmTrack merged = new OsmTrack();
    for (OsmTrack seg : segments) {
      appendTrack(merged, seg);
    }
    if (finalSegment != null) {
      appendTrack(merged, finalSegment);
    }
    merged.buildMap();
    return merged;
  }

  private void appendTrack(OsmTrack target, OsmTrack source) {
    if (source.nodes == null) return;
    boolean first = true;
    for (OsmPathElement node : source.nodes) {
      if (first && !target.nodes.isEmpty()) {
        OsmPathElement last = target.nodes.get(target.nodes.size() - 1);
        if (last.getILon() == node.getILon() && last.getILat() == node.getILat()) {
          first = false;
          continue;
        }
      }
      first = false;
      target.nodes.add(node);
    }
    target.distance += source.distance;
    target.ascend += source.ascend;
    target.cost += source.cost;
  }

  // --- Visited edge tracking (ref-counted) ---

  private void addVisitedEdges(OsmTrack track, Map<Long, Integer> edgeCounts,
                               Map<Long, Double> edgeFirstPos, double trackStartCumDist) {
    if (track.nodes == null || track.nodes.size() < 2) return;
    double cumDist = trackStartCumDist;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = a.calcDistance(b);
      long key = edgeKey(a, b);
      Integer prev = edgeCounts.get(key);
      if (prev == null || prev == 0) {
        // First visit ever — record the segment midpoint as the first-visit
        // cumulative distance, used downstream for boundary-proximity weighting.
        edgeFirstPos.put(key, cumDist + segLen / 2);
      }
      edgeCounts.merge(key, 1, Integer::sum);
      cumDist += segLen;
    }
  }

  private void removeVisitedEdges(OsmTrack track, Map<Long, Integer> edgeCounts,
                                  Map<Long, Double> edgeFirstPos) {
    if (track.nodes == null || track.nodes.size() < 2) return;
    for (int i = 1; i < track.nodes.size(); i++) {
      long key = edgeKey(track.nodes.get(i - 1), track.nodes.get(i));
      Integer count = edgeCounts.get(key);
      if (count == null) continue;
      if (count <= 1) {
        edgeCounts.remove(key);
        edgeFirstPos.remove(key);
      } else {
        edgeCounts.put(key, count - 1);
        // firstPos stays — earlier visit(s) still present in the route.
      }
    }
  }

  /**
   * Position-weighted distance-share reuse ratio. Sum of (segment-length ×
   * position-penalty) of reused edges divided by total track length. Matches
   * {@link LoopQualityMetrics}'s distance-weighted definition for the
   * unweighted case ({@link #BOUNDARY_PROXIMITY_FRAC} = 0) and adds a
   * boundary-proximity multiplier: reuse where either the first visit or the
   * current re-visit is within {@link #BOUNDARY_PROXIMITY_FRAC} of the loop's
   * start or end gets full weight (1.0); mid-loop reuse gets reduced weight
   * ({@link #MID_LOOP_REUSE_WEIGHT}). Implements the cyclist's intuition that
   * back-and-forth near start/end is much more annoying than mid-loop reuse.
   *
   * @param trackStartCumDist cumulative loop distance at the start of {@code track}
   * @param desiredDistance   target total loop distance (for proximity normalisation)
   */
  private double computeTrackVisitedRatio(OsmTrack track, Map<Long, Integer> edgeCounts,
                                          Map<Long, Double> edgeFirstPos,
                                          double trackStartCumDist, double desiredDistance) {
    if (edgeCounts.isEmpty() || track.nodes == null || track.nodes.size() < 2) return 0.0;
    double total = 0;
    double weightedReuse = 0;
    double cumDist = trackStartCumDist;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = a.calcDistance(b);
      double midPos = cumDist + segLen / 2;
      total += segLen;
      long key = edgeKey(a, b);
      if (edgeCounts.containsKey(key)) {
        Double firstPos = edgeFirstPos.get(key);
        double posWeight = (firstPos != null)
          ? boundaryProximityWeight(firstPos, midPos, desiredDistance)
          : 1.0;
        weightedReuse += segLen * posWeight;
      }
      cumDist += segLen;
    }
    return total > 0 ? weightedReuse / total : 0.0;
  }

  /** Fraction of desired distance defining "near start or end" for back-and-forth weighting. */
  private static final double BOUNDARY_PROXIMITY_FRAC = 0.20;
  /** Reuse weight for mid-loop overlap (vs 1.0 for near-boundary). */
  private static final double MID_LOOP_REUSE_WEIGHT = 0.5;

  /**
   * Returns 1.0 when either {@code firstPos} or {@code currentPos} is within
   * {@link #BOUNDARY_PROXIMITY_FRAC} of loop start (0) or loop end
   * (desiredDistance); {@link #MID_LOOP_REUSE_WEIGHT} when both are mid-loop.
   * Matches the cyclist's intuition: visible/annoying retraces are at the
   * boundaries; mid-loop crossings are often unavoidable and barely noticed.
   */
  static double boundaryProximityWeight(double firstPos, double currentPos, double desiredDistance) {
    if (desiredDistance <= 0) return 1.0;
    double firstFrac = firstPos / desiredDistance;
    double currentFrac = currentPos / desiredDistance;
    double firstBoundary = Math.min(Math.max(0, firstFrac), Math.max(0, 1 - firstFrac));
    double currentBoundary = Math.min(Math.max(0, currentFrac), Math.max(0, 1 - currentFrac));
    double minBoundary = Math.min(firstBoundary, currentBoundary);
    return (minBoundary < BOUNDARY_PROXIMITY_FRAC) ? 1.0 : MID_LOOP_REUSE_WEIGHT;
  }

  private static long edgeKey(OsmPathElement a, OsmPathElement b) {
    long idA = a.getIdFromPos();
    long idB = b.getIdFromPos();
    long lo = Math.min(idA, idB);
    long hi = Math.max(idA, idB);
    return lo ^ (hi * 0x9E3779B97F4A7C15L);
  }


  /**
   * Convert the waypoint stack (MatchedWaypoints) to a list of OsmNodeNamed
   * forming a closed loop: [start, wp1, wp2, ..., closing_point].
   * The closing point is a copy of start to form the return leg.
   */
  private List<OsmNodeNamed> buildLoopWaypoints(List<MatchedWaypoint> stack, OsmNodeNamed start) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    // First waypoint = road-snapped start position (crosspoint, not raw user position).
    // Using the crosspoint avoids beeline segments when the user's click position
    // is far from a road (park, water, etc.).
    MatchedWaypoint startMwp = stack.get(0);
    OsmNodeNamed from = new OsmNodeNamed(new OsmNode(
      startMwp.crosspoint.getILon(), startMwp.crosspoint.getILat()));
    from.name = "from";
    wps.add(from);
    // Intermediate waypoints from the stack (skip first which is start)
    for (int i = 1; i < stack.size(); i++) {
      MatchedWaypoint mwp = stack.get(i);
      OsmNodeNamed via = new OsmNodeNamed(new OsmNode(
        mwp.crosspoint.getILon(), mwp.crosspoint.getILat()));
      via.name = "via" + i;
      wps.add(via);
    }
    // Closing waypoint = same road-snapped start position
    OsmNodeNamed to = new OsmNodeNamed(new OsmNode(
      startMwp.crosspoint.getILon(), startMwp.crosspoint.getILat()));
    to.name = "to";
    wps.add(to);
    return wps;
  }

  /**
   * Build a list of pre-matched waypoints for the final routing pass.
   * Preserves node1/node2/crosspoint from the greedy planner's matching,
   * so doRouting() skips re-matching and uses the exact same road segments.
   * The start and closing waypoints are re-matched from the original start MWP.
   */
  List<MatchedWaypoint> buildMatchedWaypoints(
    List<MatchedWaypoint> stack, MatchedWaypoint startMwp) {

    List<MatchedWaypoint> mwps = new ArrayList<>();

    // Start point — use original match
    MatchedWaypoint fromMwp = copyMatchedWaypoint(startMwp, "from");
    mwps.add(fromMwp);

    // Intermediate waypoints — preserve exact matching from greedy planning
    for (int i = 1; i < stack.size(); i++) {
      MatchedWaypoint mwp = stack.get(i);
      MatchedWaypoint viaMwp = copyMatchedWaypoint(mwp, "via" + i);
      mwps.add(viaMwp);
    }

    // Closing point — same match as start
    MatchedWaypoint toMwp = copyMatchedWaypoint(startMwp, "to");
    mwps.add(toMwp);

    return mwps;
  }

  MatchedWaypoint copyMatchedWaypoint(MatchedWaypoint src, String name) {
    MatchedWaypoint copy = new MatchedWaypoint();
    copy.node1 = new OsmNode(src.node1.ilon, src.node1.ilat);
    copy.node2 = new OsmNode(src.node2.ilon, src.node2.ilat);
    // Snap to a graph node — mid-edge crosspoints cause leg gaps because
    // routing reaches the nearest node, not the interpolated position.
    OsmNode snapped = snapToNearest(src.crosspoint, copy.node1, copy.node2);
    copy.crosspoint = new OsmNode(snapped.ilon, snapped.ilat);
    // waypoint == crosspoint keeps RoutingEngine#matchWaypointsToNodes from
    // taking the dynamic beeline-insertion path (gated on snap > catchingRange).
    copy.waypoint = new OsmNode(snapped.ilon, snapped.ilat);
    copy.name = name;
    // Round-trip no-beeline invariant: greedy points must never be DIRECT.
    copy.wpttype = MatchedWaypoint.WAYPOINT_TYPE_SHAPING;
    return copy;
  }

  private OsmNode snapToNearest(OsmNode crosspoint, OsmNode node1, OsmNode node2) {
    int d1 = crosspoint.calcDistance(node1);
    int d2 = crosspoint.calcDistance(node2);
    return d1 <= d2 ? node1 : node2;
  }

  private DirectionPreference nearestDirectionPreference(double bearing) {
    bearing = CheapAngleMeter.normalize(bearing);
    DirectionPreference best = DirectionPreference.ANY;
    double minDiff = Double.MAX_VALUE;
    for (DirectionPreference dp : DirectionPreference.values()) {
      if (dp == DirectionPreference.ANY) continue;
      double diff = CheapAngleMeter.getDifferenceFromDirection(dp.bearing, bearing);
      if (diff < minDiff) {
        minDiff = diff;
        best = dp;
      }
    }
    return best;
  }

  /**
   * Combine the routed scorer score with cost-per-meter so candidate selection
   * accounts for both route shape (visited reuse, distance, loop feasibility)
   * and road quality (cost).
   */
  static double combinedRoutedScore(double scorerScore, double costPerMeter) {
    return scorerScore + COST_PER_METER_WEIGHT * costPerMeter;
  }

  /**
   * Capture an immutable view of the fallback candidate so later mutations of
   * {@code segments} / {@code waypointStack} do not desync the track from the
   * recorded waypoints and leg list.
   */
  private Snapshot snapshotFallback(List<OsmTrack> segments, OsmTrack returnTrack,
                                    List<MatchedWaypoint> waypointStack, double error) {
    Snapshot snap = new Snapshot();
    snap.track = mergeSegments(segments, returnTrack);
    snap.waypointStack = new ArrayList<>(waypointStack);
    snap.legTracks = new ArrayList<>(segments);
    snap.legTracks.add(returnTrack);
    snap.error = error;
    return snap;
  }

  private static final class Snapshot {
    OsmTrack track;
    List<MatchedWaypoint> waypointStack;
    List<OsmTrack> legTracks;
    double error;
  }

  /** A candidate that has been routed. */
  private static final class ScoredRoute {
    OsmTrack track;
    MatchedWaypoint toMwp;
    double routeDistance;
    double visitedRatio;
  }
}
