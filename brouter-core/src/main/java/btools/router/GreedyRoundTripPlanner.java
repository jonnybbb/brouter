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
  /**
   * Weight applied per self-intersection introduced by a tentative partial
   * loop. This is a placement-side signal: among otherwise similar routed
   * candidates, prefer the one that keeps the loop geometry clean before the
   * final hard gate sees the completed route.
   */
  // Phase 2.2 chaos-avoidance tuning. Raised from 0.3 → 1.0 per the
  // directive "zick zack and chaos routing must be avoided" — at 0.3
  // a candidate with 1 tentative crossing got a +0.3 score bump, which
  // got dominated by other terms; at 1.0 even one crossing pushes the
  // candidate substantially down the ranking. The 880-scenario corpus
  // measurement validates this is empirically the right magnitude:
  // weight=2.0 was measured but OVER-penalizes — it forces the planner
  // to pick candidates with 0 tentative crossings whose closed loops
  // chaos-out via different geometry, raising chaotic-loop count by
  // +11 vs weight=1.0 (production chaotic 40 → 51).
  static final double PARTIAL_SELF_INTERSECTION_WEIGHT = 1.0;
  // Multiplier applied to the air-distance return estimate when deciding
  // whether to skip the return Dijkstra. > 1 means we skip less aggressively.
  private static final double RETURN_SKIP_SAFETY = 1.5;

  /**
   * Phase 2 v3 diagnostic: when {@code -Dgreedy.diagnostic=true} is set,
   * the planner emits one line per scored candidate showing the routed
   * sub-track's distance, worst contiguous hostile stretch, and the
   * final routedScore. Used offline to investigate why borderline
   * scenarios reject — does the candidate pool contain hostility-
   * avoiding alternatives, or do they all hit the same stretch?
   */
  private static final boolean DIAGNOSTIC =
    Boolean.parseBoolean(System.getProperty("greedy.diagnostic", "false"));

  /**
   * Hoisted ranking comparators. Both are pure (capture no state) and use
   * {@link Comparator#comparingDouble}'s {@link Double#compare} semantics, so a
   * shared static instance ranks identically to a per-call allocation while
   * avoiding a comparator + lambda allocation on every attempt. {@code List.sort}
   * is stable, so equal-key ties still resolve by pre-sort insertion order.
   */
  private static final Comparator<RoundTripCandidateProvider.CandidatePoint> BY_HEURISTIC_SCORE =
    Comparator.comparingDouble(c -> c.score);
  private static final Comparator<ScoredRoute> BY_ROUTED_SCORE =
    Comparator.comparingDouble(c -> c.routedScore);

  private final RoutingEngine engine;
  private final CandidateScorer scorer;
  private final RoundTripCandidateProvider candidateProvider;

  private final int subRouteCount;
  private final double tolerance;
  private final int maxAttempts;

  /**
   * Active profile name, set by {@link RoutingEngine} before planning.
   * The planner's internal {@link #qualityGateReason fallback gate}
   * forwards to {@link RoundTripQualityGate#evaluate}, which needs the
   * profile name to apply the paved-vs-other hostility branch correctly.
   * When null (back-compat for older direct callers) the gate uses
   * profile-agnostic defaults.
   */
  private String profileName;

  /**
   * Set the active profile name. Should be called by {@link RoutingEngine}
   * during planner construction, immediately after the planner is
   * instantiated, so the internal fallback gate matches what the
   * production gate will evaluate downstream.
   */
  public void setProfileName(String profileName) {
    this.profileName = profileName;
  }

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
   * Enable iso-hostility scoring on the scorer. Only call this for paved
   * profiles whose typical {@code costFromStart/airDist} is close to 1.0;
   * other profiles (gravel, MTB) have baselines around 9 and would have
   * every candidate flagged as hostile. See {@link CandidateScorer#setHostilityActive}.
   */
  public void setHostilityActive(boolean active) {
    scorer.setHostilityActive(active);
  }

  /**
   * Plan a greedy round-trip loop.
   */
  public RoundTripResult plan(OsmNodeNamed start, double desiredDistance, double startDirection) {
    long planStart = System.currentTimeMillis();
    long deadline = planStart + DEFAULT_PLAN_DEADLINE_MS;
    RoundTripResult result = new RoundTripResult();
    double subTarget = desiredDistance / subRouteCount;
    // SAFE-3: primitive open-addressing store replacing the former
    // HashMap<Long,Integer> reuse counts + HashMap<Long,Double> first-visit
    // positions. The two were always maintained in lock-step, so they fold
    // into one boxing-free table. It is only ever point-queried (never
    // iterated), so slot layout cannot affect any routing decision.
    VisitedEdgeStore visitedEdges = new VisitedEdgeStore();
    List<OsmTrack> segments = new ArrayList<>();
    int totalAttempts = 0;
    double totalDistance = 0;
    int candidatesGenerated = 0;
    int candidatesRouted = 0;
    int returnChecksPerformed = 0;
    // Auto-quality-redesign §132: track start-iso vs non-start-iso candidates
    // separately. The public telemetry fields still say "radial" for
    // compatibility; in production GREEDY those counters now represent
    // per-step graph-native candidates.
    // Candidate source is identified via the existing
    // {@link RoundTripCandidateProvider.CandidatePoint#costFromStart} sentinel:
    // a start-iso candidate has costFromStart != NO_ISO_COST; per-step
    // graph-native and legacy radial candidates use the sentinel.
    int routedIso = 0;
    int routedRadial = 0;
    int acceptedIsoLegs = 0;
    int acceptedRadialLegs = 0;

    MatchedWaypoint startMwp = matchPoint(start.ilon, start.ilat, "greedy_start");
    if (startMwp == null) {
      result.setFallbackReason("start point not on road network");
      stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed, routedIso, routedRadial, acceptedIsoLegs, acceptedRadialLegs);
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
            startDirection,
            cachedRefTrack);
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
        candidates.sort(BY_HEURISTIC_SCORE);

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

        // Phase 1 Step 2: keep a ranked list of routed candidates instead of
        // a single best-pick. Step 2 is structural and behavior-preserving —
        // we still commit only the top-ranked candidate at the end. Step 3
        // (closure-aware trial loop) will iterate this list when the locally
        // best candidate's closed loop is rejected.
        List<ScoredRoute> routedCandidates = new ArrayList<>();
        int routeAttempts = toRoute.size();
        MatchedWaypoint fromMwp = currentMwp;

        // SAFE-4: merge the committed segments into a prefix node list ONCE per
        // attempt and share it (read-only) across every routed candidate's
        // tentative self-intersection count, instead of re-merging the whole
        // prefix per candidate. segments is not mutated inside the r-loop.
        List<OsmPathElement> committedPrefixNodes =
          segments.isEmpty() ? null : mergeSegmentsNoMap(segments, null).nodes;

        for (int r = 0; r < routeAttempts; r++) {
          RoundTripCandidateProvider.CandidatePoint cp = toRoute.get(r);

          MatchedWaypoint toMwp = matchPoint(cp.ilon, cp.ilat, "greedy_to");
          if (toMwp == null) continue;

          // Snap distance from the candidate coordinate to its routed-on-road
          // crosspoint. Reject candidates that snapped too far away.
          int snappedIlon = toMwp.crosspoint.getILon();
          int snappedIlat = toMwp.crosspoint.getILat();
          double snapDist = CheapRuler.distance(cp.ilon, cp.ilat, snappedIlon, snappedIlat);
          if (snapDist > airRadius * 0.5) continue;

          OsmTrack subTrack = cp.routedTrack;
          if (subTrack == null) {
            subTrack = timedFindTrack("greedy-sub", fromMwp, toMwp, cachedRefTrack, deadline);
          }
          candidatesRouted++;
          // Phase 2 v3 deliberate compromise: do NOT retrack candidate
          // sub-tracks here, even though it would give the scorer's
          // worst-contiguous signal real data. Retracking every
          // candidate (3 cands × 5 steps = ~15 per loop) inflates total
          // runtime ~40×. Empirically, Phase 2 v2 measurement showed the
          // scorer-level signal moves at most 0-1 pp of pass-rate. The
          // gate-side win comes from detailing ACCEPTED legs (below) so
          // the gate sees real metadata; candidate-level detail is
          // future work if it ever becomes the bottleneck.
          // Source-aware telemetry: start-iso candidates carry a non-sentinel
          // costFromStart; graph-native/non-start-iso candidates use NO_ISO_COST. We count
          // BEFORE the null/zero-distance guard so "routed" reflects what
          // Dijkstra attempted, not what succeeded.
          boolean isIsoCandidate =
            cp.costFromStart != RoundTripCandidateProvider.NO_ISO_COST;
          if (isIsoCandidate) routedIso++; else routedRadial++;
          if (subTrack == null || subTrack.distance == 0) continue;

          // Recompute scoring inputs from the SNAPPED endpoint (toMwp.crosspoint).
          // The router actually travels to that snapped location, not the raw
          // candidate point — so air-distance, bearing, return estimate, and the
          // overlong-route reject threshold should all reflect what was routed.
          double snappedAirDistFromCurrent = CheapRuler.distance(
            currentIlon, currentIlat, snappedIlon, snappedIlat);
          if (subTrack.distance > snappedAirDistFromCurrent * 3.0) continue;

          // SAFE-5: computeTrackVisitedRatio and the paved-profile
          // worst-contiguous scan below both iterate subTrack.nodes calling
          // a.calcDistance(b) over the identical segments in the identical
          // orientation. On paved profiles (where both run) compute the
          // per-segment integer distances ONCE and feed both passes, halving
          // the CheapRuler sqrt+round calls. calcDistance returns an int, so
          // the cached value widened to double is bit-identical to recomputing
          // it. Non-paved profiles run only the first pass, so they keep the
          // inline computation (no buffer to share).
          boolean pavedProfile = RoundTripQualityGate.isPavedProfile(profileName);
          int[] segLens = pavedProfile ? segmentDistances(subTrack) : null;
          double actualVisitedRatio = computeTrackVisitedRatio(subTrack,
            visitedEdges, totalDistance, desiredDistance, segLens);
          double airDistToStart = CheapRuler.distance(snappedIlon, snappedIlat, start.ilon, start.ilat);
          double estimatedReturn = airDistToStart * ROAD_INDIRECTNESS;
          double distFromPrevious = (prevIlon >= 0)
            ? CheapRuler.distance(prevIlon, prevIlat, snappedIlon, snappedIlat) * ROAD_INDIRECTNESS
            : -1;
          double snappedBearing = CheapRuler.getScaledBearing(
            currentIlon, currentIlat, snappedIlon, snappedIlat);

          // Phase 2 v2: feed the routed sub-track's worst contiguous
          // hostile stretch to the scorer. This mirrors the gate's
          // physical-experience metric (a single long unbroken off-road
          // stretch is the cyclist's complaint surface). Phase 2.1's
          // averaged cost/distance ratio was the wrong signal — diagnostic
          // data showed 99% of fastbike rejections come from contiguous-
          // stretch trips, but leg-averages dilute single bad stretches
          // across surrounding clean kilometres. Worst-contiguous is a
          // max over edges, the same shape as the gate.
          //
          // Computed only for paved profiles (the hostile predicate is
          // road-bike specific); -1 sentinel for the rest keeps the
          // scorer on its iso-hostility fall-back.
          //
          // Scorer-side approximation: the gate's worstContiguousHostileMetersPaved
          // returns 0 on single-pass subTracks because it skips edges with
          // null wayKeyValues (the tag check is the dominant hostility signal,
          // costfactor>4.0 only catches extreme cases). Use the costfactor-
          // only variant with the lower SCORER_HOSTILE_COSTFACTOR_THRESHOLD
          // to get a usable signal on single-pass tracks. The gate's precise
          // tag-aware check still runs post-detail before commit.
          int worstHostile = pavedProfile
            ? RoundTripQualityGate.worstContiguousCostlyMetersForScorer(subTrack, segLens)
            : -1;

          double routedScorerScore = scorer.score(
            subTrack.distance, subTarget,
            totalDistance, estimatedReturn, desiredDistance,
            snappedBearing, dirPref,
            step, subRouteCount,
            actualVisitedRatio,
            airDistToStart, searchRadius,
            distFromPrevious,
            cp.costFromStart, cp.bucketHits, cp.sourceContour,
            worstHostile);

          double costPerMeter = (double) subTrack.cost / subTrack.distance;
          double routedScore = combinedRoutedScore(routedScorerScore, costPerMeter);
          int tentativeSelfIntersections = countTentativeSelfIntersections(committedPrefixNodes, subTrack);
          if (tentativeSelfIntersections > 0) {
            routedScore += PARTIAL_SELF_INTERSECTION_WEIGHT * tentativeSelfIntersections;
          }

          if (DIAGNOSTIC) {
            System.err.printf(
              "[greedy-diag] step=%d cand=%d dist=%d worstHost=%d selfX=%d costPerM=%.3f score=%.3f%n",
              step, r, subTrack.distance, worstHostile, tentativeSelfIntersections,
              costPerMeter, routedScore);
          }

          ScoredRoute candidate = new ScoredRoute();
          candidate.track = subTrack;
          candidate.toMwp = toMwp;
          candidate.routeDistance = subTrack.distance;
          candidate.visitedRatio = actualVisitedRatio;
          candidate.fromIsoCandidate = isIsoCandidate;
          candidate.routedScore = routedScore;
          candidate.candidateIndex = r;
          candidate.tentativeSelfIntersections = tentativeSelfIntersections;
          candidate.routedLegWorstHostileMeters = worstHostile;
          routedCandidates.add(candidate);
        }

        sortByRoutedScore(routedCandidates);
        ScoredRoute accepted = routedCandidates.isEmpty() ? null : routedCandidates.get(0);

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
        // Phase 2 v3: upgrade the committed sub-track from single-pass
        // (fast, no per-edge MessageData) to detailed via the engine's
        // retracking pass. The quality gate's paved-profile hostility
        // check requires wayKeyValues on every edge; single-pass tracks
        // don't have them, so without this step the gate would either
        // bypass hostility (under suspect-tolerance) or trip the
        // missing-metadata floor. One Dijkstra per committed leg (5-6
        // per loop) — negligible vs the candidate scoring loop above.
        // SAFE-6: reuse cachedRefTrack instead of rebuilding it. segments is
        // not mutated between its construction (top of step) and here:
        // segments.add happens below, and every attempt-loop continue path
        // that could re-reach this point either never added a leg or added
        // then undid it, restoring step-start content. Routing/retrack treat
        // the refTrack as read-only (a fresh OsmTrack is built internally),
        // which the code already relies on by reusing cachedRefTrack across
        // all candidate sub-routes — so the merged content here is identical.
        OsmTrack refBeforeAccept = cachedRefTrack;
        OsmTrack detailedAccepted = detailAcceptedTrack(accepted, fromMwp, refBeforeAccept, deadline);
        if (detailedAccepted == null || detailedAccepted.distance == 0) {
          result.addDiagnostic("step " + step + ": accepted leg could not be detailed, retrying");
          localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_NO_CANDIDATE);
          continue;
        }
        if (metadataMissingTooHigh(detailedAccepted)) {
          result.addDiagnostic("step " + step + ": accepted leg still lacks metadata after retrack ("
            + formatPct(RoundTripQualityGate.missingMetadataFraction(detailedAccepted)) + "), retrying");
          localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_NO_CANDIDATE);
          continue;
        }
        // Phase 2 v3 hostility post-check. The scorer cannot see hostility
        // while choosing candidates (single-pass tracks lack metadata),
        // but the FINAL gate will reject any leg with a contiguous hostile
        // stretch over the cap. Doing the check here lets the planner
        // backoff + retry with a different candidate instead of
        // committing to a hostile leg and losing the whole loop. Skipped
        // on non-paved profiles where the predicate would over-flag.
        if (RoundTripQualityGate.isPavedProfile(profileName)) {
          RoundTripQualityGate.HostileStretch hostileStretch =
            RoundTripQualityGate.worstHostileStretchPaved(detailedAccepted);
          if (hostileStretch.meters > RoundTripQualityGate.MAX_CONTIGUOUS_HOSTILE_METERS) {
            result.addDiagnostic("step " + step + ": accepted leg has " + hostileStretch.meters
              + "m contiguous hostile stretch (over " + RoundTripQualityGate.MAX_CONTIGUOUS_HOSTILE_METERS
              + "), retrying with smaller radius");
            if (DIAGNOSTIC) {
              System.err.printf("[greedy-diag] accepted hostile stretch step=%d %s%n",
                step, hostileStretch.describe());
            }
            localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_NO_CANDIDATE);
            continue;
          }
        }
        accepted.track = detailedAccepted;
        accepted.routeDistance = detailedAccepted.distance;
        addVisitedEdges(accepted.track, visitedEdges, totalDistance);
        segments.add(accepted.track);
        totalDistance += accepted.routeDistance;
        if (accepted.fromIsoCandidate) acceptedIsoLegs++;
        else acceptedRadialLegs++;

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

          // Phase 2 v3: detail the closing return leg before either snapshot
          // or final commit — both paths feed the quality gate which needs
          // per-edge MessageData.
          // Detail the closing return leg before snapshotting or committing —
          // both feed the quality gate, which needs per-edge MessageData. Also
          // re-detail when the current best fallback was gate-rejected, so we
          // keep searching for a gate-accepted closure even at higher error.
          boolean needDetail = (bestFallback == null || error < bestFallback.error)
            || (error <= tolerance)
            || (bestFallback != null && !bestFallback.gateAccepted);
          if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
            needDetail = true;
          }
          if (needDetail) {
            // retrackForDetail ignores the refTrack arg (its guide track already
            // fixes the node sequence), so don't pay for a buildRefTrack merge here.
            returnTrack = engine.retrackForDetail(returnTrack, currentMwp, startMwp, null);
          }
          if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
            RoundTripQualityGate.HostileStretch returnHostile =
              RoundTripQualityGate.worstHostileStretchPaved(returnTrack);
            System.err.printf(
              "[greedy-diag] returnLeg step=%d dist=%d worstHost=%d missingMeta=%.1f%% closedDist=%d err=%.3f stretch=%s%n",
              step, returnTrack.distance, returnHostile.meters,
              RoundTripQualityGate.missingMetadataFraction(returnTrack) * 100.0,
              (int) closedDistance, error, returnHostile.describe());
          }

          // Build the closed loop and evaluate the production gate once (only
          // meaningful when the leg was detailed); reuse the verdict for both
          // fallback selection and the within-tolerance close decision.
          OsmTrack finalTrack = null;
          String reject = null;
          if (needDetail) {
            finalTrack = mergeSegmentsDetoured(segments, returnTrack);
            reject = qualityGateReason(finalTrack, desiredDistance);
            boolean gateAccepted = reject == null;
            // Prefer a gate-accepted fallback over a gate-rejected one even at a
            // higher geometric error; among same-status candidates keep the
            // lowest error. Selecting by error alone could latch a rejected
            // low-error loop and discard a usable accepted higher-error one.
            if (bestFallback == null
                || isBetterFallback(gateAccepted, error, bestFallback.gateAccepted, bestFallback.error)) {
              bestFallback = snapshotFallback(finalTrack, segments, returnTrack, waypointStack, error, gateAccepted);
            }
          }

          // Within tolerance → close the loop
          if (error <= tolerance) {
            if (reject != null) {
              if (DIAGNOSTIC) {
                System.err.printf("[greedy-diag] closed loop rejected at step %d: %s%n",
                  step, reject);
                if (RoundTripQualityGate.isPavedProfile(profileName)) {
                  System.err.printf("[greedy-diag] closed hostile stretch step=%d %s%n",
                    step, RoundTripQualityGate.worstHostileStretchPaved(finalTrack).describe());
                }
              }
              result.addDiagnostic("closed loop rejected at step " + step
                + ": " + reject + ", retrying");
              segments.remove(segments.size() - 1);
              totalDistance -= accepted.routeDistance;
              if (accepted.fromIsoCandidate) acceptedIsoLegs--;
              else acceptedRadialLegs--;
              removeVisitedEdges(accepted.track, visitedEdges);
              waypointStack.remove(waypointStack.size() - 1);
              currentMwp = waypointStack.get(waypointStack.size() - 1);
              currentIlon = currentMwp.crosspoint.getILon();
              currentIlat = currentMwp.crosspoint.getILat();
              prevIlon = savedPrevIlon;
              prevIlat = savedPrevIlat;
              localRadius = Math.max(MIN_LOCAL_RADIUS_M, localRadius * BACKOFF_FACTOR_NO_CANDIDATE);
              continue;
            }

            addVisitedEdges(returnTrack, visitedEdges, totalDistance);
            segments.add(returnTrack);
            totalDistance += returnTrack.distance; // keep consistent with segments
            if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
              System.err.printf("[greedy-diag] FINAL TRACK accepted-path step=%d totalDist=%d returnDist=%d finalWorstHost=%d%n",
                step, (int) totalDistance, returnTrack.distance,
                RoundTripQualityGate.worstContiguousHostileMetersPaved(finalTrack));
            }
            populateResult(result, finalTrack, waypointStack, start, startMwp, segments, desiredDistance, startDirection);
            result.setTotalDistanceMeters((int) closedDistance);
            result.setWithinTolerance(true);
            result.setSubRoutesChosen(step);
            result.setAttemptsUsed(totalAttempts);
            result.addDiagnostic("loop closed at step " + step
              + ", total=" + (int) closedDistance + "m"
              + ", error=" + String.format("%.1f%%", error * 100));
            stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed, routedIso, routedRadial, acceptedIsoLegs, acceptedRadialLegs);
            return result;
          }

          // Too long → undo sub-route, aggressively shrink radius, retry.
          if (closedDistance > desiredDistance * (1 + tolerance)) {
            result.addDiagnostic("step " + step + ": projected " + (int) closedDistance
              + "m exceeds desired " + (int) desiredDistance + "m, shrinking radius");
            segments.remove(segments.size() - 1);
            totalDistance -= accepted.routeDistance;
            if (accepted.fromIsoCandidate) acceptedIsoLegs--;
            else acceptedRadialLegs--;
            removeVisitedEdges(accepted.track, visitedEdges);
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
      if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
        RoundTripQualityGate.HostileStretch fallbackHostile =
          RoundTripQualityGate.worstHostileStretchPaved(bestFallback.track);
        System.err.printf("[greedy-diag] FALLBACK PATH totalDist=%d finalWorstHost=%d err=%.3f stretch=%s%n",
          bestFallback.track.distance,
          fallbackHostile.meters,
          bestFallback.error, fallbackHostile.describe());
      }
      populateResult(result, bestFallback.track, bestFallback.waypointStack, start,
        startMwp, bestFallback.legTracks, desiredDistance, startDirection);
      result.setTotalDistanceMeters(bestFallback.track.distance);
      result.setWithinTolerance(false);
      String reject = qualityGateReason(bestFallback.track, desiredDistance);
      String reason = "best error=" + String.format("%.1f%%", bestFallback.error * 100);
      result.setFallbackReason(reject == null ? reason : DEGRADED_FALLBACK_PREFIX + reject + "; " + reason);
      result.setSubRoutesChosen(bestFallback.legTracks.size());
      result.setAttemptsUsed(totalAttempts);
      stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed, routedIso, routedRadial, acceptedIsoLegs, acceptedRadialLegs);
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
        returnTrack = engine.retrackForDetail(returnTrack, currentMwp, startMwp, null);
        segments.add(returnTrack);
        OsmTrack finalTrack = mergeSegmentsDetoured(segments, null);
        if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
          RoundTripQualityGate.HostileStretch forceHostile =
            RoundTripQualityGate.worstHostileStretchPaved(finalTrack);
          System.err.printf("[greedy-diag] FORCE-CLOSE finalDist=%d finalWorstHost=%d returnDist=%d stretch=%s%n",
            finalTrack.distance,
            forceHostile.meters,
            returnTrack.distance, forceHostile.describe());
        }
        populateResult(result, finalTrack, waypointStack, start, startMwp, segments, desiredDistance, startDirection);
        result.setTotalDistanceMeters(finalTrack.distance);
        result.setWithinTolerance(false);
        String reject = qualityGateReason(finalTrack, desiredDistance);
        result.setFallbackReason(reject == null ? "forced closure" : DEGRADED_FALLBACK_PREFIX + reject + "; forced closure");
        result.setSubRoutesChosen(segments.size());
        result.setAttemptsUsed(totalAttempts);
        stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed, routedIso, routedRadial, acceptedIsoLegs, acceptedRadialLegs);
        return result;
      }
    }

    result.setFallbackReason("could not build any loop");
    stampTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed, routedIso, routedRadial, acceptedIsoLegs, acceptedRadialLegs);
    return result;
  }

  /**
   * Returns {@code null} if {@code track} meets quality bars (distance ratio,
   * reuse, closure); otherwise returns a short human-readable reason. Used to
   * tag fallback / forced-closure results so the caller can choose to demote
   * them rather than ship as success.
   */
  /**
   * Delegate the planner's internal fallback-quality check to the
   * production gate ({@link RoundTripQualityGate#evaluate}). Pre-Phase 1.5
   * this used independent {@code FALLBACK_MIN_RATIO} / {@code FALLBACK_MAX_RATIO}
   * / {@code FALLBACK_MAX_REUSE} / {@code FALLBACK_MAX_CLOSURE_M}
   * thresholds, which let the planner ship fallback loops the production
   * gate would then reject — wasting the retry budget on outcomes that
   * couldn't survive. Delegating closes that loop: the planner now
   * rejects (and retries) using the same criteria the engine will apply
   * downstream.
   *
   * <p>{@code allowSamewayback} is hard-coded to false: greedy never
   * produces same-way-back routes (it requires a normal loop closure),
   * so the gate's same-way-back permissive path is not relevant here.
   */
  // Package-private for direct testing — see GreedyRoundTripPlannerTest's
  // Phase 1.5 delegation verification.
  String qualityGateReason(OsmTrack track, double desiredDistance) {
    if (track == null || track.nodes == null || track.nodes.size() < 4) return "no track";
    RoundTripQualityResult r = RoundTripQualityGate.evaluate(
      track, desiredDistance, profileName, /*allowSamewayback*/ false);
    return r.isAccepted() ? null : r.getRejectionReason();
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

  /**
   * Stamp planner telemetry on the result including start-iso/non-start-iso
   * source breakdown. The 5-arg overload was replaced by this — callers that
   * don't track source type pass 0 for those four counters. Internally
   * uses {@link #stampBaseTelemetry} for the underlying counters.
   */
  private static void stampTelemetry(RoundTripResult result, long planStart,
                                     int candidatesGenerated, int candidatesRouted,
                                     int returnChecksPerformed,
                                     int routedIso, int routedRadial,
                                     int acceptedIsoLegs, int acceptedRadialLegs) {
    // Delegate base counters to the 5-arg overload (not the 9-arg one — that
    // would recurse forever). Sed-rename caught this site too; the explicit
    // 5-arg overload name avoids the trap.
    stampBaseTelemetry(result, planStart, candidatesGenerated, candidatesRouted, returnChecksPerformed);
    result.setRoutedIsoCandidates(routedIso);
    result.setRoutedRadialCandidates(routedRadial);
    result.setAcceptedIsoLegs(acceptedIsoLegs);
    result.setAcceptedRadialLegs(acceptedRadialLegs);
  }

  private static void stampBaseTelemetry(RoundTripResult result, long planStart,
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
      // Also surface the semantic reuse classification — what SHAPE this
      // loop is (STRICT_LOOP / LOLLIPOP / SCENIC_OUT_AND_BACK) and any
      // disclosures (e.g. "contains retraced scenic spur: 4.2km"). The
      // engine's final gate will reject INVALID_RETRACE before the result
      // is returned to the caller, so a classifier verdict here is for
      // diagnostic surfacing only — never a second accept/reject.
      RoundTripQualityResult qr = ReuseClassifier.classify(track, desiredDistance,
        /*allowSamewayback*/ false);
      result.addDiagnostic("shape: " + qr.getShape()
        + ", stem=" + qr.getTerminalStemReuseMeters() + "m"
        + ", spur=" + qr.getScenicSpurReuseMeters() + "m"
        + ", maxContiguous=" + qr.getMaxContiguousReuseMeters() + "m");
      for (String d : qr.getDisclosures()) result.addDiagnostic("disclosure: " + d);
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

  private OsmTrack detailAcceptedTrack(ScoredRoute accepted, MatchedWaypoint fromMwp,
                                       OsmTrack refTrack, long deadline) {
    OsmTrack detailed = engine.retrackForDetail(accepted.track, fromMwp, accepted.toMwp, refTrack);
    if (!metadataMissingTooHigh(detailed)) {
      return detailed;
    }

    OsmTrack rerouted = timedFindTrack("greedy-sub-detail-fallback", fromMwp, accepted.toMwp, refTrack, deadline);
    if (rerouted == null || rerouted.distance == 0) {
      return detailed;
    }
    OsmTrack detailedRerouted = engine.retrackForDetail(rerouted, fromMwp, accepted.toMwp, refTrack);
    if (DIAGNOSTIC && RoundTripQualityGate.isPavedProfile(profileName)) {
      System.err.printf("[greedy-diag] detail fallback missingMetadata raw=%.1f%% rerouted=%.1f%%%n",
        RoundTripQualityGate.missingMetadataFraction(detailed) * 100.0,
        RoundTripQualityGate.missingMetadataFraction(detailedRerouted) * 100.0);
    }
    return detailedRerouted;
  }

  private boolean metadataMissingTooHigh(OsmTrack track) {
    return RoundTripQualityGate.isPavedProfile(profileName)
      && RoundTripQualityGate.missingMetadataFraction(track) > RoundTripQualityGate.MAX_HOSTILE_FRACTION;
  }

  private static String formatPct(double fraction) {
    return String.format("%.1f%%", fraction * 100.0);
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

  /**
   * Count self-intersections of the committed prefix + one candidate leg.
   *
   * <p>SAFE-4: {@code committedPrefixNodes} is the node list of the merged
   * committed segments, built ONCE per attempt by the caller and shared
   * read-only across all routed candidates of that attempt — replacing the
   * former per-candidate re-merge of the whole prefix. We copy it into a fresh
   * list and append only this candidate's nodes (replicating
   * {@link #appendTrack}'s first-node dedupe), so the resulting node sequence
   * is element-identical to {@code mergeSegmentsNoMap(segments, candidate)} and
   * the crossing count is bit-identical. The shared prefix list is never
   * mutated.
   *
   * <p>SAFE-1: the tentative track is consumed only by
   * {@link RoundTripQualityGate#countSelfIntersections}, which reads
   * {@code track.nodes} exclusively (sampled shape nodes + integer ccw
   * geometry) and never touches {@code nodesMap}/{@code containsNode}, so no
   * map build is needed.
   */
  private int countTentativeSelfIntersections(List<OsmPathElement> committedPrefixNodes,
                                              OsmTrack candidateSegment) {
    if (candidateSegment == null || candidateSegment.nodes == null
        || candidateSegment.nodes.size() < 2) {
      return 0;
    }
    if (committedPrefixNodes == null || committedPrefixNodes.isEmpty()) {
      return RoundTripQualityGate.countSelfIntersections(candidateSegment);
    }
    OsmTrack tentative = new OsmTrack();
    tentative.nodes = new ArrayList<>(
      committedPrefixNodes.size() + candidateSegment.nodes.size());
    tentative.nodes.addAll(committedPrefixNodes);
    appendNodesDeduped(tentative.nodes, candidateSegment.nodes);
    return RoundTripQualityGate.countSelfIntersections(tentative);
  }

  /**
   * Append {@code source} nodes onto {@code targetNodes}, skipping the first
   * source node when it duplicates the current tail — the exact node-dedupe
   * {@link #appendTrack} performs (distance/ascend/cost are irrelevant here
   * because the only consumer reads the node sequence).
   */
  // Package-private for unit testing the dedupe contract (SAFE-4 parity).
  static void appendNodesDeduped(List<OsmPathElement> targetNodes,
                                 List<OsmPathElement> source) {
    boolean first = true;
    for (OsmPathElement node : source) {
      if (first && !targetNodes.isEmpty()) {
        OsmPathElement last = targetNodes.get(targetNodes.size() - 1);
        if (last.getILon() == node.getILon() && last.getILat() == node.getILat()) {
          first = false;
          continue;
        }
      }
      first = false;
      targetNodes.add(node);
    }
  }

  /**
   * Concatenate {@code segments} (then optional {@code finalSegment}) into one
   * track WITHOUT building the node lookup map. The map is only needed by
   * callers that do {@code containsNode}/{@code nodesMap} lookups on the
   * merged track (refTrack poisoning, final/snapshot output); callers that
   * only read the node sequence should use this and skip the map build.
   */
  private OsmTrack mergeSegmentsNoMap(List<OsmTrack> segments, OsmTrack finalSegment) {
    OsmTrack merged = new OsmTrack();
    for (OsmTrack seg : segments) {
      appendTrack(merged, seg);
    }
    if (finalSegment != null) {
      appendTrack(merged, finalSegment);
    }
    return merged;
  }

  private OsmTrack mergeSegments(List<OsmTrack> segments, OsmTrack finalSegment) {
    OsmTrack merged = mergeSegmentsNoMap(segments, finalSegment);
    merged.buildMap();
    return merged;
  }

  /**
   * Like {@link #mergeSegments} but also carries each detailed leg's detour data
   * onto the merged loop, so the result track has the {@code detourMap}
   * {@link OsmTrack#processVoiceHints} needs to emit turn instructions. The
   * greedy legs are already retracked for detail (frozen detourMap), so this is
   * a metadata-only merge: node geometry is identical to {@link #mergeSegments},
   * so a track validated by the quality gate stays valid. Used only for the
   * final result track, not the per-step refTrack merges (which don't need
   * detours).
   */
  private OsmTrack mergeSegmentsDetoured(List<OsmTrack> segments, OsmTrack finalSegment) {
    OsmTrack merged = new OsmTrack();
    for (OsmTrack seg : segments) {
      appendTrack(merged, seg);
      merged.mergeDetoursFrom(seg);
    }
    if (finalSegment != null) {
      appendTrack(merged, finalSegment);
      merged.mergeDetoursFrom(finalSegment);
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

  private void addVisitedEdges(OsmTrack track, VisitedEdgeStore edges,
                               double trackStartCumDist) {
    if (track.nodes == null || track.nodes.size() < 2) return;
    double cumDist = trackStartCumDist;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = a.calcDistance(b);
      long key = edgeKey(a, b);
      if (edges.count(key) == 0) {
        // First visit ever — record the segment midpoint as the first-visit
        // cumulative distance, used downstream for boundary-proximity weighting.
        edges.setFirstPos(key, cumDist + segLen / 2);
      }
      edges.increment(key);
      cumDist += segLen;
    }
  }

  private void removeVisitedEdges(OsmTrack track, VisitedEdgeStore edges) {
    if (track.nodes == null || track.nodes.size() < 2) return;
    for (int i = 1; i < track.nodes.size(); i++) {
      long key = edgeKey(track.nodes.get(i - 1), track.nodes.get(i));
      int count = edges.count(key);
      if (count == 0) continue;
      if (count <= 1) {
        edges.remove(key);
      } else {
        edges.decrement(key);
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
  /**
   * @param segLens SAFE-5 precomputed per-segment distances ({@code segLens[i-1]}
   *                = distance from node i-1 to i), or {@code null} to compute
   *                inline. When non-null it must equal {@code calcDistance} for
   *                every segment — it is the same int widened to double.
   */
  private double computeTrackVisitedRatio(OsmTrack track, VisitedEdgeStore edges,
                                          double trackStartCumDist, double desiredDistance,
                                          int[] segLens) {
    if (edges.isEmpty() || track.nodes == null || track.nodes.size() < 2) return 0.0;
    double total = 0;
    double weightedReuse = 0;
    double cumDist = trackStartCumDist;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = (segLens != null) ? segLens[i - 1] : a.calcDistance(b);
      double midPos = cumDist + segLen / 2;
      total += segLen;
      long key = edgeKey(a, b);
      // A present key always has its firstPos recorded (setFirstPos precedes
      // increment on first visit), so this reproduces the former
      // containsKey-count + non-null-firstPos path exactly; firstPos may be
      // 0.0 (1m first edge) and is still "present" via the occupancy flag.
      if (edges.containsKey(key)) {
        double posWeight = boundaryProximityWeight(edges.firstPos(key), midPos, desiredDistance);
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
   *
   * <p>Distinguishing scenic stems from accidental backtracks is the job of
   * the final {@link ReuseClassifier} gate — the per-edge heuristic here is
   * a planner steering hint, not a semantic classifier. An earlier attempt to
   * push that semantic distinction down to the per-edge level (forgiving
   * stems, penalising mid-loop) caused a real regression: in constrained
   * road networks like Dreieich, raising the mid-loop penalty pushed the
   * planner off the only viable paved loops and onto path/track terrain
   * that the profile gate then rejected outright. Keeping the per-edge
   * weights neutral (boundary=visible, mid=tolerable) lets the planner find
   * the route, and the post-routing classifier decides whether it's a
   * lollipop or accidental retrace.
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

  /**
   * SAFE-5: per-segment integer distances of {@code track}, indexed so
   * {@code result[i-1] == nodes[i-1].calcDistance(nodes[i])}. Shared by
   * {@link #computeTrackVisitedRatio} and
   * {@link RoundTripQualityGate#worstContiguousCostlyMetersForScorer} on the
   * same track so the {@link CheapRuler} distance is computed once, not twice.
   */
  private static int[] segmentDistances(OsmTrack track) {
    if (track == null || track.nodes == null || track.nodes.size() < 2) {
      return new int[0];
    }
    int[] lens = new int[track.nodes.size() - 1];
    for (int i = 1; i < track.nodes.size(); i++) {
      lens[i - 1] = track.nodes.get(i - 1).calcDistance(track.nodes.get(i));
    }
    return lens;
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
  /**
   * Fallback-selection rule: a candidate closed loop replaces the incumbent
   * best fallback when it is gate-accepted and the incumbent is not (regardless
   * of error), or — when both share the same gate verdict — when its geometric
   * error is lower. This prevents latching a gate-rejected low-error loop and
   * discarding a usable gate-accepted higher-error one.
   */
  static boolean isBetterFallback(boolean candidateAccepted, double candidateError,
                                  boolean incumbentAccepted, double incumbentError) {
    if (candidateAccepted != incumbentAccepted) {
      return candidateAccepted;
    }
    return candidateError < incumbentError;
  }

  private Snapshot snapshotFallback(OsmTrack track, List<OsmTrack> segments, OsmTrack returnTrack,
                                    List<MatchedWaypoint> waypointStack, double error, boolean gateAccepted) {
    Snapshot snap = new Snapshot();
    snap.track = track;
    snap.waypointStack = new ArrayList<>(waypointStack);
    snap.legTracks = new ArrayList<>(segments);
    snap.legTracks.add(returnTrack);
    snap.error = error;
    snap.gateAccepted = gateAccepted;
    return snap;
  }

  private static final class Snapshot {
    OsmTrack track;
    List<MatchedWaypoint> waypointStack;
    List<OsmTrack> legTracks;
    double error;
    boolean gateAccepted;
  }

  /**
   * A candidate that has been routed. Package-private so unit tests can
   * construct instances and verify candidate-list ordering (Phase 1 Step 2
   * of the closure-aware control-node planning spec).
   */
  static final class ScoredRoute {
    OsmTrack track;
    MatchedWaypoint toMwp;
    double routeDistance;
    double visitedRatio;
    /** True iff this leg was selected from an iso-derived candidate. */
    boolean fromIsoCandidate;
    /**
     * Final routed score after combinedRoutedScore() and the partial
     * self-intersection penalty (lower is better). Used to sort the
     * per-step candidate list and as the input to the Step 5 closure score.
     */
    double routedScore;
    /** Index of this candidate in the per-step trial loop (0-based). */
    int candidateIndex;
    /** Tentative self-intersections of the routed leg against committed segments. */
    int tentativeSelfIntersections;
    /**
     * Longest contiguous hostile stretch in the routed leg, in meters,
     * computed via {@link RoundTripQualityGate#worstContiguousHostileMetersPaved}.
     * Sentinel {@code -1} on non-paved profiles where the predicate would
     * over-flag.
     */
    int routedLegWorstHostileMeters;
  }

  /**
   * Sort routed candidates ascending by {@link ScoredRoute#routedScore}
   * (lower is better). Stable: candidates with equal score retain their
   * insertion order, which preserves the legacy first-best-wins tie-break.
   *
   * <p>Package-private for unit testing (Phase 1 Step 2 acceptance criterion:
   * "routed candidates are sorted by routedScore; partial self-intersection
   * penalty affects ordering").
   */
  static void sortByRoutedScore(List<ScoredRoute> candidates) {
    candidates.sort(BY_ROUTED_SCORE);
  }
}
