package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.OsmNode;
import btools.router.OsmNodeNamed;
import btools.util.CheapAngleMeter;
import btools.util.CheapRuler;

/**
 * Waypoint-based tier (WAYPOINT/ISOCHRONE): place a via skeleton
 * geometrically (FAST probe/envelope, isochrone frontier, or circle
 * fallback), then run one engine routing pass over it. Outcome lands on the
 * request and continues to the orchestrator's shared floors and gate.
 */
final class FastStrategy implements RoundTripStrategy {

  private final RoundTripOrchestrator orchestrator;
  private final RoundTripEngineOps ops;

  FastStrategy(RoundTripOrchestrator orchestrator) {
    this.orchestrator = orchestrator;
    this.ops = orchestrator.ops;
  }

  @Override
  public void attempt(RoundTripRequest request, TierSlice slice) {
    WaypointSnapper snapper = orchestrator.snapper;
    GeometricWaypointPlacer placer = orchestrator.placer;
    RoundTripAlgorithm algo = slice.algo;
    double searchRadius = slice.searchRadius;
    double direction = slice.direction;
    FastPlacementOutcome.Path placedPath = null;
    int targetPoints = 0;

    // Variety seed: bounded multi-knob perturbation for the geometric
    // placement paths (see RoutingContext.getRoundTripSeed). The phase shift stays within ±15° so the
    // direction focus is preserved; the radius stays within ±3% so the loop
    // stays inside the quality gate's distance tolerance — the gate's
    // expectedDistance is computed from the caller's UNJITTERED radius, so
    // it keeps measuring against the user's requested length. targetPoints
    // ±1 is applied below (derived values only). Seed 0/absent leaves every
    // knob at exactly 0 — bit-identical to the unseeded baseline.
    int varietySeed = ops.routingContext().getRoundTripSeed();
    if (varietySeed > 0) {
      double phaseShiftDeg = 15.0 * GreedyRoundTripPlanner.seededUnit(varietySeed, 1, 0);
      double radiusScale = 1.0 + 0.03 * GreedyRoundTripPlanner.seededUnit(varietySeed, 2, 0);
      direction = CheapAngleMeter.normalize(direction + phaseShiftDeg);
      searchRadius *= radiusScale;
      ops.logInfo("round trip variety seed " + varietySeed + ": phase shift " + (int) phaseShiftDeg
        + " deg, radius scale " + radiusScale);
    }
    if (ops.routingContext().allowSamewayback) {
      int[] pos = CheapRuler.destination(ops.waypoints().get(0).ilon, ops.waypoints().get(0).ilat, searchRadius, direction);
      OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
      onn.name = "rt1";
      ops.waypoints().add(onn);
      // No-beeline invariant: snap the tip before final matchWaypointsToNodes.
      // On snap failure the tip stays at the raw geometric point and the return
      // leg can degrade to a straight-line beeline — surface it rather than
      // silently discarding the result (cf. snapWaypointsToRoad for user vias).
      if (!snapper.snapWaypointToRoad(onn, Math.min(searchRadius * 0.3, 2000), "snapSamewaybackTip")) {
        ops.logInfo("snapSamewaybackTip: no road within snap range; samewayback return leg may include a beeline");
      }
    } else {
      // INVARIANT: this branch runs only in non-explicit-via mode, which is
      // reached only when ops.waypoints().size() == 1 (user vias are handled earlier by
      // doExplicitViaRoundTrip). Fail fast if a future refactor ever routes user
      // vias here, rather than silently re-running the old bearing-sorted via
      // injection that doExplicitViaRoundTrip was built to replace.
      if (ops.waypoints().size() > 1) {
        throw new IllegalStateException(
          "the waypoint tier expects a single start waypoint; user vias must be "
            + "handled by doExplicitViaRoundTrip (got " + ops.waypoints().size() + ")");
      }

      targetPoints = ops.routingContext().roundTripPoints == null ?
        Math.max(5, Math.min(15, (int) (searchRadius / 1500) + 3)) :
        ops.routingContext().roundTripPoints;
      // Variety seed knob: ±1 via-point count, only when the count is derived —
      // an explicit roundTripPoints is a user decision the seed must not override.
      if (varietySeed > 0 && ops.routingContext().roundTripPoints == null) {
        targetPoints = Math.max(4,
          targetPoints + (int) Math.round(GreedyRoundTripPlanner.seededUnit(varietySeed, 3, 0)));
      }

      // FAST perf optimization (ideas 1/2/4): when on, the FAST tier reuses the
      // probe's already-snapped road nodes as vias and skips the redundant
      // validateAndAdjustWaypoints re-matching pass (and its cache reset).
      // Toggle off with -Droundtrip.fast.optimized=false to compare against the
      // legacy probe+envelope+validate path. AUTO/QUALITY/ISOCHRONE unaffected.
      // (Not Boolean.getBoolean: that reads absent as false, but this flag
      // must default to ON.)
      boolean fastOptimized = Boolean.parseBoolean(
        System.getProperty("roundtrip.fast.optimized", "true"));

      if (algo == RoundTripAlgorithm.ISOCHRONE) {
        ProbeResult probe = snapper.probeReachableDirections(ops.waypoints().get(0), searchRadius);
        double[] probeDirections = (probe != null) ? probe.viableDirections : null;
        IsochroneExpansionResult iso = ops.runIsochroneExpansion(ops.waypoints().get(0), searchRadius);
        double[][] frontier = (iso != null) ? iso.frontier : null;
        double[][] merged = GeometricWaypointPlacer.mergeIsochroneWithProbe(frontier, probeDirections, searchRadius);
        if (merged != null && merged.length >= 3) {
          List<IsoCandidate> isoCandidates = (iso != null) ? iso.candidates : null;
          orchestrator.recordPlacementPath(RoundTripOrchestrator.PlacementPath.ISOCHRONE);
          placer.placeWaypointsFromIsochrone(ops.waypoints(), merged, isoCandidates, searchRadius, direction, targetPoints);
        } else if (probeDirections != null && probeDirections.length >= 3) {
          ops.logInfo("isochrone merge insufficient, falling back to probe directions");
          orchestrator.recordPlacementPath(RoundTripOrchestrator.PlacementPath.ENVELOPE_ISO_FALLBACK);
          placer.placeWaypointsFromEnvelope(ops.waypoints(), probeDirections, searchRadius, direction, targetPoints);
        } else {
          ops.logInfo("both isochrone and probe insufficient, falling back to circle");
          orchestrator.recordPlacementPath(RoundTripOrchestrator.PlacementPath.CIRCLE);
          ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
        }
      } else if (fastOptimized) {
        // Directional lobe, always: the loop heads toward the resolved bearing
        // (caller-supplied startDirection/heading, or the random draw) like the
        // pre-1.7.9 upstream circle placement — it never encircles the start
        // as the primary shape. The encircling ring exists only as fallback:
        // placement-time degeneration falls back inside the planner
        // (RING_FALLBACK / circle), and the post-routing retry at the end of
        // attempt() covers sparse terrain where the lobe places but cannot
        // route.
        FastPlacementOutcome fastOutcome = placeOptimized(searchRadius, direction,
          targetPoints, true);
        placedPath = fastOutcome.path;
      } else {
        placeLegacy(snapper, placer, searchRadius, direction, targetPoints);
      }

      // Idea 4: the optimized FAST module fully validates its own skeleton —
      // probe-snapped vias are pre-validated, and its circle fallback runs this
      // pass behind FastPlacementOps.circleFallbackValidated. The legacy
      // placement validates inside placeLegacy; only ISOCHRONE needs the
      // caller-side matching pass here.
      if (algo == RoundTripAlgorithm.ISOCHRONE) {
        snapper.validateAndAdjustWaypoints(ops.waypoints(), searchRadius);
      }

      // Snap start/end ops.waypoints() to nearest road to prevent beeline segments.
      // Without this, if the user's click position is >250m from a road (park,
      // water, etc.), the routing engine inserts straight-line beelines.
      snapper.snapStartToRoad(ops.waypoints(), searchRadius);
    }

    ops.routingContext().waypointCatchingRange = 250;
    request.setSearchRadius(searchRadius);
    // The placement above rebuilt the waypoint list; a doRouting run earlier in
    // this request (bounded-tier fallback, the ring retry below) leaves the
    // engine's matched waypoints populated, and tryFindTrack would reuse them
    // instead of matching the fresh skeleton — routing and cleaning against
    // stale vias.
    ops.setMatchedWaypoints(null);
    orchestrator.doRoutingIntoRequest(request.routingBudgetMs);

    // Post-routing ring retry: the directional lobe heads the loop toward the
    // requested bearing, but in sparse terrain routing between forward-arc
    // vias can fail where an encircling ring would not. Placement-time
    // failures already fall back inside the planner; this catches a lobe that
    // PLACED but did not ROUTE into a real loop.
    if (placedPath == FastPlacementOutcome.Path.DIRECTIONAL_LOBE && degenerateOutcome(request)) {
      ops.logInfo("FAST: directional lobe did not route to a loop"
        + (request.error == null ? "" : " (" + request.error + ")")
        + "; retrying with the encircling ring");
      orchestrator.setError(null);
      OsmNodeNamed start = ops.waypoints().get(0);
      ops.waypoints().clear();
      ops.waypoints().add(start);
      placeOptimized(searchRadius, direction, targetPoints, false);
      ops.setMatchedWaypoints(null);
      orchestrator.doRoutingIntoRequest(request.routingBudgetMs);
    }

    // Shape retry: the optimized placement is faster but can produce tangled
    // loops where the legacy probe+envelope placement stays clean (measured —
    // Limoux 180 km east: 5 self-crossings optimized vs 1 legacy; Basel
    // 180 km: hostile-stretch warning vs a gate-accepted loop). Probe this
    // attempt's own gate verdict; on a rejection, redo the placement the
    // legacy way and keep the better outcome. WAYPOINT tier only — ISOCHRONE
    // and forced-legacy runs are already on the legacy pass.
    if (algo == RoundTripAlgorithm.WAYPOINT
        && !ops.routingContext().allowSamewayback
        && Boolean.parseBoolean(System.getProperty("roundtrip.fast.optimized", "true"))
        && Boolean.parseBoolean(System.getProperty("roundtrip.fast.shape.retry", "true"))
        && !degenerateOutcome(request)) {
      RoundTripQualityResult optVerdict =
        orchestrator.evaluateRoundTripGate(request.track, searchRadius, false, false);
      // Retry on any of FAST's three measured failure shapes, not only on a
      // gate rejection (Freiburg corpus, 20 cells vs upstream's old routine):
      // tolerated self-crossings (34 vs 11 with the rejection-only trigger),
      // and near-zero-area "threads" — an out-and-back over PARALLEL streets
      // closes cleanly, retraces nothing and crosses little, so every gate
      // check passes while the enclosed area collapses (11 of 20 cells at
      // compactness 0.00-0.09 vs upstream's 0.15-0.45). The keep-better rule
      // below ships the legacy loop only when it is strictly better, so a
      // clean optimized loop never loses.
      double optCompact = trackCompactness(request);
      if (!optVerdict.isAccepted() || optVerdict.getSelfIntersections() > 0
          || optCompact < MIN_FAST_COMPACTNESS) {
        retryCirclePlacement(request, snapper, searchRadius, direction,
          targetPoints, optVerdict, optCompact);
      }
    }
  }

  /**
   * Loop-vs-thread floor for the shape retry: isoperimetric compactness below
   * this means the routed "loop" encloses almost no area. Corpus calibration
   * (Freiburg fastbike, 20 cells): degenerate threads price 0.00-0.09, the old
   * routine's real loops 0.09-0.45 — 0.10 catches the thread class while a
   * borderline real loop merely earns a (harmless) second attempt.
   */
  static final double MIN_FAST_COMPACTNESS = 0.10;

  private double trackCompactness(RoundTripRequest request) {
    // NET enclosed area, not the hull metric: only this discriminates a real
    // loop from a thread (see enclosedAreaCompactness).
    return LoopQualityMetrics.enclosedAreaCompactness(request.track.nodes, request.track.distance);
  }

  /**
   * The legacy probe+envelope FAST placement (the pre-optimization path):
   * reachability probe, confidence filter, envelope placement (circle
   * fallback), then the full re-validation pass. Selectable for a whole run
   * via {@code -Droundtrip.fast.optimized=false}; the shape retry uses it
   * when the optimized loop fails the gate.
   */
  private void placeLegacy(WaypointSnapper snapper, GeometricWaypointPlacer placer,
                           double searchRadius, double direction, int targetPoints) {
    ProbeResult probe = snapper.probeReachableDirections(ops.waypoints().get(0), searchRadius);
    // FAST tier: drop single-probe-success directions when enough strong
    // alternatives exist. Avoids fragile sea-edge/dead-end picks.
    double[] viableDirections = PlacementGeometry.filterByProbeConfidence(probe, targetPoints);
    if (viableDirections != null && viableDirections.length >= 3) {
      orchestrator.recordPlacementPath(RoundTripOrchestrator.PlacementPath.ENVELOPE_FAST);
      placer.placeWaypointsFromEnvelope(ops.waypoints(), viableDirections, searchRadius, direction, targetPoints);
    } else {
      ops.logInfo("reachability probe returned < 3 directions, falling back to circle");
      orchestrator.recordPlacementPath(RoundTripOrchestrator.PlacementPath.CIRCLE);
      ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
    }
    snapper.validateAndAdjustWaypoints(ops.waypoints(), searchRadius);
  }

  /**
   * Redo placement with the legacy pass and keep the better of the two
   * outcomes: the legacy loop ships when its verdict is accepted or it has
   * fewer self-crossings than the rejected optimized loop; otherwise the
   * optimized outcome (track + matched waypoints + skeleton) is restored.
   */
  private void retryCirclePlacement(RoundTripRequest request, WaypointSnapper snapper,
                                    double searchRadius, double direction, int targetPoints,
                                    RoundTripQualityResult optVerdict, double optCompact) {
    String why;
    if (!optVerdict.isAccepted()) {
      why = "rejected by the gate (" + (optVerdict.getRejectionReason() == null
        ? String.valueOf(optVerdict.getShape()) : optVerdict.getRejectionReason()) + ")";
    } else if (optVerdict.getSelfIntersections() > 0) {
      why = "has " + optVerdict.getSelfIntersections() + " self-crossing(s)";
    } else {
      why = String.format(java.util.Locale.ROOT,
        "is a near-zero-area thread (compactness %.2f)", optCompact);
    }
    ops.logInfo("FAST: optimized placement " + why
      + "; retrying with the classic circle placement");
    btools.router.OsmTrack optTrack = request.track;
    List<btools.mapaccess.MatchedWaypoint> optMatched = ops.matchedWaypoints();
    List<OsmNodeNamed> optSkeleton = new java.util.ArrayList<>(ops.waypoints());

    orchestrator.setError(null);
    // Route fresh: doRoutingIntoRequest seeds the engine with request.track,
    // which the round-trip anti-reuse penalty treats as a reference — leaving
    // the optimized loop in place would steer the retry AWAY from its roads
    // instead of re-deciding freely (measured: 198 km warned vs the 147 km
    // accepted loop the same placement finds on a clean engine).
    orchestrator.setTrack(null);
    OsmNodeNamed start = ops.waypoints().get(0);
    ops.waypoints().clear();
    ops.waypoints().add(start);
    // The upstream-equivalent placement class: raw geometric circle vias
    // toward the requested direction, routed as-is — NO relocation pass.
    // This is what holds direction and length in terrain where the
    // probe-filtered placements collapse (Freiburg corpus: upstream's routine
    // averages 99% of the ask where the optimized lobe threads or stars), and
    // the relocation pass is exactly what collapsed the 50km-east circle back
    // into the valley thread it was meant to escape. Degenerate outcomes
    // (beelines, islands — the old routine's failure modes) lose in the
    // keep-better cascade below, so the rawness is safe here.
    ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
    snapper.snapStartToRoad(ops.waypoints(), searchRadius);
    ops.setMatchedWaypoints(null);
    orchestrator.doRoutingIntoRequest(request.routingBudgetMs);

    boolean circleUsable = request.error == null && !degenerateOutcome(request);
    RoundTripQualityResult circleVerdict = circleUsable
      ? orchestrator.evaluateRoundTripGate(request.track, searchRadius, false, false)
      : null;
    // Shape-first cascade (the FAST tier's contract: loop FORM first —
    // length is the user's input knob): 1) a real loop beats a thread even
    // when the thread is gate-accepted (the gate cannot see threads);
    // 2) a STRUCTURALLY rejected loop (broken closure/beeline class) never
    // ships over a structurally sound one; 3) fewer self-crossings beats
    // MORE, even against gate acceptance — a QUALITY-tier warning is a
    // disclosed imperfection the lenient gate ships anyway, while a tangle
    // is exactly what the user sees (measured: an accepted 4-crossing lobe
    // must lose to a clean warned circle); 4) then acceptance; 5) then
    // clearly higher compactness. Length never decides. Ties keep the
    // optimized loop.
    boolean keepCircle = false;
    if (circleUsable) {
      double cirCompact = trackCompactness(request);
      int optCrossings = optVerdict.getSelfIntersections() < 0
        ? Integer.MAX_VALUE : optVerdict.getSelfIntersections();
      int cirCrossings = circleVerdict.getSelfIntersections() < 0
        ? Integer.MAX_VALUE : circleVerdict.getSelfIntersections();
      boolean optIsLoop = optCompact >= MIN_FAST_COMPACTNESS;
      boolean cirIsLoop = cirCompact >= MIN_FAST_COMPACTNESS;
      boolean optStructural = !optVerdict.isAccepted()
        && optVerdict.getRejectionTier() == RoundTripQualityResult.RejectionTier.STRUCTURAL;
      boolean cirStructural = !circleVerdict.isAccepted()
        && circleVerdict.getRejectionTier() == RoundTripQualityResult.RejectionTier.STRUCTURAL;
      if (cirIsLoop != optIsLoop) {
        keepCircle = cirIsLoop;
      } else if (cirStructural != optStructural) {
        keepCircle = optStructural;
      } else if (cirCrossings != optCrossings) {
        keepCircle = cirCrossings < optCrossings;
      } else if (circleVerdict.isAccepted() != optVerdict.isAccepted()) {
        keepCircle = circleVerdict.isAccepted();
      } else {
        keepCircle = cirCompact > optCompact + 0.02;
      }
    }
    if (circleUsable) {
      ops.logInfo(String.format(java.util.Locale.ROOT,
        "FAST: circle attempt %.1fkm accepted=%s crossings=%d compactness=%.2f",
        request.track.distance / 1000.0, circleVerdict.isAccepted(),
        circleVerdict.getSelfIntersections(),
        trackCompactness(request)));
    } else {
      ops.logInfo("FAST: circle attempt unusable ("
        + (request.error != null ? request.error : "degenerate track") + ")");
    }
    if (keepCircle) {
      ops.logInfo("FAST: shipping the circle-placement loop ("
        + (circleVerdict.isAccepted() ? "gate-accepted" : "better shape or length") + ")");
    } else {
      ops.logInfo("FAST: circle retry not better; keeping the optimized loop");
      orchestrator.setError(null);
      orchestrator.setTrack(optTrack);
      ops.setMatchedWaypoints(optMatched);
      ops.waypoints().clear();
      ops.waypoints().addAll(optSkeleton);
    }
  }

  /**
   * Run the optimized FAST placement and commit its skeleton in one step — a
   * degraded or failed attempt can never leave partial vias in the live
   * waypoint list.
   */
  private FastPlacementOutcome placeOptimized(double searchRadius, double direction,
                                              int targetPoints, boolean directional) {
    FastPlacementRequest fastRequest = new FastPlacementRequest(
      ops.waypoints().get(0), searchRadius, direction, targetPoints, directional);
    FastPlacementOutcome outcome = new FastWaypointPlanner(ops.fastPlacementOps()).place(fastRequest);
    ops.waypoints().clear();
    ops.waypoints().addAll(outcome.skeleton);
    orchestrator.recordPlacementPath(outcome.optimizedPlacement()
      ? RoundTripOrchestrator.PlacementPath.ENVELOPE_FAST : RoundTripOrchestrator.PlacementPath.CIRCLE);
    return outcome;
  }

  /** No routed loop, or a degenerate stub below the shared floors. */
  static boolean degenerateOutcome(RoundTripRequest request) {
    return request.track == null || request.track.nodes == null
      || request.track.nodes.size() < RoundTripOrchestrator.MIN_ROUNDTRIP_LOOP_NODES
      || request.track.distance < RoundTripOrchestrator.MIN_ROUNDTRIP_LOOP_METERS;
  }
}
