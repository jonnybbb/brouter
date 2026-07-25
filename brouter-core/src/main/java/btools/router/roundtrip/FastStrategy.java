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
    // Radius the PLACEMENT uses. Starts at the requested radius and is the
    // only thing the explicit-length correction below moves; every gate
    // evaluation keeps using searchRadius so the verdict is always measured
    // against what the USER asked for, never against a corrected geometry.
    double placementRadius = searchRadius;
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
      placementRadius = searchRadius;
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

      // 5 points (4 generated vias) at EVERY radius — the upstream old-routine
      // contract. An earlier radius-scaled formula (up to 15 points at large
      // radii) scored better on the crossing/spur metrics but forced the route
      // through a 14-stop skeleton that pleats like a folding fan at 180 km —
      // a structural failure none of those metrics can see. Few vias leave the
      // router freedom to find one natural loop; many vias overconstrain it.
      targetPoints = ops.routingContext().roundTripPoints == null ? 5 :
        ops.routingContext().roundTripPoints;
      // Variety seed knob: ±1 via-point count, only when the count is derived —
      // an explicit roundTripPoints is a user decision the seed must not override.
      if (varietySeed > 0 && ops.routingContext().roundTripPoints == null) {
        // Clamped to [4, 5] points (3-4 vias): the FAST contract is at most 4
        // generated vias — the seed may only reduce, never add a fifth.
        targetPoints = Math.max(4, Math.min(5,
          targetPoints + (int) Math.round(GreedyRoundTripPlanner.seededUnit(varietySeed, 3, 0))));
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
        FastPlacementOutcome fastOutcome = placeOptimized(placementRadius, direction,
          targetPoints, true);
        placedPath = fastOutcome.path;
      } else {
        placeLegacy(snapper, placer, placementRadius, direction, targetPoints);
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
      placeOptimized(placementRadius, direction, targetPoints, false);
      ops.setMatchedWaypoints(null);
      orchestrator.doRoutingIntoRequest(request.routingBudgetMs);
    }

    // Length contract, two strengths. roundTripLength is the strong promise —
    // "desired total loop length" — and the correction fires on any miss beyond
    // +-10% (measured before it existed: median 0.87x of the ask, worst 0.54x
    // over 24 explicit-length cells). roundTripDistance is a RADIUS ("loop
    // length is roughly 2*pi*radius") and was long left deliberately short:
    // the old routine reached full length partly by force-routing through
    // unreachable points, the stacked-waypoint class this tier removed. But
    // the guarded corrective pass is not that mechanism — it re-places at a
    // scaled radius and ships only a strictly-better loop — and the three-way
    // against stock upstream showed the undershoot is FAST's dominant visible
    // flaw (six of twelve cells at 0.65-0.83x; nominal length deviation WORSE
    // than the ungated stock fan's). So radius requests now get the same
    // single corrective pass, asymmetrically: only when the loop comes in
    // clearly SHORT (below RADIUS_UNDERSHOOT_TRIGGER of 2*pi*R) — overshoot
    // stays inside the documented "roughly" and shrinking a placement risks
    // shape for little gain. Placement radius and delivered length are
    // near-linear, so ONE pass at radius x (asked/delivered) closes most of
    // the miss. The correction moves the PLACEMENT radius only — the gate
    // keeps measuring against the user's own ask — and the loop ships only if
    // it lands closer to the ask without losing shape, so a failed or tangled
    // retry costs a budget-gated routing pass and nothing else. The shape
    // retry below re-evaluates the CORRECTED track, so the passes compose
    // without extra control flow.
    if (algo == RoundTripAlgorithm.WAYPOINT
        && !ops.routingContext().allowSamewayback
        && !degenerateOutcome(request)) {
      placementRadius = correctForLengthMiss(request, snapper, searchRadius,
        placementRadius, direction, targetPoints);
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
      // Retry on any of FAST's four measured failure shapes, not only on a
      // gate rejection (Freiburg corpus, 20 cells vs upstream's old routine):
      // tolerated self-crossings (34 vs 11 with the rejection-only trigger),
      // near-zero-area "threads" — an out-and-back over PARALLEL streets
      // closes cleanly, retraces nothing and crosses little, so every gate
      // check passes while the enclosed area collapses (11 of 20 cells at
      // compactness 0.00-0.09 vs upstream's 0.15-0.45) — plus the clover
      // (petal) and start-pinched bowtie (dwell) classes, each documented at
      // its threshold constant. The keep-better rule below ships the legacy
      // loop only when it is strictly better, so a clean optimized loop
      // never loses.
      //
      // Shape facts come from the track itself, not the verdict: a rejection
      // made at the gate's distance-band check returns BEFORE the crossing
      // scan (getSelfIntersections() is -1 there), and FAST's contract is
      // loop shape — length is the user's input knob — so a distance-band
      // rejection alone must not spend a second routing pass.
      double optCompact = trackCompactness(request);
      int optCrossings = trackCrossings(request, optVerdict);
      double optPetal = trackPetal(request);
      double optDwell = trackDwell(request);
      boolean shapeReject = !optVerdict.isAccepted() && !optVerdict.isDistanceBandRejection();
      if (shapeReject || optCrossings > 0 || optCompact < MIN_FAST_COMPACTNESS
          || optPetal >= MAX_FAST_PETAL || optDwell < MIN_FAST_DWELL) {
        retryCirclePlacement(request, snapper, searchRadius, direction,
          targetPoints, optVerdict, optCompact, optCrossings, optPetal, optDwell);
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

  /**
   * Clover ceiling for the shape retry: {@link LoopQualityMetrics#petalAmplitude}
   * above this means the loop is several lobes pinched at a shared centre —
   * several out-and-backs pretending to be a loop.
   *
   * <p>This is FAST's third measured failure shape, and the only one the other
   * two triggers structurally CANNOT see: a clover's petals meet at a shared
   * junction rather than crossing (so the crossing scan correctly reports zero)
   * and it encloses real area (so it clears the thread floor). Corpus
   * calibration (548-cell matrix): clover loops score 0.30-0.56, corpus median
   * 0.16, and the loops between 0.25 and 0.30 are ordinary ovals — 0.30 is the
   * shoulder. A false trigger costs one routing pass and nothing else: the
   * keep-better cascade ships the retry only when it is strictly better.
   */
  static final double MAX_FAST_PETAL = 0.30;

  /**
   * Petal amplitudes closer than this are the same shape as far as a cyclist is
   * concerned; the cascade must not flip a decision on harmonic noise.
   */
  private static final double PETAL_TIE_MARGIN = 0.10;

  /**
   * Far-field floor for the shape retry: {@link LoopQualityMetrics#farFieldDwell}
   * below this means the ride's arc stays pinched near the start — the two-lobe
   * bowtie hinged AT the start point. It is the one failure shape the other
   * rungs structurally cannot see: the lobes meet without crossing, they
   * enclose real area (clears the thread floor), and two lobes are below the
   * petal metric's 3..12 harmonic band. Corpus calibration (548-cell matrix):
   * median 0.62, an ideal circle through the start prices 0.59 analytically,
   * and the eye-confirmed bowties sit far below (Finale Ligure 0.25, Annecy
   * 0.27, Freiburg-east 0.28). A false trigger costs one budget-gated routing
   * pass and nothing else — the keep-better cascade decides what ships.
   */
  static final double MIN_FAST_DWELL = 0.35;

  /** Dwell gaps smaller than this are shape noise; the cascade must not flip on them. */
  private static final double DWELL_TIE_MARGIN = 0.10;

  private double trackPetal(RoundTripRequest request) {
    return LoopQualityMetrics.petalAmplitude(request.track.nodes, request.track.distance);
  }

  private double trackDwell(RoundTripRequest request) {
    return LoopQualityMetrics.farFieldDwell(request.track.nodes);
  }

  private double trackCompactness(RoundTripRequest request) {
    // NET enclosed area, not the hull metric: only this discriminates a real
    // loop from a thread (see enclosedAreaCompactness).
    return LoopQualityMetrics.enclosedAreaCompactness(request.track.nodes, request.track.distance);
  }

  /**
   * Crossing count for the routed track: the gate's own scan when the verdict
   * carries one, otherwise a direct scan — a verdict rejected before the
   * gate's crossing scan (distance band) reports -1, which must never be
   * mistaken for "maximally crossed" in the keep-better cascade.
   */
  private int trackCrossings(RoundTripRequest request, RoundTripQualityResult verdict) {
    int fromGate = verdict.getSelfIntersections();
    return fromGate >= 0 ? fromGate : LoopAnalysis.of(request.track).selfIntersections;
  }

  /** Explicit-length band: inside this the ask is already honoured. */
  private static final double LENGTH_TOLERANCE = 0.10;

  /**
   * Radius-request trigger: a loop below this fraction of the implied
   * 2*pi*radius length earns the corrective pass. Wider than the explicit
   * band because the radius contract is "roughly", and one-sided because the
   * systematic miss is short (matrix median 0.84x; the three-way's six
   * losing-length cells sat at 0.65-0.83x while 0.92-1.13x cells read fine).
   */
  private static final double RADIUS_UNDERSHOOT_TRIGGER = 0.85;

  /** Bounds on one corrective step, so a freak first pass cannot send the
   *  placement somewhere the road network cannot serve. */
  private static final double MIN_LENGTH_CORRECTION = 0.6;
  private static final double MAX_LENGTH_CORRECTION = 1.8;

  /** A corrected loop may not overshoot the ask by more than this (see below). */
  private static final double MAX_OVERSHOOT = 1.10;

  /** How much of the first pass's compactness a corrected loop must retain. */
  private static final double SHAPE_RETENTION = 0.7;

  /** A loop this compact is good on its own terms, whatever the first pass scored. */
  private static final double GOOD_COMPACTNESS = 3 * MIN_FAST_COMPACTNESS;

  /**
   * Radius-implied corrections must pay for themselves: the corrected loop
   * ships only when it lands at least this much closer to the ask. Without a
   * materiality bar a 0.66x -> 0.68x "improvement" reshuffles the entire loop
   * for a length gain no rider can feel (measured: Freiburg 30 km mtb west
   * traded 3pp retrace and a rougher shape for exactly that). The
   * explicit-length path keeps strict improvement — that contract is the
   * user's own number, and every step toward it counts.
   */
  private static final double MIN_RADIUS_CORRECTION_GAIN = 0.05;

  /**
   * The materiality bar above measures LENGTH gain only — but the corrective
   * re-placement sometimes fixes the loop's SHAPE in passing, and a length
   * bar must not veto that. Eye-checked case (Dreieich 30 km north): the
   * shipped loop pinches into a waist at the start (compactness 0.34), the
   * rejected candidate is a clean full polygon (0.57) — thrown away because
   * its length gain was 0.04 instead of 0.05. So the bar is waived — not the
   * other guards — when compactness improves by at least this much; the
   * weighted miss must still improve strictly.
   */
  private static final double SHAPE_GAIN_WAIVER = 0.15;

  /**
   * Radius-ask length tolerance. The explicit path's MAX_OVERSHOOT cliff is
   * right for the user's own number, but the "roughly 2*pi*R" contract is a
   * soft promise, and length strictness must not veto good loops (product
   * call, 2026-07-24: "FAST should be more tolerant for length"). Two rules:
   *
   * 1) TOLERANT BAND [RADIUS_TOLERANT_UNDERSHOOT, RADIUS_MAX_OVERSHOOT]:
   *    when BOTH the first and the corrected loop land inside it, length is
   *    considered satisfied and stops deciding — a visible shape upgrade
   *    (compactness + IN_BAND_SHAPE_MARGIN, every other guard still holding)
   *    ships the correction. Eye-calibrated on Crete Senesi 75 km south: the
   *    88 km full-step loop (1.17x, compactness +0.06, fuller ring coverage)
   *    is plainly the better ride than the 61.5 km incumbent, yet every
   *    miss-based rule rejects it because 1.17x and 0.82x are near-equal
   *    misses; +0.03 gaps read as visual siblings and stay rejected.
   * 2) Outside the band the weighted-miss improvement test applies:
   *    overshoot rides free to +10% and costs OVERSHOOT_MISS_WEIGHT x
   *    beyond (running long outlasts daylight, water and legs — but only
   *    clearly beyond the tolerance, not from the first metre over).
   *    Basel 1.41x and Grenoble 1.35x full-step corrections stay rejected;
   *    their half-steps (1.13x / 1.15x) ship.
   */
  static final double RADIUS_MAX_OVERSHOOT = 1.25;
  static final double RADIUS_TOLERANT_UNDERSHOOT = 0.75;

  /**
   * Whether a delivered/asked length ratio is one the rider would accept as the
   * size they requested — the same band the radius correction treats as "length
   * satisfied either way". One band for the whole tier.
   */
  static boolean inTolerantLengthBand(double ratio) {
    return ratio >= RADIUS_TOLERANT_UNDERSHOOT && ratio <= RADIUS_MAX_OVERSHOOT;
  }

  /**
   * The shape-retry cascade's length rung: {@code +1} keep the circle, {@code -1}
   * keep the optimized loop, {@code 0} no opinion.
   *
   * <p>The cascade is shape-first by design and every rung above this one stays
   * that way — a thread, a structural rejection, a crossing, a clover and a
   * bowtie all still outrank length. This rung breaks only the tie that used to
   * fall through to raw compactness, and only when the two candidates disagree
   * about whether they are a plausible answer AT ALL: exactly one of them inside
   * {@link #inTolerantLengthBand}.
   *
   * <p>Motivating measurement — Garmisch 96 km east (fastbike): the optimized
   * lobe delivered 0.67x of the ask carrying a 12.7 km profile-hostile stretch,
   * the circle rung delivered 0.91x with zero crossings and dwell 0.62, both
   * gate-rejected, and the cascade kept the 0.67x loop because its compactness
   * was 0.50 against 0.37. Two loops that are both acceptable lengths must still
   * be decided on shape; a loop a third short of the ask is not an answer.
   *
   * <p>Costs nothing: both tracks are already routed and measured when this runs.
   */
  static int lengthBandPreference(double optRatio, double cirRatio) {
    boolean optIn = inTolerantLengthBand(optRatio);
    boolean cirIn = inTolerantLengthBand(cirRatio);
    if (optIn == cirIn) return 0;
    return cirIn ? 1 : -1;
  }
  private static final double IN_BAND_SHAPE_MARGIN = 0.05;
  private static final double OVERSHOOT_MISS_WEIGHT = 2.0;

  /** Overshoot cap for the weighted-miss acceptance path (see the acceptance
   *  block for why it stays tighter than the band edge). */
  private static final double RADIUS_WEIGHTED_PATH_CAP = 1.15;

  /**
   * In-band shape upgrades may spend at most this much weighted length-miss:
   * "length is satisfied inside the band" must not mean length is free
   * (measured: Freiburg 30 km mtb south traded a near-perfect 0.87x up to
   * the 1.24x band edge for a marginal compactness gain — a +43% ride the
   * rider never asked for; Crete Senesi's eye-confirmed upgrade costs only
   * 6 pp and passes).
   */
  private static final double IN_BAND_MISS_SLACK = 0.10;

  /**
   * Miss from the ask for the radius-ask improvement test. Undershoot is
   * priced linearly. Overshoot rides free up to the explicit path's own
   * tolerance (MAX_OVERSHOOT − 1 = 10% — "roughly 2*pi*R" plainly covers
   * that) and costs OVERSHOOT_MISS_WEIGHT x beyond it. A flat 2:1 price
   * from zero made a shape-held 1.13x lose to the 0.73x it was fixing
   * (Basel 50 km north) — the rider model in the criteria catalog prices
   * overshoot double only BEYOND the tolerance band, not from the first
   * metre over.
   */
  private static double weightedMiss(double ratio) {
    if (ratio >= 1.0) {
      double freeBand = MAX_OVERSHOOT - 1.0;
      double over = ratio - 1.0;
      return Math.min(over, freeBand)
        + OVERSHOOT_MISS_WEIGHT * Math.max(0.0, over - freeBand);
    }
    return 1.0 - ratio;
  }

  /**
   * One corrective placement pass for a missed length ask — the explicit
   * {@code roundTripLength} (any miss beyond the tolerance band) or the
   * radius-implied 2*pi*R (clear undershoot only). Returns the placement
   * radius to use from here on: the corrected one when the retry landed
   * closer to the ask without losing shape, otherwise the radius passed in
   * (with the original loop restored).
   */
  private double correctForLengthMiss(RoundTripRequest request, WaypointSnapper snapper,
                                      double searchRadius, double placementRadius,
                                      double direction, int targetPoints) {
    Integer explicitAsk = ops.routingContext().roundTripLength;
    double asked = explicitAsk != null ? explicitAsk : 2 * Math.PI * searchRadius;
    double got = request.track.distance;
    if (asked <= 0 || got <= 0) return placementRadius;
    double ratio = got / asked;
    boolean miss = explicitAsk != null
      ? Math.abs(ratio - 1.0) > LENGTH_TOLERANCE
      : ratio < RADIUS_UNDERSHOOT_TRIGGER;
    if (!miss) return placementRadius;

    double correction = Math.max(MIN_LENGTH_CORRECTION,
      Math.min(MAX_LENGTH_CORRECTION, asked / got));
    ops.logInfo(String.format(java.util.Locale.ROOT,
      "FAST: %s %.1fkm asked, %.1fkm placed (%.2fx); re-placing at radius x%.2f",
      explicitAsk != null ? "explicit length" : "radius-implied length",
      asked / 1000.0, got / 1000.0, ratio, correction));

    btools.router.OsmTrack firstTrack = request.track;
    List<btools.mapaccess.MatchedWaypoint> firstMatched = ops.matchedWaypoints();
    List<OsmNodeNamed> firstSkeleton = new java.util.ArrayList<>(ops.waypoints());
    double firstMiss = Math.abs(ratio - 1.0);
    double firstPetal = trackPetal(request);
    double firstCompact = trackCompactness(request);
    int firstCrossings = LoopAnalysis.of(request.track).selfIntersections;

    // At most two attempts: the full step, and — radius asks only, when the
    // full step's loop routed fine but ran PAST the ask — one half step
    // (geometric mean). Loop length grows super-linearly with the placement
    // radius in some terrain (measured elasticities up to ~2x), so the
    // linear step overshoots exactly where the undershoot was worst; the
    // half step is the one bisection move that recovers those cells (Basel
    // 50 km north: x1.37 -> 1.41x rejected, half step x1.17 lands ~1.0x)
    // without opening an unbounded search.
    double factor = correction;
    for (int attemptNo = 1; attemptNo <= 2; attemptNo++) {
      double correctedRadius = placementRadius * factor;
      orchestrator.setError(null);
      // Same reason the shape retry routes fresh: leaving the first loop in
      // place makes it a reference the anti-reuse penalty steers AWAY from,
      // so the second pass would not re-decide freely.
      orchestrator.setTrack(null);
      OsmNodeNamed start = ops.waypoints().get(0);
      ops.waypoints().clear();
      ops.waypoints().add(start);
      placeOptimized(correctedRadius, direction, targetPoints, true);
      snapper.snapStartToRoad(ops.waypoints(), correctedRadius);
      ops.setMatchedWaypoints(null);
      orchestrator.doRoutingIntoRequest(request.routingBudgetMs);

      boolean usable = request.error == null && !degenerateOutcome(request);
      double secondRatio = usable ? request.track.distance / asked : 0.0;
      double secondMiss = usable ? Math.abs(secondRatio - 1.0) : Double.MAX_VALUE;
      // Length is why this pass ran, but it must not buy length with shape:
      // a corrected loop that turns into a thread or a clover is not an
      // improvement a rider would accept. Measured: an uncorrected Annecy
      // 100 km gravel loop at compactness 0.37 came back corrected at 0.17 —
      // longer and visibly worse — so the floor is relative to what the first
      // pass achieved, not just the absolute thread floor.
      double secondCompact = usable ? trackCompactness(request) : 0.0;
      boolean shapeHeld = usable
        && secondCompact >= MIN_FAST_COMPACTNESS
        // Either it keeps most of what the first pass had, OR it is a solidly
        // round loop in its own right. The second clause matters: a first pass
        // can be very compact precisely BECAUSE it is far too short (a 27 km
        // circle for a 50 km ask), and a relative-only floor would then reject
        // the correction that finally delivers the asked length at a perfectly
        // good shape. GOOD_COMPACTNESS is expressed in the tier's own calibrated
        // unit — 3x the thread floor — not a second independent constant.
        && (secondCompact >= firstCompact * SHAPE_RETENTION
            || secondCompact >= GOOD_COMPACTNESS)
        && (trackPetal(request) < MAX_FAST_PETAL || trackPetal(request) <= firstPetal)
        // A correction must never buy length with a tangle: an added
        // self-crossing would only re-trigger the shape retry, which then
        // swaps in a circle loop the first pass never needed (measured:
        // Dreieich 100 km south — the corrected 106 km loop carried one
        // crossing and cascaded into a worse same-length circle; Berlin
        // 100 km south likewise).
        && LoopAnalysis.of(request.track).selfIntersections <= firstCrossings;
      // Asymmetric on purpose: length error is not symmetric for a rider. A loop
      // that comes in short can be extended on the day; one that runs long can
      // outlast the daylight, the water or the legs. So a correction ships only
      // when it does NOT overshoot the ask by more than MAX_OVERSHOOT — measured,
      // the undamped step overshot 4 of 24 cells (worst 1.32x) because loop
      // length grows super-linearly with the placement radius in some terrain.
      // Radius asks use the wider cap and the 2:1 weighted miss (see
      // RADIUS_MAX_OVERSHOOT).
      // Each acceptance path carries its own overshoot cap. The weighted-miss
      // path keeps the tighter 1.15 cap: with a terrible first pass (0.55x)
      // the weighted metric happily judges a 1.24x loop "closer", and that
      // acceptance PREEMPTS the shape retry's circle rescue which would land
      // near the ask (measured: Freiburg 30 km mtb south — base pipeline
      // rescued to 0.87x, the capless correction shipped 1.24x instead).
      // Only the band shape-upgrade path — which demands both loops already
      // in tolerable length and a visible shape gain at bounded miss slack —
      // may use the full band edge.
      boolean weightedPathSafe = usable && secondRatio
        <= (explicitAsk != null ? MAX_OVERSHOOT : RADIUS_WEIGHTED_PATH_CAP);
      double requiredGain = explicitAsk != null ? 0.0
        : (secondCompact >= firstCompact + SHAPE_GAIN_WAIVER
           ? 0.0 : MIN_RADIUS_CORRECTION_GAIN);
      boolean closerToAsk = explicitAsk != null
        ? secondMiss < firstMiss
        : weightedMiss(secondRatio) < weightedMiss(ratio) - requiredGain;
      // Tolerant-band shape upgrade (radius asks): both loops inside
      // [0.75, 1.25] of the ask means length is satisfied either way —
      // a visibly better shape decides, spending at most IN_BAND_MISS_SLACK
      // of weighted miss (see RADIUS_MAX_OVERSHOOT).
      boolean shapeUpgradeInBand = explicitAsk == null && usable
        && ratio >= RADIUS_TOLERANT_UNDERSHOOT && ratio <= RADIUS_MAX_OVERSHOOT
        && secondRatio >= RADIUS_TOLERANT_UNDERSHOOT && secondRatio <= RADIUS_MAX_OVERSHOOT
        && secondCompact > firstCompact + IN_BAND_SHAPE_MARGIN
        && weightedMiss(secondRatio) <= weightedMiss(ratio) + IN_BAND_MISS_SLACK;
      if (usable && shapeHeld
          && ((closerToAsk && weightedPathSafe) || shapeUpgradeInBand)) {
        ops.logInfo(String.format(java.util.Locale.ROOT,
          "FAST: corrected loop %.1fkm (%.2fx of ask); keeping it",
          request.track.distance / 1000.0, request.track.distance / asked));
        return correctedRadius;
      }
      ops.logInfo("FAST: length correction not better ("
        + (usable ? String.format(java.util.Locale.ROOT, "%.2fx of ask, shape held=%s",
            request.track.distance / asked, shapeHeld)
          : "unusable: " + (request.error != null ? request.error : "degenerate"))
        + "); keeping the first loop");
      orchestrator.setError(null);
      orchestrator.setTrack(firstTrack);
      ops.setMatchedWaypoints(firstMatched);
      ops.waypoints().clear();
      ops.waypoints().addAll(firstSkeleton);
      boolean overshot = usable && secondRatio > 1.0;
      if (attemptNo == 1 && explicitAsk == null && correction > 1.0 && overshot) {
        factor = Math.sqrt(correction);
        ops.logInfo(String.format(java.util.Locale.ROOT,
          "FAST: correction overshot; one half-step retry at radius x%.2f", factor));
        continue;
      }
      break;
    }
    return placementRadius;
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
                                    double searchRadius,
                                    double direction, int targetPoints,
                                    RoundTripQualityResult optVerdict, double optCompact,
                                    int optCrossings, double optPetal, double optDwell) {
    String why;
    if (!optVerdict.isAccepted() && !optVerdict.isDistanceBandRejection()) {
      why = "rejected by the gate (" + (optVerdict.getRejectionReason() == null
        ? String.valueOf(optVerdict.getShape()) : optVerdict.getRejectionReason()) + ")";
    } else if (optCrossings > 0) {
      why = "has " + optCrossings + " self-crossing(s)";
    } else if (optPetal >= MAX_FAST_PETAL) {
      why = String.format(java.util.Locale.ROOT,
        "is a clover of pinched lobes (petal %.2f)", optPetal);
    } else if (optCompact < MIN_FAST_COMPACTNESS) {
      why = String.format(java.util.Locale.ROOT,
        "is a near-zero-area thread (compactness %.2f)", optCompact);
    } else {
      why = String.format(java.util.Locale.ROOT,
        "is a bowtie pinched at the start (far-field dwell %.2f)", optDwell);
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
    //
    // Placed at searchRadius, NOT placementRadius: the length-correction pass
    // scales the placement radius against the LOBE placement's measured length
    // response, while the circle fan reaches ~2*pi*R by construction — the
    // user's own radius already maps to the ask under this placement's
    // response. Feeding it the lobe-corrected radius double-corrects
    // (measured: Girona 75 km mtb west — correction x1.66 kept a 62.5 km
    // loop, then the corridor-warned shape retry's circle at the corrected
    // radius shipped a gate-accepted 118.7 km loop, 1.58x of the ask).
    ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
    snapper.snapStartToRoad(ops.waypoints(), searchRadius);
    ops.setMatchedWaypoints(null);
    orchestrator.doRoutingIntoRequest(request.routingBudgetMs);

    Integer explicitAsk = ops.routingContext().roundTripLength;
    double asked = explicitAsk != null ? explicitAsk : 2 * Math.PI * searchRadius;
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
    // must lose to a clean warned circle); 4) then acceptance, where a
    // distance-band rejection counts as accepted — a soft length verdict must
    // not outrank shape; 5) then the length rung, which speaks ONLY when the
    // two candidates disagree about being a plausible size at all (see
    // lengthBandPreference); 6) then clearly higher compactness.
    // Ties keep the optimized loop. Crossing counts come from
    // trackCrossings, never from a verdict's -1 pre-scan placeholder.
    boolean keepCircle = false;
    if (circleUsable) {
      double cirCompact = trackCompactness(request);
      int cirCrossings = trackCrossings(request, circleVerdict);
      double cirPetal = trackPetal(request);
      double cirDwell = trackDwell(request);
      int lengthPreference = asked > 0
        ? lengthBandPreference(optTrack.distance / asked, request.track.distance / asked) : 0;
      boolean optIsLoop = optCompact >= MIN_FAST_COMPACTNESS;
      boolean cirIsLoop = cirCompact >= MIN_FAST_COMPACTNESS;
      boolean optStructural = !optVerdict.isAccepted()
        && optVerdict.getRejectionTier() == RoundTripQualityResult.RejectionTier.STRUCTURAL;
      boolean cirStructural = !circleVerdict.isAccepted()
        && circleVerdict.getRejectionTier() == RoundTripQualityResult.RejectionTier.STRUCTURAL;
      boolean optAcc = optVerdict.isAccepted() || optVerdict.isDistanceBandRejection();
      boolean cirAcc = circleVerdict.isAccepted() || circleVerdict.isDistanceBandRejection();
      if (cirIsLoop != optIsLoop) {
        keepCircle = cirIsLoop;
      } else if (cirStructural != optStructural) {
        keepCircle = optStructural;
      } else if (cirCrossings != optCrossings) {
        keepCircle = cirCrossings < optCrossings;
      } else if (Math.abs(cirPetal - optPetal) > PETAL_TIE_MARGIN
                 && (cirPetal >= MAX_FAST_PETAL || optPetal >= MAX_FAST_PETAL)) {
        // Clover discrimination, ranked with the other shape defects and above
        // acceptance for the same reason crossings are: a gate warning is a
        // disclosed imperfection the lenient gate ships anyway, while a clover
        // is what the rider sees. Only decides when at least one side actually
        // IS a clover and the gap is bigger than harmonic noise.
        keepCircle = cirPetal < optPetal;
      } else if (Math.abs(cirDwell - optDwell) > DWELL_TIE_MARGIN
                 && (cirDwell < MIN_FAST_DWELL || optDwell < MIN_FAST_DWELL)) {
        // Bowtie discrimination, same contract as the clover rung: decides
        // only when at least one side actually IS pinched at the start and
        // the gap is bigger than shape noise. Higher dwell = more of the
        // ride spent genuinely away from home.
        keepCircle = cirDwell > optDwell;
      } else if (cirAcc != optAcc) {
        keepCircle = cirAcc;
      } else if (lengthPreference != 0) {
        // Length rung — the ONLY place length speaks in this cascade, and only
        // when the two candidates disagree about being a plausible size at all.
        keepCircle = lengthPreference > 0;
      } else {
        keepCircle = cirCompact > optCompact + 0.02;
      }
    }
    if (circleUsable) {
      ops.logInfo(String.format(java.util.Locale.ROOT,
        "FAST: circle attempt %.1fkm accepted=%s crossings=%d compactness=%.2f petal=%.2f dwell=%.2f",
        request.track.distance / 1000.0, circleVerdict.isAccepted(),
        trackCrossings(request, circleVerdict),
        trackCompactness(request), trackPetal(request), trackDwell(request)));
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
