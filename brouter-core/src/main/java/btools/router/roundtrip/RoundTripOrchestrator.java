package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import btools.mapaccess.OsmNode;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;
import btools.util.CheapAngleMeter;
import btools.util.CheapRuler;

/**
 * Round-trip request orchestrator: the FAST &lt; BALANCED &lt; AUTO &lt; QUALITY
 * tier ladder, waypoint-based and explicit-via generation, greedy/bounded
 * planner dispatch, and the AUTO candidate competition over child engines.
 * Reaches the engine only through {@link RoundTripEngineOps}; child engines are
 * driven through their public API and their own ops seam.
 */
public final class RoundTripOrchestrator {

  private final RoundTripEngineOps ops;

  /** The active request's mutable state; recreated at each doRoundTrip entry. */
  private RoundTripRequest request = new RoundTripRequest();

  /** One resolved rung of the tier ladder: which strategy runs, with which slice. */
  private static final class Rung {
    final RoundTripStrategy strategy;
    final TierSlice slice;

    Rung(RoundTripStrategy strategy, TierSlice slice) {
      this.strategy = strategy;
      this.slice = slice;
    }
  }

  /** Waypoint-based tier: geometric/FAST placement, then one routing run. */
  private final RoundTripStrategy fastStrategy = (request, slice) -> {
    doWaypointBasedRoundTrip(slice.searchRadius, slice.direction, slice.algo);
    return true;
  };

  /** Greedy plan-and-route tier (GREEDY / ISO_GREEDY). */
  private final RoundTripStrategy greedyStrategy = (request, slice) -> {
    doGreedyRoundTrip(slice.searchRadius, slice.direction, slice.algo);
    return true;
  };

  /** Bounded tier: budget-sliced greedy attempt with a waypoint fallback. */
  private final RoundTripStrategy boundedStrategy = (request, slice) -> {
    doBoundedRoundTrip(slice.searchRadius, slice.direction, slice.effortPolicy,
      slice.label, slice.greedyCapable);
    return true;
  };

  /**
   * AUTO candidate competition over child engines; self-finalizing (children
   * are gated inside the competition, the winner is decorated on adoption).
   * QUALITY is this strategy pinned to the MAX preset — configuration, not a
   * separate implementation.
   */
  private final RoundTripStrategy autoCompetitionStrategy = (request, slice) -> {
    request.effortPolicy = slice.effortPolicy;
    runAutoCandidateCompetition(slice.searchRadius, slice.direction);
    return false;
  };

  /**
   * Resolve the tier ladder for this request context. QUALITY pins the
   * competition to the MAX preset; AUTO resolves an effort preset from context
   * and runs the competition — or, on constrained resources, the bounded tier;
   * explicit BALANCED runs bounded; GREEDY/ISO_GREEDY run the planner when the
   * request supports it; everything else (and every samewayback downgrade)
   * runs the waypoint tier. Returns the rungs to attempt in order — currently
   * always exactly one; multi-rung fallback is the extension point.
   */
  private List<Rung> resolveLadder(RoundTripAlgorithm algo, double searchRadius, double direction) {
    // Request context for the effort policy: profile class from the profile's
    // own validFor* globals (name-independent), coarse length class, and
    // resources. Logged once so future policy rules land on recorded evidence.
    RoundTripEffortPolicy.ProfileClass profileClass = classifyProfileClass();
    RoundTripEffortPolicy.LengthClass lengthClass =
      RoundTripEffortPolicy.classifyLength(2 * Math.PI * searchRadius);
    if (profileClass == RoundTripEffortPolicy.ProfileClass.MOTOR) {
      ops.logInfo("round trip: profile class MOTOR — loop quality is unvalidated for"
        + " motorized profiles; using bike-derived policies (provisional)");
    }

    boolean greedyCapable = greedySupports(ops.routingContext().allowSamewayback, ops.waypoints().size());

    // QUALITY: the full competition at max effort — both planners always run,
    // wider routed top-K, doubled plan budget. NOT an ISO_GREEDY alias: greedy
    // wins ~a quarter of competition cells.
    if (algo == RoundTripAlgorithm.QUALITY && greedyCapable) {
      ops.logInfo("round trip effort: " + RoundTripEffortPolicy.MAX_PRESET.rationale);
      return Collections.singletonList(new Rung(autoCompetitionStrategy,
        new TierSlice(algo, RoundTripEffortPolicy.MAX_PRESET, searchRadius, direction, true, "QUALITY")));
    }
    if (algo == RoundTripAlgorithm.QUALITY) {
      // The planners do not honor allowSamewayback. Name the tier in the log —
      // the silent rewrite below (QUALITY -> selectRoundTripAlgorithm ->
      // WAYPOINT) otherwise hides that the MAX effort request was downgraded.
      ops.logInfo("QUALITY round trip does not support allowSamewayback, falling back to waypoint algorithm");
    }

    // AUTO candidate competition, effort resolved from context. Constrained
    // resources (short request budget, memory-constrained device) resolve to
    // the BOUNDED preset — the bounded tier instead of the full competition,
    // with the same fall-through to the shared floors and quality gate (an
    // early return would ship ungated tracks that an identical explicit
    // BALANCED request rejects or returns with a Warning).
    if (algo == RoundTripAlgorithm.AUTO && greedyCapable) {
      RoundTripEffortPolicy resolved = RoundTripEffortPolicy.resolveAuto(
        profileClass, lengthClass, ops.routingContext().memoryclass, ops.maxRunningTime());
      ops.logInfo("round trip effort: " + resolved.rationale);
      if (resolved.preset != RoundTripEffortPolicy.Preset.BOUNDED) {
        return Collections.singletonList(new Rung(autoCompetitionStrategy,
          new TierSlice(algo, resolved, searchRadius, direction, true, "AUTO")));
      }
      return Collections.singletonList(new Rung(boundedStrategy,
        new TierSlice(algo, resolved, searchRadius, direction, true, "AUTO(bounded)")));
    }

    if (algo == RoundTripAlgorithm.AUTO || algo == RoundTripAlgorithm.QUALITY) {
      algo = selectRoundTripAlgorithm(searchRadius);
    }
    ops.logInfo("round trip algorithm: " + algo);

    if (algo == RoundTripAlgorithm.BALANCED) {
      // allowSamewayback is handled inside the bounded tier: the planner slice
      // is skipped, but the waypoint placement keeps the tier budget instead
      // of inheriting the full request budget.
      return Collections.singletonList(new Rung(boundedStrategy,
        new TierSlice(algo, RoundTripEffortPolicy.BOUNDED_PRESET, searchRadius, direction, greedyCapable, "BALANCED")));
    }
    if (algo == RoundTripAlgorithm.GREEDY || algo == RoundTripAlgorithm.ISO_GREEDY) {
      if (!greedyCapable) {
        // Greedy generates its own intermediate points and does not honor
        // allowSamewayback. (User vias are handled in explicit-via mode.)
        ops.logInfo("greedy round trip does not support allowSamewayback, falling back to waypoint algorithm");
        return Collections.singletonList(new Rung(fastStrategy,
          new TierSlice(RoundTripAlgorithm.WAYPOINT, null, searchRadius, direction, false, "WAYPOINT")));
      }
      // ISO_GREEDY: isochrone-derived candidate pool; falls back to plain
      // GREEDY internally if the candidate pool is insufficient.
      return Collections.singletonList(new Rung(greedyStrategy,
        new TierSlice(algo, null, searchRadius, direction, true, algo.toString())));
    }
    return Collections.singletonList(new Rung(fastStrategy,
      new TierSlice(algo, null, searchRadius, direction, greedyCapable, algo.toString())));
  }

  /**
   * Publish the engine-read slice of the request (radius, deadline,
   * explicit-via, guide tracks) to the engine's search loops. Call after every
   * mutation of one of those request fields.
   */
  private void publishRuntimeHints() {
    ops.setRoundTripRuntimeHints(new RoundTripRuntimeHints(
      request.searchRadius, request.requestDeadline, request.explicitVia, request.greedyLegTracks));
  }

  /** Record the gate-rejected track on the active request (post-mortem surface). */
  private void setRejectedTrack(OsmTrack track) {
    request.lastRejectedTrack = track;
  }

  /** Set the request's working result track (published to the engine at request end). */
  private void setTrack(OsmTrack track) {
    request.track = track;
  }

  /** Set the request's working error message (published to the engine at request end). */
  private void setError(String error) {
    request.error = error;
  }

  /**
   * Run the engine routing pipeline and capture its outcome on the request.
   * The engine's result fields are seeded from the request first, so the run
   * sees exactly the state the individual field writes used to leave behind.
   */
  private void doRoutingIntoRequest(long budgetMs) {
    ops.setFoundTrack(request.track);
    ops.setErrorMessage(request.error);
    RoutingOutcome outcome = ops.doRouting(budgetMs);
    request.track = outcome.track;
    request.error = outcome.error;
  }

  /** Record the planner-result telemetry on the active request. */
  private void setPlannerResult(RoundTripResult result) {
    request.lastResult = result;
  }
  private final WaypointSnapper snapper;
  private final GeometricWaypointPlacer placer;
  private final RoundTripTrackCleanup cleanup;

  public RoundTripOrchestrator(RoundTripEngineOps ops) {
    this.ops = ops;
    this.snapper = new WaypointSnapper(ops, ops, ops);
    this.placer = new GeometricWaypointPlacer(ops);
    this.cleanup = new RoundTripTrackCleanup(snapper, ops, ops, ops);
  }

  // A loop must enclose area: at least a triangle (start + 2 intermediate waypoints).
  // A single intermediate point is only an out-and-back, not a loop.
  private static final int MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS = 2;

  // A produced round-trip below either bound is a degenerate stub, not a loop.
  private static final int MIN_ROUNDTRIP_LOOP_NODES = 6;

  private static final int MIN_ROUNDTRIP_LOOP_METERS = 200;

  private int ROUNDTRIP_DEFAULT_DIRECTIONADD = 45;

  // AUTO competition runs its candidates sequentially in the calling thread and
  // cannot interrupt a child mid-run, so it shares one wall-clock budget across
  // all candidates instead of giving each the full timeout. DEFAULT applies when
  // the caller passes no timeout (maxRunningTime <= 0); MIN_CHILD guarantees a
  // spawned candidate still gets a usable slice.
  private static final long DEFAULT_AUTO_BUDGET_MS = 60_000;

  /**
   * Loops up to this length must work on the standard request budget; longer
   * loops require the caller to opt in with a raised timeout (gate in doRoundTrip).
   */
  static final double MAX_STANDARD_LOOP_METERS = 200_000;

  /** Minimum request budget accepted for loops above {@link #MAX_STANDARD_LOOP_METERS}. */
  static final long LONG_LOOP_MIN_BUDGET_MS = 120_000;

  /**
   * Grace (ms) the request thread waits past a parallel AUTO GREEDY child's own
   * budget before terminating it — bounds the join so a wedged or overshooting
   * child can never hang the request thread.
   */
  private static final long AUTO_CHILD_JOIN_UNWIND_MS = 3_000;

  /**
   * When false (default), do not start plain GREEDY speculatively before
   * ISO_GREEDY proves it is needed — avoids duplicate runs on strong or
   * graph-native-absorbed ISO_GREEDY results. Opt back into the old lower-latency
   * tradeoff with {@code -DroundTripSpeculativeAutoGreedy=true}.
   */
  private static final boolean SPECULATIVE_AUTO_GREEDY =
    Boolean.getBoolean("roundTripSpeculativeAutoGreedy");

  /**
   * JVM-wide permit pool capping how many AUTO requests run their speculative
   * GREEDY child in parallel when {@link #SPECULATIVE_AUTO_GREEDY} is on — routing
   * is CPU-bound, so this bounds the extra threads. Tune via
   * {@code -DroundTripParallelAutoPermits}; 0 forces fully-sequential AUTO.
   */
  private static final java.util.concurrent.Semaphore PARALLEL_AUTO_SEMAPHORE =
    new java.util.concurrent.Semaphore(Math.max(0,
      Integer.getInteger("roundTripParallelAutoPermits",
        Runtime.getRuntime().availableProcessors() - 1)));

  private static final java.util.concurrent.atomic.AtomicLongArray PLACEMENT_PATH_COUNTS =
    new java.util.concurrent.atomic.AtomicLongArray(PlacementPath.values().length);

  private static void addUniqueCount(List<Integer> counts, int n) {
    if (n < 3 || n > 6 || counts.contains(n)) return;
    counts.add(n);
  }

  /**
   * Append a space-separated line to {@code track.message} (advisories and gate
   * disclosures for the GPX/JSON formatters). No-op if either argument is null/empty.
   */
  private static void appendRouteMessage(OsmTrack track, String message) {
    if (track == null || message == null || message.isEmpty()) return;
    if (track.message == null || track.message.isEmpty()) {
      track.message = message;
    } else {
      track.message += " " + message;
    }
  }

  /**
   * AUTO's plain-GREEDY entitlement check: a below-threshold ISO_GREEDY does not
   * imply a useful second GREEDY run — if ISO_GREEDY already used graph-native
   * candidates (provider fallback or internal graph-native compare), GREEDY would
   * just duplicate the same source truth.
   */
  static boolean autoNeedsPlainGreedy(RoundTripCandidateResult isoGreedyR,
                                      long now, long deadline) {
    return autoPlainGreedyDiscardReason(isoGreedyR, now, deadline) == null;
  }

  static String autoPlainGreedyDiscardReason(RoundTripCandidateResult isoGreedyR,
                                             long now, long deadline) {
    if (now >= deadline) {
      return "past deadline at decision point";
    }
    if (isoGreedyR == null || !isoGreedyR.accepted()) {
      return null;
    }
    if (isoGreedyR.scoreValue() >= CLEAR_ACCEPT_THRESHOLD) {
      return "ISO_GREEDY strong";
    }
    if (isoGreedyR.internalGraphNativeCompared()) {
      return "ISO_GREEDY already compared graph-native branch";
    }
    if (isoGreedyAbsorbedGraphNativeTruth(isoGreedyR)) {
      return "ISO_GREEDY absorbed graph-native truth";
    }
    return null;
  }

  /** Human-readable axis label for the infeasibility error. */
  private static String axisName(double axisBearingDegrees) {
    // axisBearingDegrees is canonical [0, 180). Snap to the nearest cardinal
    // pair for a readable label.
    double a = ((axisBearingDegrees % 180) + 180) % 180;
    if (a < 22.5 || a >= 157.5) return "N-S";
    if (a < 67.5) return "NE-SW";
    if (a < 112.5) return "E-W";
    return "NW-SE";
  }

  /**
   * Budget (ms) for the next sequential AUTO candidate: time left to the shared
   * competition deadline, floored at {@link #MIN_CHILD_BUDGET_MS} so a spawned
   * candidate gets a usable slice rather than ~0.
   */
  static long childCandidateBudgetMs(long deadline, long now) {
    return Math.max(MIN_CHILD_BUDGET_MS, deadline - now);
  }

  /**
   * Profile family from the profile's own validFor* globals (name-independent):
   * validForBikes, validForFoot, or validForCars. A profile declaring none reads
   * UNKNOWN and keeps standard-effort behavior.
   */
  private RoundTripEffortPolicy.ProfileClass classifyProfileClass() {
    if (ops.routingContext() == null || ops.routingContext().expctxWay == null) {
      return RoundTripEffortPolicy.ProfileClass.UNKNOWN;
    }
    return RoundTripEffortPolicy.classifyProfile(
      ops.routingContext().expctxWay.getVariableValue("validForFoot", 0f) == 1f,
      ops.routingContext().expctxWay.getVariableValue("validForBikes", 0f) == 1f,
      ops.routingContext().expctxWay.getVariableValue("validForCars", 0f) == 1f);
  }

  /**
   * Uniform round-trip gate — the single source of truth for the gate flags,
   * shared by {@code doRoundTrip}'s verdict and the bounded tier's fallback so the
   * two can never drift. Explicit-via mode makes distance advisory (skeleton
   * defines the route) but still enforces beeline/closure/hostility. A forced
   * same-way-back corridor is accepted as a disclosed OUT_AND_BACK (keep-when-forced;
   * the planner sets the flag only when no clean alternative exists).
   */
  private RoundTripQualityResult evaluateRoundTripGate(OsmTrack track, double searchRadius,
                                                       boolean explicitViaMode) {
    return evaluateRoundTripGate(track, searchRadius, explicitViaMode,
      request.forcedCorridorAccepted);
  }

  /**
   * A blended verdict below the clear-accept bar (or null) warrants the internal
   * graph-native comparison. Trigger and selection both read verdicts from
   * {@link #scoreInternalGreedyResult} so they can never judge a track differently
   * — they drifted once (ferries hard-coded off in a separate trigger path) and
   * every ferry-using loop paid a spurious extra ladder.
   */
  static boolean internalBranchNeeded(RouteChoiceScore.Verdict blendedVerdict) {
    return blendedVerdict == null || blendedVerdict.score() < CLEAR_ACCEPT_THRESHOLD;
  }

  private static boolean isDegradedGreedyResult(RoundTripResult result) {
    return result != null
      && result.getFallbackReason() != null
      && result.getFallbackReason().startsWith(GreedyRoundTripPlanner.DEGRADED_FALLBACK_PREFIX);
  }

  /**
   * Pick between the blended result and the internal graph-native branch, on
   * verdicts computed ONCE at the call site (each gate+score pass rebuilds the
   * crossing grid and corridor index — two per comparison, not four).
   */
  private static RoundTripResult selectBetterInternalIsoGreedyResult(
      RoundTripResult blended, RouteChoiceScore.Verdict blendedScore,
      RoundTripResult graphNative, RouteChoiceScore.Verdict graphScore) {
    if (graphScore == null) {
      return blended;
    }
    if (blendedScore == null) {
      return graphNative;
    }
    if (graphScore.score() > blendedScore.score() + 1e-9) {
      return graphNative;
    }
    return blended;
  }

  public void doRoundTrip() {
    request = new RoundTripRequest();
    request.effortPolicy = ops.roundTripEffortPolicy();
    request.routingBudgetMs = ops.roundTripRoutingBudgetMs();
    request.requestDeadline = ops.roundTripRequestDeadline();
    // Track/error seeds: the engine starts with an initial empty track and a
    // null error; early-return paths must publish exactly those back.
    request.track = ops.foundTrack();
    request.error = ops.errorMessage();
    try {
      long wallStart = System.currentTimeMillis();

      ops.routingContext().useDynamicDistance = true;
      // Classify the profile's surface policy once, from its cost model (not its
      // name), so the quality gate and planner hostility checks use a consistent,
      // name-independent verdict for the rest of this request.
      RoundTripQualityGate.classifyPavedProfile(ops.routingContext().expctxWay, ops.routingContext().getProfileName());
      double searchRadius;
      if (ops.routingContext().roundTripLength != null) {
        // roundTripLength is the desired total loop distance — convert to internal search radius.
        // The waypoint strategies place points at searchRadius from start and route between them,
        // so the loop traces roughly the circle circumference: total ≈ 2*PI * searchRadius.
        // Do NOT raise this factor toward L/2 (the out-and-back relation) thinking it gives a
        // "wider" loop: a closed loop traces the circumference, so a larger radius overshoots.
        // Measured across 4 real regions (urban/alpine/coastal/rural) for a 40km target, the
        // distance ratio climbs monotonically with the factor — L/2π≈0.91, 0.20→1.3, 0.25→1.6,
        // 0.33→2.1, L/2→3.2 — so L/2π is the calibrated optimum (closest to 1.0, best composite).
        searchRadius = ops.routingContext().roundTripLength / (2 * Math.PI);
      } else {
        // Defensive floor: a non-positive roundTripDistance (e.g. set directly on
        // the context, bypassing the param-layer guard) would otherwise become a
        // zero/negative searchRadius. That ships a wrong-scale loop with the
        // distance gate silently disabled — the ratio check is skipped when
        // expectedDistance (2*PI*searchRadius) <= 0 — so floor it to the default.
        searchRadius = (ops.routingContext().roundTripDistance == null
          || ops.routingContext().roundTripDistance <= 0) ? 1500 : ops.routingContext().roundTripDistance;
      }

      // Fail fast on a missing start tile. Start-tile availability is invariant
      // across every attempt below (direction × subRouteCount × AUTO candidate),
      // yet the greedy/iso paths discover it only lazily — and every earlier touch
      // point (the reachability probe, isochrone expansion) swallows the
      // IllegalArgumentException — so without this it is either re-discovered per
      // attempt or, in AUTO, wrapped as a generic "candidate threw" failure.
      // Checking once here surfaces the canonical "datafile … not found" before any
      // provider/isochrone/competition work, and keeps a missing-data start from
      // being mislabeled "start point not on road network" by the greedy planner
      // (whose matchPoint now uniformly maps any match failure to null).
      String tileError = ops.startTileMissingError(ops.waypoints().get(0));
      if (tileError != null) {
        setError(tileError);
        ops.logInfo(request.error);
        return;
      }

      double direction = (ops.routingContext().startDirection == null ? -1 :ops.routingContext().startDirection);
      double directionAdd = (ops.routingContext().roundTripDirectionAdd == null ? ROUNDTRIP_DEFAULT_DIRECTIONADD :ops.routingContext().roundTripDirectionAdd);
      if (direction == -1) {
        direction = ops.getRandomDirectionFromData(ops.waypoints().get(0), searchRadius);
        direction += directionAdd;
      }
      // Normalize to a [0,360) compass bearing: ops.getRandomDirectionFromData()+directionAdd
      // can exceed 360 (e.g. 332+45=377), and a user-supplied startDirection may be out of
      // range, while downstream bearing comparisons assume a normalized value.
      direction = CheapAngleMeter.normalize(direction);

      // Explicit-via round-trip: when the caller supplied via points (any
      // waypoint beyond the start), treat those vias as a hard route
      // skeleton and bypass all generated-loop placement, regardless of
      // roundTripAlgorithm. User vias express stronger intent than any AUTO
      // heuristic, so they win. Generated rt* points are never added; the via
      // order is preserved exactly; distance settings become advisory.
      boolean explicitViaMode = ops.waypoints().size() > 1;
      if (explicitViaMode) {
        ops.logInfo("round trip: explicit-via mode (" + (ops.waypoints().size() - 1) + " user via points)");
        // Variety-seed disclosure: user vias are a hard skeleton expressing
        // stronger intent than any heuristic, so the alternativeidx seed is ignored.
        if (ops.routingContext().getRoundTripSeed() > 0) {
          ops.logInfo("alternativeidx has no effect in explicit-via round trips");
        }
        doExplicitViaRoundTrip(searchRadius, direction);
      } else {
        // Product sizing gate: the standard loop class is 40-100km and up to
        // 200km must work without special action; ABOVE 200km the caller must
        // explicitly ask for a longer calculation by raising the request
        // timeout (server: -DmaxRunningTime, embedders: doRun budget). A
        // default 60s budget cannot fund a good 200km+ loop, so failing fast
        // with instructions beats a guaranteed degraded result. Untimed
        // callers (budget <= 0, e.g. CLI) are already explicit and pass.
        double requestedLoopMeters = 2 * Math.PI * searchRadius;
        if (requestedLoopMeters > MAX_STANDARD_LOOP_METERS
            && ops.maxRunningTime() > 0 && ops.maxRunningTime() < LONG_LOOP_MIN_BUDGET_MS) {
          setError("round trips above " + (int) (MAX_STANDARD_LOOP_METERS / 1000)
            + "km need an explicitly increased calculation budget: requested "
            + Math.round(requestedLoopMeters / 1000.0) + "km with a "
            + (ops.maxRunningTime() / 1000) + "s timeout; raise maxRunningTime to at least "
            + (LONG_LOOP_MIN_BUDGET_MS / 1000) + "s");
          ops.logInfo(request.error);
          return;
        }
        // Resolve the roundTripIsochrone shortcut into the canonical
        // roundTripAlgorithm ONCE, so the algorithm is the single source of
        // truth from here down and the boolean never has to propagate to child
        // contexts. Honoured only when no explicit algorithm was chosen — an
        // explicit algorithm always wins.
        if (ops.routingContext().roundTripAlgorithm == RoundTripAlgorithm.AUTO
            && ops.routingContext().roundTripIsochrone) {
          ops.routingContext().roundTripAlgorithm = RoundTripAlgorithm.ISOCHRONE;
        }
        RoundTripAlgorithm algo = ops.routingContext().roundTripAlgorithm;

        for (Rung rung : resolveLadder(algo, searchRadius, direction)) {
          if (!rung.strategy.attempt(request, rung.slice)) {
            // The strategy finalized the result itself: the competition tiers
            // gate their candidates internally and decorate the winner.
            return;
          }
          if (request.track != null || request.error != null) {
            break; // outcome decided — hand it to the shared floors + gate below
          }
        }
      }

      if (request.track == null && request.error != null) {
        return;
      }

      // A loop needs at least a triangle (start + 2 intermediate ops.waypoints()). With a single
      // intermediate the route is only an out-and-back, which closure/detour handling cannot
      // turn into a loop. Same-way-back is the deliberate exception (it IS an out-and-back).
      //
      // Explicit-via mode skips this check: a single user-supplied via is a valid
      // route skeleton (start → via1 → start), even though the result shape is
      // out-and-back. The user is expressing route intent, not a loop request.
      int intermediateWaypoints = (ops.matchedWaypoints() == null) ? 0 : ops.matchedWaypoints().size() - 2;
      if (!ops.routingContext().allowSamewayback && !explicitViaMode
          && intermediateWaypoints < MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS) {
        setError("round-trip could not place enough waypoints to form a loop (need "
          + MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS + " intermediate, got " + Math.max(0, intermediateWaypoints)
          + ") for direction " + (int) direction + " at radius " + (int) searchRadius + "m");
        ops.logInfo(request.error);
        setTrack(null);
        return;
      }

      // Contract: a round-trip must yield an actual loop. When intermediate ops.waypoints()
      // cannot be placed on reachable roads (e.g. the requested direction has no roads
      // within this radius), routing collapses to a 1-3 node stub. Report that as a
      // failure rather than returning a non-loop as success.
      //
      // Explicit-via mode also bypasses the strict node/length floors: a short
      // one-via route may produce fewer than MIN_ROUNDTRIP_LOOP_NODES if the
      // via is right next to the start. We still reject null/no-track outcomes
      // below as a safety net.
      if (request.track == null || request.track.nodes == null
          || (!explicitViaMode && (request.track.nodes.size() < MIN_ROUNDTRIP_LOOP_NODES
                                || request.track.distance < MIN_ROUNDTRIP_LOOP_METERS))) {
        int n = (request.track == null || request.track.nodes == null) ? 0 : request.track.nodes.size();
        int d = request.track == null ? 0 : request.track.distance;
        setError("round-trip could not form a loop for direction " + (int) direction
          + " at radius " + (int) searchRadius + "m (only " + n + " nodes, " + d
          + "m) — no reachable roads in that direction at this distance");
        ops.logInfo(request.error);
        setRejectedTrack(request.track); // preserve stub for post-mortem
        setTrack(null);
        return;
      }

      // Production-safety acceptance gate: applied uniformly across all
      // round-trip algorithms (WAYPOINT/ISOCHRONE/GREEDY/ISO_GREEDY) right
      // before returning success. The gate rejects unsafe routes (beeline
      // segments, broken closure, distance way off, profile-hostile surfaces,
      // accidental mid-route backtracking). Acceptance is shape-aware:
      // STRICT_LOOP/LOLLIPOP/OUT_AND_BACK each get explicit
      // disclosures so the cyclist knows what they're getting; only
      // INVALID_RETRACE is rejected. See {@link RoundTripQualityGate}.
      double expectedDistance = 2 * Math.PI * searchRadius;
      // Reuse the bounded tier's verdict when it evaluated this same track
      // (set only when the planner track survived its pre-gate; the fallback
      // path leaves it null). Consumed once.
      RoundTripQualityResult quality = request.boundedGateVerdict != null
        ? request.boundedGateVerdict
        : evaluateRoundTripGate(request.track, searchRadius, explicitViaMode);
      request.boundedGateVerdict = null;
      if (!quality.isAccepted()) {
        // STRUCTURAL failures (broken / un-routable / not-a-loop) are always
        // hard-rejected — there is nothing usable to offer. QUALITY failures
        // (distance off-target, self-crossing/hairpin chaos, hostile surface,
        // mid-route backtracking) are advisory by default: the route is
        // rideable, so we return it with a Warning and let the user decide.
        // roundTripStrictQuality=1 restores the old hard-reject behaviour.
        boolean hardReject = ops.roundTripQualityHardReject(quality);
        if (hardReject) {
          setError("round-trip rejected by quality gate (direction " + (int) direction
            + ", radius " + (int) searchRadius + "m, shape=" + quality.getShape() + "): "
            + quality.getRejectionReason());
          ops.logInfo(request.error);
          setRejectedTrack(request.track);
          setTrack(null);
          return;
        }
        // Lenient default: surface the quality issue as a warning and keep the
        // route. The planner already searched strictly and shipped its best
        // effort; we disclose the problem rather than discard a rideable loop.
        String advisory = "Warning: " + quality.getRejectionReason()
          + " (shape=" + quality.getShape() + ") — route returned anyway; ride at your"
          + " discretion, or set roundTripStrictQuality=1 to reject it.";
        ops.logInfo("round-trip quality advisory (lenient): " + advisory);
        appendRouteMessage(request.track, advisory);
        // fall through to disclosure surfacing + success
      }
      // Surface the route shape + disclosures (e.g. "contains retraced
      // scenic spur: 4.2km") so the cyclist isn't surprised to find
      // they're returning the same way along a stretch. Stays in the
      // route message stream so it propagates to GPX/JSON exports.
      ops.logInfo("round-trip quality: " + quality);
      for (String d : quality.getDisclosures()) {
        appendRouteMessage(request.track, d);
      }

      // Transparency for the silent band: 1..MAX crossings and guard-blocked
      // spurs pass the gate without any message, yet the cyclist sees them on
      // the map. Disclose every nonzero count — informational only, the route
      // ships either way (lenient product policy: odd-but-cycleable > nothing).
      // The whole decoration block runs under its own guard: the loop is
      // complete and gate-accepted at this point, and the outer catch nulls
      // request.track — an exception in a cosmetic advisory must never destroy
      // a rideable result.
      try {
        int shippedCrossings = RoundTripQualityGate.countSelfIntersections(request.track);
        if (shippedCrossings > 0) {
          appendRouteMessage(request.track, String.format(Locale.US,
            "Note: route crosses its own path %d time%s.",
            shippedCrossings, shippedCrossings == 1 ? "" : "s"));
        }
        if (request.track.nodes != null) {
          int[] spurInfo = LoopQualityMetrics.computeSpurInfo(request.track.nodes);
          if (spurInfo[0] > 0 && spurInfo[1] > 600) {
            appendRouteMessage(request.track, String.format(Locale.US,
              "Note: route contains %d out-and-back section%s (longest %.1fkm).",
              spurInfo[0], spurInfo[0] == 1 ? "" : "s", spurInfo[1] / 1000.0));
          }
        }

        // Residual-chord advisory (loop-review backlog item 1): the planner's
        // fidelity enforcement retries chord legs, but a best-effort adoption or
        // a non-greedy path can still ship a long null-tag edge that renders as
        // a straight line cutting across terrain. Ground truth (Lozère study):
        // these follow a real curving road whose detail is missing, so the route
        // is rideable — disclose, don't reject. Same threshold as the planner's
        // fidelity check so the two mechanisms never disagree about what a
        // chord is.
        int chordMeters = LoopQualityMetrics.maxSingleNullEdgeMeters(request.track);
        if (chordMeters > GreedyRoundTripPlanner.MAX_UNDETAILED_EDGE_METERS) {
          appendRouteMessage(request.track, String.format(Locale.US,
            "Note: route contains an undetailed straight-line section of ~%dm "
              + "(way detail missing in the map data; the actual road may curve).",
            chordMeters));
        }

        // Soft advisory: even within the [0.5, 1.8] ratio band, a >1.5
        // overshoot is worth flagging so the caller can suggest a shorter
        // distance. This stays informational because the hard gate above
        // already rejects ratios outside the safe range.
        if (request.track.distance > 0) {
          double ratio = request.track.distance / expectedDistance;
          if (ratio > 1.5) {
            String warning = String.format(
              "Warning: route distance (%dkm) exceeds requested loop distance (%dkm) by %.0f%%. "
              + "The road network in this area is too constrained for a compact loop at this distance. "
              + "Consider a shorter distance or an out-and-back route.",
              request.track.distance / 1000, (int) (expectedDistance / 1000), (ratio - 1) * 100);
            ops.logInfo(warning);
            appendRouteMessage(request.track, warning);
          }
        }

        // The advisory/disclosures above were appended to request.track.message, but
        // FormatGpx emits <brouter:info> and its message comments from
        // messageList, not message. Sync messageList[0] so the quality warning
        // actually reaches GPX/JSON consumers. Idempotent; no-op for the AUTO
        // path (which returns earlier and syncs via adoptCandidateWinner).
        cleanup.ensureInfoMessage(request.track);
      } catch (RuntimeException advisoryFailure) {
        ops.logInfo("round-trip advisory decoration failed ("
          + advisoryFailure.getClass().getSimpleName()
          + "); returning the track without advisories");
        ops.logThrowable(advisoryFailure);
      }

      long endTime = System.currentTimeMillis();
      ops.logInfo("round trip execution time = " + (endTime - wallStart) / 1000. + " seconds");
    } catch (Exception e) {
      ops.logException(e);
      ops.logThrowable(e);
      // logException publishes the exception text on the ENGINE's error field;
      // mirror it into the request, which is the working copy the finally
      // publishes.
      setError(ops.errorMessage());
      // Contract: a round trip ends with a usable track XOR a clean error. An
      // exception can land here before any assignment, leaving request.track as
      // the constructor's initial empty OsmTrack (or a partial one) — and
      // logException copies e.getMessage(), which is null for message-less
      // exceptions. Guarantee both halves of the contract: a non-empty error
      // and no degenerate "success" track. Non-empty geometry is preserved on
      // request.lastRejectedTrack for post-mortem inspection like other reject paths.
      if (request.error == null || request.error.isEmpty()) {
        setError("round trip failed: " + e.getClass().getSimpleName());
      }
      if (request.track != null && request.track.nodes != null && !request.track.nodes.isEmpty()) {
        request.lastRejectedTrack = request.track;
      }
      setTrack(null);
    } finally {
      // Final result + telemetry publication: the engine's public getters
      // (getFoundTrack/getErrorMessage/getLastRejectedTrack/
      // getLastRoundTripResult) serve these after the request; nothing
      // engine-side reads them mid-request.
      ops.setFoundTrack(request.track);
      ops.setErrorMessage(request.error);
      ops.setLastRejectedTrack(request.lastRejectedTrack);
      ops.setLastRoundTripResult(request.lastResult);
      ops.cleanupRoutingResources();
    }

  }

  /**
   * Explicit-via round-trip: route through the caller's via points exactly, in
   * input order, with no generated {@code rt*} waypoints.
   *
   * <p>Skeleton:
   * <ul>
   *   <li>{@code allowSamewayback=false}: {@code start → via1 → ... → viaN → start}</li>
   *   <li>{@code allowSamewayback=true}: forward chain only; {@code doRouting} mirrors it back.</li>
   * </ul>
   *
   * <p>{@code roundTripPoints} is ignored; {@code roundTripDistance}/{@code roundTripLength}
   * and {@code startDirection} are advisory (distance-ratio mismatch becomes a
   * disclosure, not a rejection; direction does not reorder vias). A via that
   * cannot be snapped within range fails with an error naming it — user vias are
   * hard constraints, never silently dropped (no-beeline invariant).
   *
   * @param searchRadius sizes the snap tolerance; also logged
   * @param direction    logged only; does not reorder vias
   */
  private void doExplicitViaRoundTrip(double searchRadius, double direction) {
    OsmNodeNamed start = ops.waypoints().get(0);
    List<OsmNodeNamed> userVias = new ArrayList<>(ops.waypoints().subList(1, ops.waypoints().size()));
    ops.waypoints().subList(1, ops.waypoints().size()).clear();
    // Default-name only blanks; preserve any user-supplied via names so that
    // diagnostic output references the user's identifiers.
    for (int i = 0; i < userVias.size(); i++) {
      OsmNodeNamed v = userVias.get(i);
      if (v.name == null || v.name.isEmpty()) {
        v.name = "via" + (i + 1);
      }
    }

    // Snap start and every user via. Failure on a user via is fatal and
    // names the via — explicit vias are hard constraints, never dropped.
    // Note: `snapper.snapStartToRoad(ops.waypoints(), ...)` short-circuits when
    // ops.waypoints().size() < 2, so we snap the start directly via the
    // single-waypoint helper to avoid that early-return.
    double userSnapDist = Math.min(searchRadius * 0.3, 2000);
    snapper.snapStartProfileAware(start, userSnapDist);
    List<Boolean> matched = snapper.snapWaypointsToRoad(userVias, userSnapDist, "snapUserVia");
    for (int i = 0; i < userVias.size(); i++) {
      if (!matched.get(i)) {
        throw new IllegalArgumentException("user waypoint " + userVias.get(i).name
          + " has no road within " + (int) userSnapDist + "m");
      }
    }
    // Densification gate (ship A gated). OFF by default: inserting generated bulge points
    // would violate the user-via skeleton contract (no generated ops.waypoints(), order preserved),
    // so it must be explicitly opted into ({@code roundTripDensify=1} →
    // {@code explicitViaDensifyOverride=TRUE}) — a "length-honoring loop" mode. Even when opted
    // in it is gated to NON-PAVED profiles: for a road bike in sparse terrain a retracing paved
    // lollipop beats a one-way track loop the quality gate would reject, so paved keeps the
    // plain route.
    ops.routingContext().explicitViaDensify =
      Boolean.TRUE.equals(ops.routingContext().explicitViaDensifyOverride)
        && !RoundTripQualityGate.isPavedProfile(ops.routingContext().getProfileName());

    // Anchor cycle [start, via1, ..., viaN]. With densification on, insert generated
    // arc-following "bulge" points between consecutive anchors so legs follow the loop
    // perimeter instead of cutting the chord (corner-cut undershoot fix).
    List<OsmNodeNamed> anchors = new ArrayList<>();
    anchors.add(start);
    anchors.addAll(userVias);

    ops.waypoints().clear();
    if (ops.routingContext().explicitViaDensify && !ops.routingContext().allowSamewayback && anchors.size() >= 2) {
      ops.waypoints().addAll(snapper.densifyViaArcs(anchors, searchRadius, userSnapDist));
    } else {
      ops.waypoints().addAll(anchors);
    }

    // For allowSamewayback=false append the closing start copy so the route
    // forms a closed loop. For allowSamewayback=true the existing doRouting
    // expansion at the top of {@link #doRouting} mirrors the chain back —
    // we must NOT add a closing copy here or we'd double-close.
    if (!ops.routingContext().allowSamewayback) {
      OsmNodeNamed closing = new OsmNodeNamed(new OsmNode(start.ilon, start.ilat));
      closing.name = "to";
      ops.waypoints().add(closing);
    }

    ops.routingContext().waypointCatchingRange = 250;
    request.searchRadius = searchRadius;
    request.explicitVia = true;
    publishRuntimeHints();
    ops.logInfo("explicit-via round-trip: " + userVias.size() + " user via(s), "
      + "allowSamewayback=" + ops.routingContext().allowSamewayback
      + ", direction=" + (int) direction + " (advisory only)");
    doRoutingIntoRequest(request.routingBudgetMs);
  }

  private void doWaypointBasedRoundTrip(double searchRadius, double direction, RoundTripAlgorithm algo) {
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
          "doWaypointBasedRoundTrip expects a single start waypoint; user vias must be "
            + "handled by doExplicitViaRoundTrip (got " + ops.waypoints().size() + ")");
      }

      int targetPoints = ops.routingContext().roundTripPoints == null ?
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
      boolean fastOptimized = !"false".equals(
        System.getProperty("roundtrip.fast.optimized", "true"));

      if (algo == RoundTripAlgorithm.ISOCHRONE) {
        ProbeResult probe = snapper.probeReachableDirections(ops.waypoints().get(0), searchRadius);
        double[] probeDirections = (probe != null) ? probe.viableDirections : null;
        IsochroneExpansionResult iso = ops.runIsochroneExpansion(ops.waypoints().get(0), searchRadius);
        double[][] frontier = (iso != null) ? iso.frontier : null;
        double[][] merged = GeometricWaypointPlacer.mergeIsochroneWithProbe(frontier, probeDirections, searchRadius);
        if (merged != null && merged.length >= 3) {
          List<IsoCandidate> isoCandidates = (iso != null) ? iso.candidates : null;
          recordPlacementPath(PlacementPath.ISOCHRONE);
          placer.placeWaypointsFromIsochrone(ops.waypoints(), merged, isoCandidates, searchRadius, direction, targetPoints);
        } else if (probeDirections != null && probeDirections.length >= 3) {
          ops.logInfo("isochrone merge insufficient, falling back to probe directions");
          recordPlacementPath(PlacementPath.ENVELOPE_ISO_FALLBACK);
          placer.placeWaypointsFromEnvelope(ops.waypoints(), probeDirections, searchRadius, direction, targetPoints);
        } else {
          ops.logInfo("both isochrone and probe insufficient, falling back to circle");
          recordPlacementPath(PlacementPath.CIRCLE);
          ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
        }
      } else if (fastOptimized) {
        // Directional lobe (opt-in): head the loop toward the requested bearing like
        // the pre-903 routine, instead of encircling the start. Off by default while
        // the sparse-terrain robustness (routing between forward-arc vias can fail
        // where an encircling ring would not) is finished via a post-routing retry.
        FastPlacementRequest fastRequest = new FastPlacementRequest(
          ops.waypoints().get(0), searchRadius, direction, targetPoints,
          direction >= 0
            && "true".equals(System.getProperty("roundtrip.fast.directional", "false")),
          Integer.getInteger("roundtrip.fast.maxvias", 5));
        // Placement builds its skeleton on a local list and the outcome is
        // committed in one step — a degraded or failed attempt can never
        // leave partial vias in the live waypoint list.
        FastPlacementOutcome fastOutcome =
          new FastWaypointPlanner(ops.fastPlacementOps()).place(fastRequest);
        ops.waypoints().clear();
        ops.waypoints().addAll(fastOutcome.skeleton);
        recordPlacementPath(fastOutcome.optimizedPlacement()
          ? PlacementPath.ENVELOPE_FAST : PlacementPath.CIRCLE);
      } else {
        ProbeResult probe = snapper.probeReachableDirections(ops.waypoints().get(0), searchRadius);
        // FAST tier: drop single-probe-success directions when enough strong
        // alternatives exist. Avoids fragile sea-edge/dead-end picks.
        double[] viableDirections = PlacementGeometry.filterByProbeConfidence(probe, targetPoints);
        if (viableDirections != null && viableDirections.length >= 3) {
          recordPlacementPath(PlacementPath.ENVELOPE_FAST);
          placer.placeWaypointsFromEnvelope(ops.waypoints(), viableDirections, searchRadius, direction, targetPoints);
        } else {
          ops.logInfo("reachability probe returned < 3 directions, falling back to circle");
          recordPlacementPath(PlacementPath.CIRCLE);
          ops.buildPointsFromCircle(ops.waypoints(), direction, searchRadius, targetPoints);
        }
      }

      // Idea 4: the optimized FAST module fully validates its own skeleton —
      // probe-snapped vias are pre-validated, and its circle fallback runs this
      // pass behind FastPlacementOps.circleFallbackValidated. Only the ISOCHRONE
      // and legacy A/B placements need the caller-side matching pass.
      if (algo == RoundTripAlgorithm.ISOCHRONE || !fastOptimized) {
        snapper.validateAndAdjustWaypoints(ops.waypoints(), searchRadius);
      }

      // Snap start/end ops.waypoints() to nearest road to prevent beeline segments.
      // Without this, if the user's click position is >250m from a road (park,
      // water, etc.), the routing engine inserts straight-line beelines.
      snapper.snapStartToRoad(ops.waypoints(), searchRadius);
    }

    ops.routingContext().waypointCatchingRange = 250;
    request.searchRadius = searchRadius;
    publishRuntimeHints();
    doRoutingIntoRequest(request.routingBudgetMs);
  }

  void doGreedyRoundTrip(double searchRadius, double direction, RoundTripAlgorithm algo) {
    // Initialize nodesCache — needed before the planner can match ops.waypoints() to the graph.
    ops.resetCache(false);
    request.forcedCorridorAccepted = false;
    // Loop scale for the via-relocation bound (profileAwareMatchPoint): must be
    // set BEFORE planner via matching — the doRouting fallthrough below used to
    // set it only late, leaving the bound inert during greedy placement.
    request.searchRadius = searchRadius;
    publishRuntimeHints();

    OsmNodeNamed start = ops.waypoints().get(0);
    double desiredDistance = 2 * Math.PI * searchRadius;
    ops.logInfo("greedy round trip: desired distance=" + (int) desiredDistance
      + "m, searchRadius=" + (int) searchRadius + "m, direction=" + (int) direction
      + ", mode=" + algo);

    // Phase 2.0: when ISO_GREEDY runs without an explicit user direction,
    // use the isochrone's reachability asymmetry to bias the initial bearing
    // toward the most-reaching sector. The legacy default of "direction=-1"
    // (ANY) means candidate scoring's direction term is inert at step 1,
    // and the candidate placement uses an unrelated heuristic. On terrain-
    // asymmetric networks (coast, valley, island) this can place initial
    // candidates in geographically unreachable regions. The asymmetry bias
    // grounds the initial direction in actual graph reachability.
    //
    // Bias applies ONLY when:
    //   - algo == ISO_GREEDY (we need the frontier table)
    //   - direction < 0 (user did not specify a direction)
    //   - at least one bucket meets quality thresholds (airDist >= 0.6 *
    //     searchRadius AND hits >= 3)
    // Otherwise direction is preserved verbatim.
    // NOTE (measured 2026-07-04, do not re-attempt without new evidence):
    // adopting the expansion's compiled step-1 legs as planner sub-legs
    // (includeCandidateTracks=true here + routedTrack forwarding at step 1)
    // was implemented and A/B-measured on the deterministic Basel matrix.
    // Result: quality-neutral mean with a systematic short-bias (exact
    // Dijkstra legs are shorter than the pass1coefficient-directed legs the
    // planner is tuned around), one deterministic shipped regression
    // (basel_30km_gravel_W: AUTO 0.84 -> 0.58 composite, 21km for a 30km
    // request), and no latency win (+4%: track-compile overhead outweighed
    // the saved step-1 re-routes). Reverted; diff preserved in the session
    // findings.
    IsochroneExpansionResult iso = algo == RoundTripAlgorithm.ISO_GREEDY
      ? ops.runIsochroneExpansion(start, searchRadius)
      : null;
    double effectiveDirection = direction;
    IsoAsymmetryBias bias = IsoAsymmetryBias.NONE;
    if (algo == RoundTripAlgorithm.ISO_GREEDY && direction < 0 && iso != null) {
      bias = GeometricWaypointPlacer.computeIsoAsymmetryBearing(iso.frontier, searchRadius);
      if (bias.applied) {
        effectiveDirection = bias.bearingDegrees;
        ops.logInfo("ISO_GREEDY: iso-asymmetry bias selected bearing="
          + (int) bias.bearingDegrees + "° (indirectness=" + String.format("%.2f", bias.indirectness)
          + ", hits=" + bias.hits + ", airDist=" + bias.airDistMeters + "m)");
      }
    }
    GraphNativeCandidateProvider graphNativeProvider = new GraphNativeCandidateProvider(ops, ops);
    RoundTripCandidateProvider provider = buildCandidateProvider(algo, start, searchRadius,
      effectiveDirection, iso, graphNativeProvider);
    int baseSubRouteCount = selectGreedySubRouteCount(desiredDistance, ops.routingContext().getProfileName());

    // Return-distance oracle (F6): sector-resolved return estimates from the
    // start-centered pool expansion when one exists (ISO_GREEDY — largest
    // coverage). Plain GREEDY deliberately has no oracle: a step-1 expansion
    // oracle was measured quality-negative, so null means the planner falls
    // back to the global-EMA estimate everywhere.
    ReturnDistanceOracle returnOracle = ReturnDistanceOracle.build(iso, start.ilon, start.ilat);
    if (returnOracle != null) {
      ops.logInfo("greedy: return oracle from pool expansion (kappa="
        + String.format(Locale.ROOT, "%.2f", returnOracle.kappa()) + ")");
    }

    // Iso-pool shape for the planner's health tracker (issue #26): measured
    // once per pool, shared across the ladder rungs (each rung wraps it in a
    // fresh per-plan IsoPoolHealth). Null when the provider is graph-native
    // only — the planner then skips every health hook (plain GREEDY parity).
    IsoPoolHealth.PoolShape poolShape = null;
    if (provider instanceof BlendedCandidateProvider) {
      IsochroneCandidateProvider isoProvider = ((BlendedCandidateProvider) provider).isoProvider();
      poolShape = new IsoPoolHealth.PoolShape(isoProvider.poolSize(),
        isoProvider.distinctSectorCount(), isoProvider.angularSpanDegrees(),
        isoProvider.contourLevelCount(), returnOracle != null);
      ops.logInfo("ISO_GREEDY: iso-pool shape: " + poolShape.describe());
    }

    FrontierAxis frontierAxis = (algo == RoundTripAlgorithm.ISO_GREEDY && iso != null)
      ? GeometricWaypointPlacer.computeFrontierAxis(iso.frontier, searchRadius) : FrontierAxis.NONE;
    IsoStartPolicy isoStartPolicy = algo == RoundTripAlgorithm.ISO_GREEDY
      ? selectIsoStartPolicy(poolShape)
      : IsoStartPolicy.BLEND;
    if (algo == RoundTripAlgorithm.ISO_GREEDY) {
      ops.logInfo("ISO_GREEDY: start policy " + isoStartPolicy);
    }

    boolean startGraphNativeOnly = isoStartPolicy == IsoStartPolicy.GRAPH_NATIVE_ONLY;
    RoundTripCandidateProvider primaryProvider = startGraphNativeOnly ? graphNativeProvider : provider;
    // The return oracle survives a graph-native-only start: it calibrates from
    // the raw expansion cell cloud, not the filtered pool, so it stays valid in
    // exactly the constrained terrain that demotes the pool.
    ReturnDistanceOracle primaryReturnOracle = returnOracle;
    IsoPoolHealth.PoolShape primaryPoolShape = startGraphNativeOnly ? null : poolShape;

    // First attempt — user direction (or Phase 2.0 biased bearing).
    RoundTripResult result = runGreedyAttempt(start, searchRadius, desiredDistance,
      effectiveDirection, baseSubRouteCount, primaryProvider, bias,
      primaryReturnOracle, primaryPoolShape, isoStartPolicy);

    // Phase 2.1: if the first attempt degraded AND the user supplied an
    // explicit direction AND the frontier has a strong terrain axis AND
    // the user's direction is perpendicular to that axis, retry once
    // along the axis. This addresses the Inn-Valley pattern: 100km loop
    // requested heading N where the road network only supports E-W.
    boolean phase21Triggered = false;
    boolean phase21Succeeded = false;
    double phase21RetryDir = Double.NaN;
    // Bounded effort: the axis retry re-runs the whole ladder exactly when
    // the terrain is hard — the opposite of a bounded tier's contract.
    // (Deliberately NOT gated on the start policy: corridor terrain is both
    // what demotes the pool and what the axis retry exists to recover.)
    if (!request.effortPolicy.skipRetryLayers
        && isDegradedGreedyResult(result)
        && direction >= 0
        && frontierAxis.hasStrongAxis
        && GeometricWaypointPlacer.isPerpendicularToAxis(direction, frontierAxis.axisBearingDegrees)
        // Request-budget gate: the axis retry re-runs the whole subRouteCount
        // ladder — only worth starting when the request can still fund it.
        && ops.remainingRequestBudgetMs() >= MIN_LADDER_RUNG_BUDGET_MS) {
      phase21Triggered = true;
      phase21RetryDir = GeometricWaypointPlacer.chooseAxisBearing(frontierAxis.axisBearingDegrees, direction);
      ops.logInfo("ISO_GREEDY: Phase 2.1 axis retry — user direction " + (int) direction
        + "° is perpendicular to terrain axis " + String.format("%.0f", frontierAxis.axisBearingDegrees)
        + "° (strength=" + String.format("%.1fx", frontierAxis.strength)
        + "); retrying with axis-aligned direction " + (int) phase21RetryDir + "°");
      RoundTripResult retry = runGreedyAttempt(start, searchRadius, desiredDistance,
        phase21RetryDir, baseSubRouteCount, provider, bias, returnOracle, poolShape,
        IsoStartPolicy.BLEND);
      if (!isDegradedGreedyResult(retry)
          && retry != null && retry.getLoopWaypoints() != null
          && retry.getLoopWaypoints().size() >= 4) {
        phase21Succeeded = true;
        result = retry;
      } else {
        // Retry also degraded → geographic infeasibility. Keep first-attempt
        // result for diagnostic display but mark the infeasibility for the
        // caller's error path below.
        ops.logInfo("ISO_GREEDY: Phase 2.1 axis retry ALSO degraded — geographic infeasibility detected");
      }
    }

    RouteChoiceScore.Verdict blendedInternalVerdict = null;
    boolean runInternalBranch = false;
    if (algo == RoundTripAlgorithm.ISO_GREEDY
        && ops.routingContext().roundTripInternalCompare
        && !startGraphNativeOnly
        // QUALITY (runGreedyAlways) already fields a dedicated plain-GREEDY
        // child in the parent competition — this internal comparison would run
        // materially the same graph-native ladder a second time.
        && !request.effortPolicy.runGreedyAlways
        && provider instanceof BlendedCandidateProvider
        && System.currentTimeMillis() < (request.requestDeadline == 0
            ? Long.MAX_VALUE : request.requestDeadline)) {
      // Evaluate the blended verdict ONCE; the selection below reuses it.
      blendedInternalVerdict = scoreInternalGreedyResult(result, desiredDistance, effectiveDirection);
      runInternalBranch = internalBranchNeeded(blendedInternalVerdict);
    }
    if (runInternalBranch) {
      ops.logInfo("ISO_GREEDY: running internal graph-native-only comparison branch");
      // Ladder order: BLEND (base first), NOT GRAPH_NATIVE_ONLY (base-1 first).
      // This branch replaced the ISO_GREEDY→GREEDY recursion, which ran the
      // BLEND-order ladder — and the fewer-steps-first order is measurably
      // wrong here: at mallorca_30km_gravel_W the base-1 rung returns a
      // non-degraded 10.4%-error plan that STOPS the ladder with a 4-point
      // loop routing to distR 0.62 (the undershoot-sentinel contraction
      // class), while the base rung produces the healthy loop the old
      // recursion shipped. Fewer-first remains correct for the START policy
      // (pool unhealthy from step 0), which keeps GRAPH_NATIVE_ONLY.
      RoundTripResult graphNativeResult = runGreedyAttempt(start, searchRadius, desiredDistance,
        effectiveDirection, baseSubRouteCount, graphNativeProvider, bias, null, null,
        IsoStartPolicy.BLEND);
      RouteChoiceScore.Verdict graphNativeVerdict = scoreInternalGreedyResult(
        graphNativeResult, desiredDistance, effectiveDirection);
      boolean comparable = graphNativeVerdict != null;
      RoundTripResult selected = selectBetterInternalIsoGreedyResult(
        result, blendedInternalVerdict, graphNativeResult, graphNativeVerdict);
      if (selected == graphNativeResult) {
        ops.logInfo("ISO_GREEDY: internal graph-native branch selected");
      } else if (comparable) {
        ops.logInfo("ISO_GREEDY: blended branch kept after internal graph-native comparison");
      } else {
        ops.logInfo("ISO_GREEDY: internal graph-native branch produced no comparable track");
      }
      result = selected;
      if (comparable && result != null) {
        result.setInternalGraphNativeCompared(true);
      }
      setPlannerResult(result);
    }

    if (result != null) {
      // The explicit record of the shipped result's candidate source. When no
      // blend exists at all (pool not admitted → `provider` IS the graph-native
      // provider), every attempt — including a successful Phase 2.1 axis retry —
      // planned on graph-native candidates. With an admitted blend, a
      // successful retry ran the blend, so only the primary attempt's start
      // policy counts.
      boolean blendAvailable = provider instanceof BlendedCandidateProvider;
      result.setGraphNativeOnlyStart(!blendAvailable
        || (startGraphNativeOnly && !phase21Succeeded));
      result.setPhase21AxisRetryTriggered(phase21Triggered);
      result.setPhase21AxisRetrySucceeded(phase21Succeeded);
      result.setPhase21AxisBearingDegrees(frontierAxis.hasStrongAxis
        ? frontierAxis.axisBearingDegrees : Double.NaN);
      result.setPhase21AxisStrength(frontierAxis.hasStrongAxis ? frontierAxis.strength : 0.0);
      result.setPhase21RetryDirectionDegrees(phase21RetryDir);
    }

    // Phase 2.1 used to also set request.error when both attempts degraded
    // (the spec's "refuse with infeasibility error" option). That cut off
    // doRoundTrip's later fallback path (waypoint algorithm), losing 2
    // iso_greedy/gravel scenarios on the broader corpus that the legacy
    // waypoint fallback had been salvaging. Drop the request.error write;
    // let the result return as degraded so the caller can fall back as
    // before. The axis info is still surfaced via the Phase 2.1 telemetry
    // fields on RoundTripResult for diagnostic purposes.
    if (phase21Triggered && !phase21Succeeded) {
      ops.logInfo("ISO_GREEDY: Phase 2.1 axis retry also degraded — geographic"
        + " infeasibility (axis " + axisName(frontierAxis.axisBearingDegrees)
        + ", strength " + String.format("%.1fx", frontierAxis.strength)
        + "); falling through to legacy fallback chain");
    }

    // A real loop needs at least a triangle: start + 2 intermediate ops.waypoints() + closing
    // start (>= 4 entries). A single intermediate is just an out-and-back, so reject it
    // rather than attributing a legacy waypoint/probe fallback route to GREEDY.
    // Reject loops the planner explicitly flagged as failing its quality gates
    // (DEGRADED_FALLBACK_PREFIX) — shipping a 180% overshoot or 60%-reused
    // forced-closure loop as success would silently fool downstream consumers.
    request.forcedCorridorAccepted = result != null && result.isForcedCorridorAccepted();
    boolean degradedFallback = isDegradedGreedyResult(result);
    if (degradedFallback) {
      ops.logInfo("greedy: rejecting degraded fallback (" + result.getFallbackReason()
        + ")");
    }
    if (!degradedFallback
        && result != null && result.getLoopWaypoints() != null
        && result.getLoopWaypoints().size() >= 4) {
      for (String diag : result.getDiagnostics()) {
        ops.logInfo("greedy: " + diag);
      }
      // Spec §10 telemetry — compute-budget audit.
      ops.logInfo("greedy telemetry: candidatesGenerated=" + result.getCandidatesGenerated()
        + ", candidatesRouted=" + result.getCandidatesRouted()
        + ", returnChecks=" + result.getReturnChecksPerformed()
        + ", runtimeMs=" + result.getRuntimeMillis()
        + ", fallbackReason=" + (result.getFallbackReason() == null ? "none" : result.getFallbackReason()));
      // Issue #26 source attribution — the aggregate view of the per-leg
      // "leg N source:" diagnostics logged above.
      ops.logInfo("greedy source attribution: acceptedIso=" + result.getAcceptedIsoLegs()
        + ", acceptedGraphNative=" + result.getAcceptedNonIsoLegs()
        + ", quotaInjectedAccepted=" + result.getAcceptedQuotaInjectedLegs()
        + ", poolHealth=" + (Double.isNaN(result.getIsoPoolHealthScore())
            ? "n/a" : String.format(Locale.US, "%.2f", result.getIsoPoolHealthScore()))
        + ", poolDemotedAtStep=" + result.getPoolDemotedAtStep());
      if (!result.isWithinTolerance()) {
        ops.logInfo("greedy: fallback — " + result.getFallbackReason());
      }
      ops.logInfo("greedy: planned " + result.getLoopWaypoints().size() + " waypoints"
        + ", estimated distance=" + result.getTotalDistanceMeters() + "m");

      // Route through the greedy ops.waypoints() with the standard routing engine.
      // The greedy planner's lookahead ensures ops.waypoints() are in well-connected
      // areas (not dead-end valleys), so doRoutingIntoRequest() produces gap-free tracks
      // following roads appropriate for the profile.
      ops.waypoints().clear();
      ops.waypoints().addAll(result.getLoopWaypoints());

      if (result.getMatchedWaypoints() != null) {
        ops.setMatchedWaypoints(result.getMatchedWaypoints());
      }

      if (result.getLegTracks() != null) {
        List<OsmTrack> legs = result.getLegTracks();
        request.greedyLegTracks = legs.toArray(new OsmTrack[0]);
        publishRuntimeHints();
      }

      // Phase 2 v3: the planner now retracks each committed leg, so its
      // merged track has full per-edge MessageData. Use that directly
      // instead of running doRoutingIntoRequest() which re-routes via a fragile
      // corridor mechanism that frequently fails or diverges. The
      // re-routing was wiping out the planner's hostility-aware
      // candidate choices, so the quality gate was seeing routes the
      // planner itself would have rejected. Diagnostic data: roughly
      // 80% of greedy legs in failing fastbike scenarios had the
      // corridor fail or diverge.
      boolean useDetailedPlannerTrack = result != null && result.getTrack() != null
        && result.getTrack().nodes != null && result.getTrack().nodes.size() >= MIN_ROUNDTRIP_LOOP_NODES;
      if (useDetailedPlannerTrack) {
        try {
          setTrack(result.getTrack());
          if (result.getMatchedWaypoints() != null) {
            ops.setMatchedWaypoints(result.getMatchedWaypoints());
          }
          cleanup.finalizeAdoptedRoundTripTrack(request.track, ops.matchedWaypoints());
        } catch (Exception e) {
          ops.logInfo("greedy: bypass path failed (" + e.getClass().getSimpleName() + ": " + e.getMessage() + "), falling back to doRouting");
          useDetailedPlannerTrack = false;
        }
      }
      if (!useDetailedPlannerTrack) {
        ops.routingContext().waypointCatchingRange = 250;
        request.searchRadius = searchRadius;
        publishRuntimeHints();
        // Honor the request deadline: once it has fully passed, do NOT start
        // the fallback re-route at all (doRouting resets ops.startTime(), so any
        // budget handed to it is a real overrun). While budget remains, fund
        // the fallback with the REMAINING budget, floored so a nearly-spent
        // request still gets a usable (bounded, < MIN_LADDER_RUNG_BUDGET_MS
        // overrun) salvage slice rather than a guaranteed instant timeout.
        long remaining = ops.remainingRequestBudgetMs();
        if (request.routingBudgetMs > 0 && remaining <= 0) {
          setError("round-trip request budget exhausted before the fallback re-route ("
            + remaining + "ms remaining)");
          ops.logInfo(request.error);
          setTrack(null);
          request.greedyLegTracks = null;
          publishRuntimeHints();
          return;
        }
        try {
          long fallbackBudget = request.routingBudgetMs <= 0
            ? request.routingBudgetMs
            : Math.min(request.routingBudgetMs,
                Math.max(MIN_LADDER_RUNG_BUDGET_MS, remaining));
          doRoutingIntoRequest(fallbackBudget);
        } catch (Exception e) {
          ops.logInfo("greedy: doRouting failed (" + e.getClass().getSimpleName() + ": " + e.getMessage() + ")");
          throw e;
        } finally {
          request.greedyLegTracks = null;
          publishRuntimeHints();
        }
      }
    } else {
      // ISO_GREEDY only fails over to plain GREEDY if it also failed; otherwise
      // ISO_GREEDY's planner already added graph-native per-step candidates
      // when the start-centered iso pool was insufficient (see buildCandidateProvider).
      // BALANCED skips this recursion (another full ladder): it adopts the
      // best-effort track below instead, and its caller falls back to the
      // cheap WAYPOINT placement when no track exists at all.
      if (algo == RoundTripAlgorithm.ISO_GREEDY
          && !request.effortPolicy.skipRetryLayers
          && ops.remainingRequestBudgetMs() >= MIN_LADDER_RUNG_BUDGET_MS) {
        ops.logInfo("ISO_GREEDY produced no loop, falling back to GREEDY with graph-native candidates");
        doGreedyRoundTrip(searchRadius, direction, RoundTripAlgorithm.GREEDY);
      } else if (algo == RoundTripAlgorithm.ISO_GREEDY && !request.effortPolicy.skipRetryLayers) {
        // Same recursion, but the request budget is spent — adopt/report what
        // we have instead of starting another multi-plan GREEDY ladder.
        ops.logInfo("ISO_GREEDY produced no loop and request budget is exhausted ("
          + ops.remainingRequestBudgetMs() + "ms left), skipping GREEDY fallback ladder");
        setError("greedy round trip planner produced no acceptable loop within the request budget"
          + (result == null || result.getFallbackReason() == null ? "" : ": " + result.getFallbackReason()));
        setRejectedTrack(result == null ? null : result.getTrack());
        setTrack(null);
      } else {
        // Adopt the planner's best-effort loop (if any) and hand it up to the
        // uniform quality gate in doRoundTrip, which is the single place that
        // decides hard-reject (STRUCTURAL, or any failure under strict mode) vs.
        // a lenient advisory. This keeps greedy consistent with the other
        // algorithms and removes a duplicate, tier-blind leniency decision: the
        // gate (plus the node/distance floor just above it) inspects the verdict
        // rather than re-deriving "usable" from node counts here.
        OsmTrack bestEffort = result == null ? null : result.getTrack();
        if (bestEffort != null && bestEffort.nodes != null && !bestEffort.nodes.isEmpty()) {
          ops.logInfo("greedy: adopting best-effort loop for the quality gate to grade ("
            + (result.getFallbackReason() == null ? "?" : result.getFallbackReason()) + ")");
          setTrack(bestEffort);
          if (result.getMatchedWaypoints() != null) {
            ops.setMatchedWaypoints(result.getMatchedWaypoints());
          }
          // finalize can throw (voice hints / speed profile / spur removal). Guard
          // it like the bypass path above: an exception here would otherwise
          // unwind past doRoundTrip's floor + quality gate (its catch does not
          // null request.track), shipping this un-gated best-effort track as a
          // success. On failure, reject instead so nothing skips the gate.
          try {
            cleanup.finalizeAdoptedRoundTripTrack(request.track, ops.matchedWaypoints());
            // request.error stays null: the floor check + quality gate in
            // doRoundTrip reject (and set request.error) if the loop is too small,
            // structurally broken, or strict mode is on; else it ships with a warning.
          } catch (Exception e) {
            setError("greedy best-effort finalize failed ("
              + e.getClass().getSimpleName() + ": " + e.getMessage() + ")");
            ops.logInfo(request.error);
            setRejectedTrack(bestEffort);
            setTrack(null);
          }
        } else {
          // Reached by plain GREEDY and by BALANCED's bounded ISO_GREEDY run
          // (which skips the GREEDY recursion) — keep the wording source-neutral.
          setError("greedy round trip planner produced no acceptable loop"
            + (result == null || result.getFallbackReason() == null ? "" : ": " + result.getFallbackReason()));
          ops.logInfo(request.error);
          setRejectedTrack(result == null ? null : result.getTrack());
          setTrack(null);
        }
      }
    }
  }

  /**
   * Bounded-effort dispatch: one bounded, graph-aware planning run with
   * predictable latency. Used by the BALANCED tier and by AUTO when the effort
   * policy resolves BOUNDED (constrained resources).
   *
   * <p>A single ISO_GREEDY dispatch (its internal graph-native compare stays
   * available) under a hard {@code min(request budget, tierBudget)} deadline and a
   * reduced routed top-K. The Phase 2.1 axis retry and the ISO_GREEDY→GREEDY
   * recursion are skipped ({@link RoundTripEffortPolicy#skipRetryLayers}); a
   * degraded-but-rideable loop is adopted best-effort for the lenient gate. When
   * the planner produces no track, or one the gate would hard-reject, the tier
   * falls back to a single FAST/WAYPOINT attempt under a fresh tier slice — always
   * returning some loop beats strict adherence to one slice. With
   * {@code greedyCapable == false} (allowSamewayback) only the budgeted fallback
   * runs. The caller passes through the shared floors and gate in
   * {@code doRoundTrip}; this method never returns an ungated success.
   */
  private void doBoundedRoundTrip(double searchRadius, double direction,
                                  RoundTripEffortPolicy policy, String tierLabel,
                                  boolean greedyCapable) {
    long tierBudgetMs = policy.tierBudgetMs;
    long t0 = System.currentTimeMillis();
    long savedDeadline = request.requestDeadline;
    long plannerMs = 0;
    if (!greedyCapable) {
      // Same constraint as the greedy dispatch: the planner generates its own
      // intermediate points and does not honor allowSamewayback. The waypoint
      // placement below still runs under the tier budget — bypassing the tier
      // would hand this input the full request budget.
      ops.logInfo(tierLabel + ": planner does not support allowSamewayback,"
        + " using waypoint placement under the tier budget");
    } else {
      RoundTripEffortPolicy savedPolicy = request.effortPolicy;
      long effectiveMs = tierSliceMs(tierBudgetMs, savedDeadline, t0);
      request.requestDeadline = t0 + effectiveMs;
      publishRuntimeHints();
      request.effortPolicy = policy;
      // The engine-level timers (island check, leg searches) run in THIS engine
      // and consult ops.maxRunningTime() — floor it to the slice too, or a nearly-
      // spent request budget times out the matching before the planner starts.
      // (The competition path achieves the same by flooring each child's doRun
      // budget.) 0 stays 0: an untimed request keeps engine timers off here;
      // the planner slice is still bounded by ops.roundTripRequestDeadline().
      long savedMaxRunningTime = ops.maxRunningTime();
      if (ops.maxRunningTime() > 0) {
        ops.setMaxRunningTime((t0 + effectiveMs) - ops.startTime());
      }
      try {
        doGreedyRoundTrip(searchRadius, direction, RoundTripAlgorithm.ISO_GREEDY);
      } finally {
        request.effortPolicy = savedPolicy;
        request.requestDeadline = savedDeadline;
        publishRuntimeHints();
        ops.setMaxRunningTime(savedMaxRunningTime);
      }
      plannerMs = System.currentTimeMillis() - t0;
      if (request.track != null) {
        // The bounded planner adopts degraded best-effort snapshots and defers
        // the verdict to the uniform gate in doRoundTrip. Take that verdict
        // now: a track the gate will hard-reject must not suppress the tier's
        // geometric fallback — by the time the shared gate nulls the track,
        // the chance to fall back is gone and the tier returns a hard error
        // instead of the loop it promises.
        // explicitViaMode == false by construction: the bounded tier is only
        // dispatched in generated-loop mode (the explicit-via skeleton
        // branches off before the tier dispatch).
        RoundTripQualityResult verdict = evaluateRoundTripGate(request.track, searchRadius, false);
        if (!verdict.isAccepted() && ops.roundTripQualityHardReject(verdict)) {
          ops.logInfo(tierLabel + ": bounded planner track fails the quality gate ("
            + verdict.getRejectionReason() + "); falling back to waypoint placement");
          setRejectedTrack(request.track);
          setTrack(null);
        } else {
          // The surviving track flows unchanged to the shared gate in
          // doRoundTrip — stash the verdict so that gate consumes it instead
          // of paying a second full-track evaluation (crossing grid, corridor
          // index) on every interactive bounded request. The fallback path
          // leaves this null: its track needs a fresh verdict.
          request.boundedGateVerdict = verdict;
        }
      }
      if (request.track == null) {
        ops.logInfo(tierLabel + ": bounded planner produced no accepted loop in " + plannerMs
          + "ms (budget " + tierBudgetMs + "ms)"
          + (request.error == null ? "" : " — " + request.error)
          + "; falling back to waypoint placement");
      }
    }
    if (request.track == null) {
      setError(null);
      // Fresh tier slice for the fallback (see method javadoc). Worst case is
      // two slices; the request-level watchdog still applies on top. Same
      // minimum-slice floor as above so a spent budget still funds the one
      // cheap geometric attempt.
      long fallbackStart = System.currentTimeMillis();
      long fallbackMs = tierSliceMs(tierBudgetMs, savedDeadline, fallbackStart);
      request.requestDeadline = fallbackStart + fallbackMs;
      publishRuntimeHints();
      long savedRoutingBudget = request.routingBudgetMs;
      long savedMaxRunningTime = ops.maxRunningTime();
      // Scope the engine timers to the fallback slice, UNCONDITIONALLY. The
      // placement phase (probing + the islanded-via guard) runs before
      // doRouting re-arms ops.startTime()/ops.maxRunningTime() from the routing budget:
      // under the request-scoped timer a spent budget makes every placement
      // engine call throw instantly — the island guard degrades to
      // keep-every-via and routing then dies on "target island detected" —
      // and an untimed request (all timer fields 0) would run the fallback
      // with no bound at all. Both violate the tier's slice contract.
      request.routingBudgetMs = fallbackMs;
      ops.setMaxRunningTime((fallbackStart + fallbackMs) - ops.startTime());
      try {
        doWaypointBasedRoundTrip(searchRadius, direction, RoundTripAlgorithm.WAYPOINT);
      } finally {
        request.requestDeadline = savedDeadline;
        publishRuntimeHints();
        request.routingBudgetMs = savedRoutingBudget;
        ops.setMaxRunningTime(savedMaxRunningTime);
      }
      if (request.track != null) {
        // The shipped track came from the waypoint fallback, not the planner —
        // keeping the FAILED planner result would attribute its counters and
        // pool-health telemetry to a loop the planner never produced.
        setPlannerResult(null);
      }
    }
    ops.logInfo(tierLabel + ": finished in " + (System.currentTimeMillis() - t0)
      + "ms (planner " + plannerMs + "ms, budget " + tierBudgetMs + "ms/slice, "
      + (request.track == null ? "no track" : "track " + request.track.distance + "m") + ")");
  }

  /**
   * AUTO candidate competition for generated round trips (no user vias). Runs
   * greedy candidates first, the legacy probe/WAYPOINT generator only as fallback:
   * <ol>
   *   <li>ISO_GREEDY — iso-derived candidates fed to the greedy planner.</li>
   *   <li>GREEDY — plain graph-native planner, if ISO_GREEDY fails or is weak.</li>
   *   <li>WAYPOINT/probe — only if greedy produced no accepted route.</li>
   * </ol>
   *
   * <p>Each candidate runs in an isolated child {@link RoutingEngine} built from a
   * request-fields-only copy of the parent {@link RoutingContext} (no parsed/runtime
   * state shared, output suppressed). The highest-scoring accepted candidate's
   * {@link OsmTrack} is adopted; its disclosures are surfaced. If none pass strict
   * validation, the lenient default adopts the least-bad best-effort track (see
   * {@link #selectBestEffortCandidate}); strict mode leaves the track null and sets
   * an error.
   */
  private void runAutoCandidateCompetition(double searchRadius, double direction) {
    long t0 = System.currentTimeMillis();
    // One wall-clock budget shared across the sequentially-run candidates, so
    // the competition cannot run ~Nx the requested timeout. Each child gets the
    // remaining slice (see runChildCandidate); once it is exhausted we stop
    // spawning further candidates.
    long deadline = t0 + (ops.maxRunningTime() > 0 ? ops.maxRunningTime() : DEFAULT_AUTO_BUDGET_MS);
    List<RoundTripCandidateResult> results = new ArrayList<>(3);

    // 1+2. Run ISO_GREEDY first, then plain GREEDY only when the ISO result
    // proves the comparison is still useful. This is the issue-#26 default:
    // avoid duplicate production algorithm runs when ISO_GREEDY is strong or
    // has already absorbed the graph-native provider fallback. An opt-in
    // speculative mode can still start GREEDY in parallel for deployments that
    // prefer lower single-request latency over duplicate CPU work.
    RoundTripCandidateResult[] parallel = new RoundTripCandidateResult[2];
    java.util.concurrent.atomic.AtomicReference<RoutingEngine> greedyEngineOut =
      new java.util.concurrent.atomic.AtomicReference<>();
    Thread greedyThread = null;
    // Optional load-aware parallelism: routing is CPU-bound, so speculative
    // GREEDY is opt-in and also gated on a NON-BLOCKING permit. If the permit
    // is unavailable, or speculation is disabled, GREEDY runs sequentially only
    // if the ISO result needs it.
    boolean parallelPermit = SPECULATIVE_AUTO_GREEDY
      && System.currentTimeMillis() < deadline
      && PARALLEL_AUTO_SEMAPHORE.tryAcquire();
    if (parallelPermit) {
      greedyThread = new Thread(() -> {
        try {
          parallel[1] = runChildCandidate(RoundTripAlgorithm.GREEDY, searchRadius, direction,
            deadline, greedyEngineOut);
        } finally {
          PARALLEL_AUTO_SEMAPHORE.release();
        }
      }, "roundtrip-auto-greedy");
      // Daemon: a discarded speculative child must never delay JVM exit (CLI).
      greedyThread.setDaemon(true);
      greedyThread.start();
    }
    parallel[0] = runChildCandidate(RoundTripAlgorithm.ISO_GREEDY, searchRadius, direction, deadline);
    RoundTripCandidateResult isoGreedyR = parallel[0] != null
      ? parallel[0] : new RoundTripCandidateResult(RoundTripAlgorithm.ISO_GREEDY);
    // Whether GREEDY will be consulted is fully decidable BEFORE the join:
    // the spec calls for GREEDY when iso pool is not viable OR ISO_GREEDY is
    // weak (same single threshold for both signals), and the sequential
    // competition decided whether to START GREEDY right after ISO_GREEDY
    // completed — recording the entitlement instant here keeps the budget
    // accounting identical (a tiny budget still runs/counts exactly one
    // candidate). Deciding now means a STRONG ISO_GREEDY never waits out the
    // speculative child: it is terminated instead, so AUTO latency on the
    // good path stays that of ISO_GREEDY alone.
    long greedyDecisionTime = System.currentTimeMillis();
    // MAX effort (QUALITY tier): the plain-GREEDY competitor always runs — the
    // caller asked for the best loop and accepts the cost; the health-gated
    // skip is a latency optimization the tier explicitly opts out of. Still
    // bounded by the shared deadline.
    boolean greedyNeeded = request.effortPolicy.runGreedyAlways
      && System.currentTimeMillis() < deadline
      || autoNeedsPlainGreedy(isoGreedyR, greedyDecisionTime, deadline);
    String greedyDiscardReason = autoPlainGreedyDiscardReason(isoGreedyR, greedyDecisionTime, deadline);
    boolean greedyResultIgnored = false;
    if (greedyThread != null) {
      RoutingEngine greedyChild = greedyEngineOut.get();
      if (!greedyNeeded && greedyChild != null) {
        // The speculative child's result will not be consulted — kill it so
        // the bounded join below returns promptly (the volatile flag aborts
        // its searches/expansions within ~one heap pop).
        greedyChild.terminate();
      }
      // ALWAYS bound the join. Even a needed child must not hang the request
      // thread: its own budget ends at the shared deadline, so wait only up to
      // the remaining budget plus an unwind margin. If it overstays that
      // (overshot its budget, or wedged in a path slow to honor termination),
      // terminate it and give it a final short window — never block forever.
      // A discarded child gets only the unwind margin.
      long joinBudgetMs = greedyNeeded
        ? Math.max(0L, deadline - System.currentTimeMillis()) + AUTO_CHILD_JOIN_UNWIND_MS
        : AUTO_CHILD_JOIN_UNWIND_MS;
      try {
        greedyThread.join(joinBudgetMs);
        if (greedyThread.isAlive()) {
          ops.logInfo("AUTO: GREEDY child overstayed its budget; terminating");
          if (greedyChild != null) {
            greedyChild.terminate();
          }
          greedyThread.join(AUTO_CHILD_JOIN_UNWIND_MS);
        }
      } catch (InterruptedException ie) {
        Thread.currentThread().interrupt();
      }
      // If the (needed) child is STILL alive its result slot is not safely
      // published — treat it as no candidate rather than reading a
      // half-written result. The daemon thread cannot block JVM exit.
      if (greedyThread.isAlive()) {
        greedyNeeded = false;
        greedyResultIgnored = true;
        ops.logInfo("AUTO: GREEDY child did not stop in time; ignoring its result");
      }
    }
    results.add(isoGreedyR);
    ops.logInfo("AUTO candidate: " + isoGreedyR);

    // Sequential fallback: no spare-CPU permit was available (busy box) or the
    // budget was already spent at spawn time, so GREEDY was not started in
    // parallel. Run it now on this thread iff it is actually needed — exactly
    // the pre-parallel competition's behaviour (GREEDY only when ISO_GREEDY is
    // weak). No oversubscription: this reuses the request's own core.
    if (greedyThread == null && greedyNeeded) {
      parallel[1] = runChildCandidate(RoundTripAlgorithm.GREEDY, searchRadius, direction, deadline);
    }

    if (greedyNeeded && parallel[1] != null) {
      results.add(parallel[1]);
      ops.logInfo("AUTO candidate: " + parallel[1]);
    } else if (greedyThread != null && !greedyResultIgnored) {
      ops.logInfo("AUTO: speculative GREEDY child discarded ("
        + (greedyDiscardReason == null ? "not needed" : greedyDiscardReason)
        + ") — policy parity with the sequential competition");
    }

    // 3. Compare accepted greedy candidates; pick highest score.
    RoundTripCandidateResult winner = null;
    for (RoundTripCandidateResult r : results) {
      if (!r.accepted()) continue;
      if (winner == null || r.scoreValue() > winner.scoreValue()) {
        winner = r;
      }
    }

    // 4. Legacy fallback only if both greedy variants failed hard validation
    //    and budget remains.
    if (winner == null && System.currentTimeMillis() < deadline) {
      RoundTripCandidateResult waypointR = runChildCandidate(
        RoundTripAlgorithm.WAYPOINT, searchRadius, direction, deadline);
      results.add(waypointR);
      ops.logInfo("AUTO candidate: " + waypointR);
      if (waypointR.accepted()) {
        winner = waypointR;
      }
    }

    // 5. Last-resort ISOCHRONE fallback. The direct isochrone-frontier
    //    placement reaches loops the greedy radial candidates miss in
    //    constrained terrain (e.g. a valley where the radial probe can't
    //    form a loop in the requested direction, or only finds a chaotic
    //    one). Purely additive: only runs when ISO_GREEDY, GREEDY and
    //    WAYPOINT have all already failed, so it cannot displace a winner.
    if (winner == null && System.currentTimeMillis() < deadline) {
      RoundTripCandidateResult isochroneR = runChildCandidate(
        RoundTripAlgorithm.ISOCHRONE, searchRadius, direction, deadline);
      results.add(isochroneR);
      ops.logInfo("AUTO candidate: " + isochroneR);
      if (isochroneR.accepted()) {
        winner = isochroneR;
      }
    }
    long totalMs = System.currentTimeMillis() - t0;

    // Lenient default: if no candidate passed strict validation but one produced
    // a rideable route that failed only a QUALITY check, adopt the best-effort
    // one (the child already attached its "Warning:" advisory) instead of
    // returning nothing — keeping AUTO consistent with direct-dispatch leniency.
    // Candidates are in algorithm-quality order (ISO_GREEDY, GREEDY, WAYPOINT,
    // ISOCHRONE), so the first quality-failed track is the best best-effort.
    // The lenient/strict decision uses the same predicate as the gate path
    // (roundTripQualityHardReject), so strict mode keeps the hard "no acceptable
    // route" and only QUALITY verdicts are adopted leniently.
    if (winner == null) {
      // Among the QUALITY-tier best-effort candidates (STRUCTURAL and, under strict
      // mode, every failure are excluded by roundTripQualityHardReject), pick the
      // LEAST-BAD overall rather than the first by algorithm order. We rank with the
      // same multi-factor RouteChoiceScore used for accepted winners — distance
      // closeness (its largest weight), profile cost/m match, and reuse/shape — so
      // each candidate is penalised on the very axis it failed and the most rideable
      // degraded loop wins. No extra routing: the tracks are already generated.
      List<RoundTripCandidateResult> bestEffort = new ArrayList<>();
      for (RoundTripCandidateResult r : results) {
        if (r.track != null && r.gateVerdict != null
            && !ops.roundTripQualityHardReject(r.gateVerdict)) {
          bestEffort.add(r);
        }
      }
      winner = selectBestEffortCandidate(bestEffort, 2 * Math.PI * searchRadius,
        ops.routingContext().getProfileName(), direction);
      if (winner != null) {
        ops.logInfo("AUTO: no strictly-accepted route; adopting best-effort " + winner.algorithm
          + " (most rideable of " + bestEffort.size()
          + " degraded candidate(s)) with quality warning (lenient mode)");
      }
    }

    if (winner == null) {
      // All candidates failed. Surface the most recent (richest) error.
      String err = null;
      for (int i = results.size() - 1; i >= 0; i--) {
        if (results.get(i).errorMessage != null) { err = results.get(i).errorMessage; break; }
      }
      setError("AUTO competition produced no acceptable route "
        + "(tried " + results.size() + " candidates in " + totalMs + "ms): "
        + (err == null ? "unknown" : err));
      ops.logInfo(request.error);
      // Surface the best-geometry rejected candidate for post-mortem inspection,
      // mirroring the direct-dispatch path which sets request.lastRejectedTrack before
      // nulling request.track. Candidates are in algorithm-quality order, so the
      // first with a track is the best available rejected geometry.
      for (RoundTripCandidateResult r : results) {
        if (r.track != null) {
          setRejectedTrack(r.track);
          break;
        }
      }
      setTrack(null);
      return;
    }
    adoptCandidateWinner(winner, results, totalMs);
  }

  /**
   * Run one AUTO candidate in an isolated child engine, score it, return the
   * wrapper. Never throws — failures land in the result's {@code errorMessage}.
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline) {
    return runChildCandidate(algo, searchRadius, direction, deadline, null);
  }

  /**
   * Adopt the winning candidate's track as this engine's result and attach a
   * summary diagnostic of what was tried and which won.
   */
  private void adoptCandidateWinner(RoundTripCandidateResult winner,
                                    List<RoundTripCandidateResult> all, long totalMs) {
    setTrack(winner.track);
    setError(null);
    cleanup.finalizeAdoptedRoundTripTrack(request.track, request.track == null ? null : request.track.getMatchedWaypoints());
    // Best-effort (quality-failed) winner adopted under lenient mode: make sure
    // the user-facing quality Warning is present. The child engine usually
    // attaches it, but when the parent's gate re-evaluation in runChildCandidate
    // disagrees with the child's own verdict the child may not have — so attach
    // it here if absent, mirroring the direct-dispatch advisory (and skip when a
    // "Warning:" is already present to avoid a duplicate).
    if (request.track != null && !winner.accepted() && winner.gateVerdict != null
        && (request.track.message == null || !request.track.message.contains("Warning:"))) {
      appendRouteMessage(request.track, "Warning: " + winner.gateVerdict.getRejectionReason()
        + " (shape=" + winner.gateVerdict.getShape() + ") — route returned anyway; ride at your"
        + " discretion, or set roundTripStrictQuality=1 to reject it.");
    }
    // Append a summary message so debugging consumers can see the
    // competition outcome. Score breakdown is in the route-choice verdict.
    StringBuilder summary = new StringBuilder(256);
    summary.append("AUTO selected ").append(winner.algorithm)
      .append(" (score ").append(String.format(Locale.US, "%.3f", winner.scoreValue()))
      .append(") after ").append(all.size()).append(" candidate(s) in ").append(totalMs).append("ms.");
    for (RoundTripCandidateResult r : all) {
      if (r == winner) continue;
      summary.append(" Also tried ").append(r.algorithm).append(": ")
        .append(r.accepted() ? String.format(Locale.US, "score %.3f", r.scoreValue())
                             : (r.errorMessage == null ? "no track" : "rejected"))
        .append('.');
    }
    if (request.track != null) {
      // request.track is nullable here (a best-effort winner can carry no track —
      // see the null-guards above at adoption and the warning block); only
      // attach the AUTO summary when there is a track to annotate.
      if (request.track.message == null || request.track.message.isEmpty()) {
        request.track.message = summary.toString();
      } else {
        request.track.message += " " + summary.toString();
      }
    }
    // Keep messageList.get(0) in sync with the just-extended message so the
    // GPX <brouter:info> / comment block reflects the AUTO summary too.
    cleanup.ensureInfoMessage(request.track);
    ops.logInfo(summary.toString());
    if (winner.score != null) {
      ops.logInfo("AUTO winner score breakdown:\n" + winner.score.describe());
    }
    // Format + persist the adopted track if the caller asked for an
    // output file. The child engines ran with null outfileBase (output
    // suppressed); the parent does the single final write.
    ops.writeAdoptedTrackOutput(request.track);
  }

  private RouteChoiceScore.Verdict scoreInternalGreedyResult(RoundTripResult result,
                                                            double desiredDistance,
                                                            double direction) {
    return scoreInternalGreedyResult(result, desiredDistance,
      ops.routingContext().getProfileName(), direction,
      ops.routingContext().allowSamewayback, ops.roundTripFerriesAllowed());
  }

  /**
   * Rank degraded best-effort candidates, return the most rideable (or {@code null}
   * if none have a track). Uses {@link RouteChoiceScore#scoreBestEffort}, which
   * bypasses the scorer's accepted-only zero-guard (a rejected track is ranked on
   * real geometry, not collapsed to 0) but still applies the gate verdict's shape
   * penalty, so a rejected LOLLIPOP/OUT_AND_BACK cannot outrank a strict loop.
   * Ties keep {@code candidates} order (AUTO algorithm-quality order). Does no routing.
   */
  static RoundTripCandidateResult selectBestEffortCandidate(
      List<RoundTripCandidateResult> candidates, double expectedDistance,
      String profileName, double direction) {
    RoundTripCandidateResult best = null;
    double bestScore = -1.0;
    RouteChoiceScore.Verdict bestVerdict = null;
    for (RoundTripCandidateResult r : candidates) {
      if (r.track == null) {
        continue;
      }
      RouteChoiceScore.Verdict v = RouteChoiceScore.scoreBestEffort(
        r.track, expectedDistance, profileName, r.gateVerdict, direction);
      double s = v.score();
      if (s > bestScore) {
        bestScore = s;
        best = r;
        bestVerdict = v;
      }
    }
    // Surface the computed best-effort score on the winner so the adoption
    // summary logs the real value (and the score breakdown) instead of 0.000;
    // r.score is otherwise only set for strictly-accepted candidates.
    if (best != null && best.score == null) {
      best.score = bestVerdict;
    }
    return best;
  }

  /**
   * Run one greedy planning attempt — the inner sub-route-count loop for a single
   * {@code tryDirection}. Stamps iso-asymmetry telemetry on the result and updates
   * the last-round-trip-result on every iteration so cross-attempt comparison sees
   * consistent metadata. The returned {@link RoundTripResult} may be degraded —
   * the caller decides whether to accept or retry.
   */
  private RoundTripResult runGreedyAttempt(OsmNodeNamed start, double searchRadius,
                                           double desiredDistance, double tryDirection,
                                           int baseSubRouteCount,
                                           RoundTripCandidateProvider provider,
                                           IsoAsymmetryBias bias,
                                           ReturnDistanceOracle returnOracle,
                                           IsoPoolHealth.PoolShape poolShape,
                                           IsoStartPolicy subRoutePolicy) {
    RoundTripResult result = null;
    boolean firstRung = true;
    for (int subRouteCount : greedySubRouteCountPlan(baseSubRouteCount, subRoutePolicy)) {
      // Request-budget gate on the retry ladder: each plan() used to get a
      // fresh 30s deadline regardless of remaining request budget, so the
      // ladder alone could run ~4x the requested timeout. Stop starting new
      // rungs once the request budget cannot fund a useful plan anymore.
      // The FIRST rung is exempt: minimum-slice floors (the bounded tier, the
      // competition's MIN_CHILD) deliberately fund exactly one run, and that
      // floor equals this gate's threshold — checking remaining-vs-threshold
      // a millisecond into the slice would veto the very run the floor
      // funded. The planner still honors its external deadline internally.
      long remaining = ops.remainingRequestBudgetMs();
      if (!firstRung && remaining < MIN_LADDER_RUNG_BUDGET_MS) {
        ops.logInfo("greedy: request budget exhausted (" + remaining
          + "ms left), skipping remaining subRouteCount ladder");
        break;
      }
      firstRung = false;
      ops.logInfo("greedy round trip: subRouteCount=" + subRouteCount + ", direction=" + (int) tryDirection);
      GreedyRoundTripPlanner planner = new GreedyRoundTripPlanner(ops, provider,
        new CandidateScorer(), subRouteCount, 0.05, 8);
      planner.setHostilityActive(RoundTripQualityGate.isPavedProfile(ops.routingContext().getProfileName()));
      planner.setProfileName(ops.routingContext().getProfileName());
      planner.setVarietySeed(ops.routingContext().getRoundTripSeed());
      planner.setRouteBudgets(request.effortPolicy.topKNormal, request.effortPolicy.topKLate);
      planner.setPlanBudgetScale(request.effortPolicy.planBudgetScale);
      planner.setReturnOracle(returnOracle);
      // Fresh per-plan health tracker: dynamic evidence must not leak across
      // ladder rungs (a demotion earned at subRouteCount=5 says nothing about
      // the 4-step plan's pool usage).
      planner.setPoolHealth(poolShape == null ? null : new IsoPoolHealth(poolShape));
      planner.setExternalDeadline(request.requestDeadline == 0
        ? Long.MAX_VALUE : request.requestDeadline);
      result = planner.plan(start, desiredDistance, tryDirection);
      if (result != null) {
        result.setIsoAsymmetryBearingApplied(bias.applied);
        result.setIsoAsymmetryBearingDegrees(bias.bearingDegrees);
        result.setIsoAsymmetryBestBucketIndirectness(bias.indirectness);
        result.setIsoAsymmetryBestBucketHits(bias.hits);
        result.setIsoAsymmetryBestBucketAirDistMeters(bias.airDistMeters);
      }
      setPlannerResult(result);
      if (!isDegradedGreedyResult(result)
          && result != null && result.getLoopWaypoints() != null
          && result.getLoopWaypoints().size() >= 4) {
        return result;
      }
      ops.logInfo("greedy: attempt with " + subRouteCount + " sub-routes did not produce an acceptable loop"
        + (result == null || result.getFallbackReason() == null ? "" : " (" + result.getFallbackReason() + ")"));
    }
    return result;
  }

  /**
   * Candidate provider for the mode: GREEDY uses per-step graph-native candidates;
   * ISO_GREEDY blends a bounded start-centered isochrone pool with that same
   * provider. Geometric radial placement is intentionally unused here.
   */
  private RoundTripCandidateProvider buildCandidateProvider(RoundTripAlgorithm algo,
                                                            OsmNodeNamed start,
                                                            double searchRadius,
                                                            double startDirection,
                                                            IsochroneExpansionResult iso,
                                                            GraphNativeCandidateProvider graphNative) {
    if (algo != RoundTripAlgorithm.ISO_GREEDY) {
      return graphNative;
    }
    if (iso == null || iso.frontier.length < 6 || iso.candidates.size() < 12) {
      ops.logInfo("ISO_GREEDY: insufficient isochrone data ("
        + (iso == null ? 0 : iso.frontier.length) + " buckets, "
        + (iso == null ? 0 : iso.candidates.size()) + " raw candidates), using graph-native candidates");
      return graphNative;
    }
    IsochroneCandidateProvider isoProvider =
      IsochroneCandidateProvider.fromPool(searchRadius, startDirection, iso.candidates);
    if (isoProvider.poolSize() < 6) {
      ops.logInfo("ISO_GREEDY: candidate pool too small after filtering ("
        + isoProvider.poolSize() + "), using graph-native candidates");
      return graphNative;
    }
    if (!isoProvider.isDiverse()) {
      ops.logInfo("ISO_GREEDY: candidate pool concentrated in a narrow corridor ("
        + isoProvider.poolSize() + " candidates), using graph-native candidates");
      return graphNative;
    }
    // ISO_GREEDY: blend start-centered iso depth with per-step graph-native
    // candidates. Both sources are road-native; neither invents coordinates.
    ops.logInfo("ISO_GREEDY: blended isochrone+graph-native provider (iso pool="
      + isoProvider.poolSize() + ")");
    return new BlendedCandidateProvider(isoProvider, graphNative);
  }

  public static RoundTripAlgorithm selectRoundTripAlgorithm(double searchRadius) {
    // Cheap fallback selector. The full AUTO policy lives in
    // {@link #runAutoCandidateCompetition}; this helper remains as a stable
    // entry point for direct callers and unsupported AUTO modes.
    return RoundTripAlgorithm.GREEDY;
  }

  public static int selectGreedySubRouteCount(double desiredDistance, String profileName) {
    int n;
    if (desiredDistance < 8000) {
      n = 3;
    } else if (desiredDistance < 30000) {
      n = 4;
    } else if (desiredDistance < 80000) {
      n = 5;
    } else {
      n = 6;
    }
    if (profileName != null && profileName.toLowerCase(Locale.US).contains("mtb")) {
      n++;
    }
    return Math.max(3, Math.min(6, n));
  }

  public static int[] greedySubRouteCountPlan(int base) {
    return greedySubRouteCountPlan(base, IsoStartPolicy.BLEND);
  }

  public static int[] greedySubRouteCountPlan(int base, IsoStartPolicy policy) {
    int clamped = Math.max(3, Math.min(6, base));
    List<Integer> counts = new ArrayList<>(6);
    if (policy == IsoStartPolicy.GRAPH_NATIVE_ONLY) {
      addUniqueCount(counts, clamped - 1);
      addUniqueCount(counts, clamped);
      addUniqueCount(counts, clamped - 2);
      addUniqueCount(counts, clamped + 1);
      addUniqueCount(counts, clamped + 2);
      addUniqueCount(counts, clamped - 3);
    } else {
      addUniqueCount(counts, clamped);
      addUniqueCount(counts, clamped + 1);
      addUniqueCount(counts, clamped - 1);
      addUniqueCount(counts, clamped - 2);
      addUniqueCount(counts, clamped + 2);
      addUniqueCount(counts, clamped - 3);
    }
    int[] result = new int[counts.size()];
    for (int i = 0; i < counts.size(); i++) result[i] = counts.get(i);
    return result;
  }

  public static IsoStartPolicy selectIsoStartPolicy(IsoPoolHealth.PoolShape poolShape) {
    if (poolShape == null) {
      return IsoStartPolicy.GRAPH_NATIVE_ONLY;
    }
    IsoPoolHealth staticHealth = new IsoPoolHealth(poolShape);
    // Only UNHEALTHY escalates to a graph-native-only start. A statically
    // DEGRADED pool (the weakest admitted shape sits exactly at 0.50) keeps
    // the blend: the planner's influence reduction — stripped prior terms
    // plus an extra routed quota seat — is the calibrated response, and it
    // engages from step 1 because the static deduction is already in the
    // score. UNHEALTHY stays reachable only through in-plan evidence with
    // the current weights (static floor 0.50); unadmitted pools take the
    // poolShape == null arm above.
    if (staticHealth.state() == IsoPoolHealth.State.UNHEALTHY) {
      return IsoStartPolicy.GRAPH_NATIVE_ONLY;
    }
    // No third value for the perpendicular-strong-axis situation: the former
    // DUAL_IF_WEAK was behaviorally identical to BLEND (no consumer ever
    // distinguished them), and the Phase 2.1 axis retry derives its own
    // trigger conditions from the frontier axis directly.
    return IsoStartPolicy.BLEND;
  }

  public enum IsoStartPolicy {
    BLEND,
    GRAPH_NATIVE_ONLY
  }

  /**
   * Whether greedy planning applies: it generates its own intermediate waypoints,
   * so user vias and allowSamewayback are not honored.
   */
  public static boolean greedySupports(boolean allowSamewayback, int waypointCount) {
    return !allowSamewayback && waypointCount <= 1;
  }

  /**
   * One bounded tier slice: the tier budget clamped to the remaining request
   * budget, floored at {@link #MIN_LADDER_RUNG_BUDGET_MS} so a nearly-spent
   * request still funds ONE run (deliberate bounded overrun, not a guaranteed
   * instant timeout). An untimed request (deadline 0) gets the full tier budget.
   */
  private static long tierSliceMs(long tierBudgetMs, long requestDeadline, long now) {
    return Math.min(tierBudgetMs, requestDeadline == 0 ? tierBudgetMs
      : Math.max(requestDeadline - now, MIN_LADDER_RUNG_BUDGET_MS));
  }

  // --- Placement-path instrumentation (diagnostic only) -------------------
  // Monotonic process-wide counters recording which waypoint-placement path
  // each round-trip leg used. Purely additive: NO routing logic reads these.
  // They exist to measure how often the terrain-unaware ENVELOPE path is taken
  // (esp. ENVELOPE_ISO_FALLBACK, the only envelope case where an indirectness
  // compensation could be derived) so the P5 envelope-compensation work can be
  // prioritised and validated against the loop-quality corpus. AUTO runs its
  // candidates in `quite` child engines whose logInfo is suppressed, so a
  // static counter — not per-call logging — is what survives a corpus run.
  // Aggregate with placementPathCounts(); reset between corpus cases with
  // resetPlacementPathCounts().
  enum PlacementPath { ISOCHRONE, ENVELOPE_ISO_FALLBACK, ENVELOPE_FAST, CIRCLE }

  private void recordPlacementPath(PlacementPath path) {
    PLACEMENT_PATH_COUNTS.incrementAndGet(path.ordinal());
    ops.logInfo("roundtrip placement path: " + path); // no-op for quite child engines
  }

  /** Snapshot of placement-path counts, indexed by {@link PlacementPath#ordinal()}. */
  public static long[] placementPathCounts() {
    long[] out = new long[PLACEMENT_PATH_COUNTS.length()];
    for (int i = 0; i < out.length; i++) out[i] = PLACEMENT_PATH_COUNTS.get(i);
    return out;
  }

  /** Reset the placement-path counters (for test/corpus isolation). */
  public static void resetPlacementPathCounts() {
    for (int i = 0; i < PLACEMENT_PATH_COUNTS.length(); i++) PLACEMENT_PATH_COUNTS.set(i, 0L);
  }

  /**
   * Minimum remaining request budget worth starting another subRouteCount rung,
   * Phase-2.1 retry, or ISO_GREEDY→GREEDY recursion — below this a fresh plan()
   * could not route even a couple of legs, so leave the time to the fallback.
   */
  private static final long MIN_LADDER_RUNG_BUDGET_MS = 3_000;

  /**
   * As above, but publishes the child engine into {@code engineOut} as soon as it
   * is constructed, so a concurrent coordinator can {@code terminate()} a
   * speculative child no longer needed (the kill flag is honoured per search pop
   * and per expansion pop).
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline,
                                                     java.util.concurrent.atomic.AtomicReference<RoutingEngine> engineOut) {
    long t0 = System.currentTimeMillis();
    RoundTripCandidateResult r = new RoundTripCandidateResult(algo);
    try {
      RoutingContext childCtx = ops.routingContext().copyRequestFields();
      childCtx.roundTripAlgorithm = algo;
      childCtx.startDirection = (int) direction;
      // Inherit the user's direction intent from copyRequestFields rather than
      // hard-forcing it. forceUseStartDirection makes the first leg leave on a
      // strict bearing; when the user supplied only a soft `direction` (or
      // none) that over-constrains the loop and can shove the opening leg onto
      // a profile-hostile stretch, failing a candidate that the same algorithm
      // accepts when free to pick a nearby bearing. Only an explicit `heading`
      // (which sets forceUseStartDirection on the parent) hard-forces here.
      // Copy waypoint list — child engine mutates its own list.
      List<OsmNodeNamed> childWps = new ArrayList<>(ops.waypoints().size());
      for (OsmNodeNamed wp : ops.waypoints()) {
        OsmNodeNamed copy = new OsmNodeNamed(new OsmNode(wp.ilon, wp.ilat));
        copy.name = wp.name;
        childWps.add(copy);
      }
      // Output suppressed (null outfileBase). Child runs its own pipeline
      // including post-routing checks + quality gate; we just inspect the
      // result.
      RoutingEngine child = new RoutingEngine(null, null, ops.segmentDir(), childWps, childCtx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      child.quite = true;
      // The child plans with the parent's resolved effort (QUALITY's raised
      // top-K / plan budget must reach the planners it spawns).
      child.roundTripOps().setRoundTripEffortPolicy(request.effortPolicy);
      if (engineOut != null) {
        engineOut.set(child);
      }
      // Give the child only the remaining shared budget (floored so a spawned
      // candidate still gets a usable slice), not the full request timeout.
      long budget = childCandidateBudgetMs(deadline, System.currentTimeMillis());
      child.doRun(budget);
      r.track = child.getFoundTrack();
      r.errorMessage = child.getErrorMessage();
      r.runtimeMillis = System.currentTimeMillis() - t0;
      // Aggregate the child's expansion work into the parent so
      // getLinksProcessed() reports request-level totals (the perf budget
      // suite's work metric). Same-thread for sequential children; the
      // speculative parallel child is joined before its result is read.
      ops.addLinksProcessed(child.getLinksProcessed());
      // All winner-attribution telemetry (incl. the keep-when-forced marker
      // the re-gate below honors) reads through this reference — no
      // field-by-field copy to forget when RoundTripResult grows.
      r.planner = child.getLastRoundTripResult();

      if (r.track != null) {
        // Score against the parent's expected loop distance. This produces
        // a verdict that may differ from the child's internal gate result
        // because the parent's ops.routingContext() is the source of truth (e.g.
        // for profile-name lookup), but in practice both agree.
        double expectedDist = 2 * Math.PI * searchRadius;
        String profileName = ops.routingContext().getProfileName();
        r.gateVerdict = evaluateRoundTripGate(r.track, searchRadius, false,
          r.forcedCorridorAccepted());
        if (r.gateVerdict.isAccepted()) {
          r.score = RouteChoiceScore.score(r.track, expectedDist,
            profileName, r.gateVerdict, direction);
        }
      }
    } catch (RuntimeException e) {
      // Preserve the exception type: e.getMessage() is null for NPE/AIOOBE/CCE,
      // which otherwise surfaces an undiagnosable "threw: null" to the operator.
      // Also log the full stack trace on the parent (which, unlike the child, is
      // not `quite`) so a recurring child failure is diagnosable from logs — the
      // child suppressed its own logging via quite=true + null outfileBase.
      ops.logThrowable(e);
      r.errorMessage = "candidate " + algo + " threw: " + e.getClass().getSimpleName()
        + (e.getMessage() == null ? "" : ": " + e.getMessage());
      r.runtimeMillis = System.currentTimeMillis() - t0;
    }
    return r;
  }

  /**
   * Clear-accept threshold: below this, AUTO normally runs the plain GREEDY
   * candidate as a comparison before the legacy WAYPOINT fallback. ISO_GREEDY's
   * own internal graph-native fallback (see {@link IsoPoolHealth}) makes that
   * comparison win less over time; it is retained until winner-attribution
   * evidence proves it unneeded — check the ISO_GREEDY candidate's
   * {@code quotaAccepted}/{@code poolHealth}/{@code demotedAtStep} suffix
   * ({@link RoundTripCandidateResult#toString}) before removing it.
   */
  private static final double CLEAR_ACCEPT_THRESHOLD = 0.85;

  private static final long MIN_CHILD_BUDGET_MS = 5_000;

  /** Overload for verdicts on a CANDIDATE's track, whose forced-corridor
   *  marker lives on the candidate rather than the engine field. */
  private RoundTripQualityResult evaluateRoundTripGate(OsmTrack track, double searchRadius,
                                                       boolean explicitViaMode,
                                                       boolean forcedCorridorAccepted) {
    boolean allowSamewayback = ops.routingContext().allowSamewayback || forcedCorridorAccepted;
    return RoundTripQualityGate.evaluate(track, 2 * Math.PI * searchRadius,
      ops.routingContext().getProfileName(), allowSamewayback, explicitViaMode,
      ops.roundTripFerriesAllowed());
  }

  private static boolean isoGreedyAbsorbedGraphNativeTruth(RoundTripCandidateResult isoGreedyR) {
    // The child's explicit start-policy decision: a graph-native-only plan
    // already used the same candidate source as plain GREEDY, so a separate
    // GREEDY child would duplicate it. (This used to be inferred from three
    // telemetry sentinels — no iso legs + some non-iso legs + NaN health —
    // which any telemetry-semantics change could silently flip.)
    return isoGreedyR.algorithm == RoundTripAlgorithm.ISO_GREEDY
      && isoGreedyR.graphNativeOnlyStart();
  }

  static RouteChoiceScore.Verdict scoreInternalGreedyResult(RoundTripResult result,
                                                            double desiredDistance,
                                                            String profileName,
                                                            double direction,
                                                            boolean allowSamewayback,
                                                            boolean allowFerries) {
    if (result == null || isDegradedGreedyResult(result)
        || result.getTrack() == null
        || result.getLoopWaypoints() == null
        || result.getLoopWaypoints().size() < 4) {
      return null;
    }
    RoundTripQualityResult gate = RoundTripQualityGate.evaluate(result.getTrack(),
      desiredDistance, profileName,
      allowSamewayback || result.isForcedCorridorAccepted(),
      false, allowFerries);
    if (gate == null || !gate.isAccepted()) {
      return null;
    }
    return RouteChoiceScore.score(result.getTrack(), desiredDistance,
      profileName, gate, direction);
  }
}
