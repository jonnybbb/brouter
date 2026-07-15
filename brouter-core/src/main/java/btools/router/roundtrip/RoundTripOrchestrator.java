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

  final RoundTripEngineOps ops;

  /** The active request's mutable state; recreated at each doRoundTrip entry. */
  RoundTripRequest request = new RoundTripRequest();

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
  private final RoundTripStrategy fastStrategy;

  /** Greedy plan-and-route tier (GREEDY / ISO_GREEDY). */
  private final RoundTripStrategy greedyStrategy;

  /** Bounded tier: budget-sliced greedy attempt with a waypoint fallback. */
  private final RoundTripStrategy boundedStrategy;

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
  void publishRuntimeHints() {
    ops.setRoundTripRuntimeHints(new RoundTripRuntimeHints(
      request.searchRadius, request.requestDeadline, request.explicitVia, request.greedyLegTracks));
  }

  /** Record the gate-rejected track on the active request (post-mortem surface). */
  void setRejectedTrack(OsmTrack track) {
    request.lastRejectedTrack = track;
  }

  /** Set the request's working result track (published to the engine at request end). */
  void setTrack(OsmTrack track) {
    request.track = track;
  }

  /** Set the request's working error message (published to the engine at request end). */
  void setError(String error) {
    request.error = error;
  }

  /**
   * Run the engine routing pipeline and capture its outcome on the request.
   * The engine's result fields are seeded from the request first, so the run
   * sees exactly the state the individual field writes used to leave behind.
   */
  void doRoutingIntoRequest(long budgetMs) {
    ops.setFoundTrack(request.track);
    ops.setErrorMessage(request.error);
    RoutingOutcome outcome = ops.doRouting(budgetMs);
    request.track = outcome.track;
    request.error = outcome.error;
  }

  /** Record the planner-result telemetry on the active request. */
  void setPlannerResult(RoundTripResult result) {
    request.lastResult = result;
  }
  final WaypointSnapper snapper;
  final GeometricWaypointPlacer placer;
  final RoundTripTrackCleanup cleanup;

  public RoundTripOrchestrator(RoundTripEngineOps ops) {
    this.ops = ops;
    this.snapper = new WaypointSnapper(ops, ops, ops);
    this.placer = new GeometricWaypointPlacer(ops);
    this.cleanup = new RoundTripTrackCleanup(snapper, ops, ops, ops);
    // Constructed here, not in field initializers: the strategies capture
    // collaborators off this orchestrator, which must be fully wired first.
    this.fastStrategy = new FastStrategy(this);
    this.greedyStrategy = new GreedyStrategy(this);
    this.boundedStrategy = new BoundedStrategy(this);
  }

  // A loop must enclose area: at least a triangle (start + 2 intermediate waypoints).
  // A single intermediate point is only an out-and-back, not a loop.
  private static final int MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS = 2;

  // A produced round-trip below either bound is a degenerate stub, not a loop.
  static final int MIN_ROUNDTRIP_LOOP_NODES = 6;

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
  RoundTripQualityResult evaluateRoundTripGate(OsmTrack track, double searchRadius,
                                                boolean explicitViaMode) {
    return evaluateRoundTripGate(track, searchRadius, explicitViaMode,
      request.forcedCorridorAccepted);
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

  /**
   * Bridge for tier-internal fallbacks (bounded tier) into the waypoint tier;
   * the ladder itself dispatches through {@link FastStrategy}.
   */
  void doWaypointBasedRoundTrip(double searchRadius, double direction, RoundTripAlgorithm algo) {
    fastStrategy.attempt(request,
      new TierSlice(algo, null, searchRadius, direction, false, algo.toString()));
  }

  /**
   * Bridge for the bounded tier's planner slice; the ladder dispatches
   * through {@link GreedyStrategy}.
   */
  void doGreedyRoundTrip(double searchRadius, double direction, RoundTripAlgorithm algo) {
    greedyStrategy.attempt(request,
      new TierSlice(algo, null, searchRadius, direction, true, algo.toString()));
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



  public static RoundTripAlgorithm selectRoundTripAlgorithm(double searchRadius) {
    // Cheap fallback selector. The full AUTO policy lives in
    // {@link #runAutoCandidateCompetition}; this helper remains as a stable
    // entry point for direct callers and unsupported AUTO modes.
    return RoundTripAlgorithm.GREEDY;
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
  static long tierSliceMs(long tierBudgetMs, long requestDeadline, long now) {
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

  void recordPlacementPath(PlacementPath path) {
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
  static final long MIN_LADDER_RUNG_BUDGET_MS = 3_000;

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
  static final double CLEAR_ACCEPT_THRESHOLD = 0.85;

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

}
