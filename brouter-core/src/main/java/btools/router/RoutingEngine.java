package btools.router;

import btools.mapaccess.*;
import btools.router.roundtrip.*;
import btools.util.*;

import java.io.*;
import java.util.*;

public class RoutingEngine extends Thread {

  public final static int BROUTER_ENGINEMODE_ROUTING = 0;
  public final static int BROUTER_ENGINEMODE_SEED = 1;
  public final static int BROUTER_ENGINEMODE_GETELEV = 2;
  public final static int BROUTER_ENGINEMODE_GETINFO = 3;
  public final static int BROUTER_ENGINEMODE_ROUNDTRIP = 4;

  NodesCache nodesCache;
  private SortedHeap<OsmPath> openSet = new SortedHeap<>();
  private volatile boolean finished = false;

  protected List<OsmNodeNamed> waypoints = null;
  List<OsmNodeNamed> extraWaypoints = null;
  protected List<MatchedWaypoint> matchedWaypoints;
  private int linksProcessed = 0;

  private int nodeLimit; // used for target island search
  private int MAXNODES_ISLAND_CHECK = 500;
  OsmNodePairSet islandNodePairs = new OsmNodePairSet(MAXNODES_ISLAND_CHECK);
  private boolean useNodePoints = false; // use the start/end nodes  instead of crosspoint

  private int engineMode = 0;

  private int MAX_STEPS_CHECK = 500;

  private int ROUNDTRIP_DEFAULT_DIRECTIONADD = 45;
  private static final String PROFILE_PARAM_ALLOW_FERRIES = "allow_ferries";

  // A loop must enclose area: at least a triangle (start + 2 intermediate waypoints).
  // A single intermediate point is only an out-and-back, not a loop.
  private static final int MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS = 2;
  // A produced round-trip below either bound is a degenerate stub, not a loop.
  private static final int MIN_ROUNDTRIP_LOOP_NODES = 6;
  private static final int MIN_ROUNDTRIP_LOOP_METERS = 200;
  // A loop whose start/end gap exceeds this never returned to the origin.
  private static final int MAX_ROUNDTRIP_CLOSURE_METERS = 400;
  /** searchRadius for a 30km loop (=30km/2π); maxNodes baseline scales relative to this. */
  private static final double REFERENCE_LOOP_RADIUS_M = 30_000.0 / (2 * Math.PI);
  /** Per-area base maxNodes for isochrone Dijkstra at the reference radius. */
  private static final int BASE_ISOCHRONE_MAX_NODES = 300_000;
  /** Absolute ceiling for isochrone Dijkstra maxNodes (circuit breaker). */
  private static final int CEILING_ISOCHRONE_MAX_NODES = 1_500_000;

  /** Reference road-geometry indirectness the geometric loop-radius calibration is tuned to. */
  private static final double REFERENCE_GEOM_INDIRECTNESS = 1.25;
  /**
   * Assumed road/air indirectness for a direction with NO observed isochrone
   * geometry (probe-only frontier entries, and the no-iso-data fallback).
   * Deliberately equal to {@link #REFERENCE_GEOM_INDIRECTNESS}: an unknown
   * direction is assumed to behave like the calibration baseline, so the number
   * of probe-only directions does not perturb the global indirectness
   * compensation. (Was an inline {@code 1.3} literal at two sites — 0.05 above
   * the calibration reference for no documented reason; unified here so the
   * compensation has a single indirectness baseline.) Validate against the
   * loop-quality corpus when changing.
   */
  private static final double DEFAULT_PROBE_INDIRECTNESS = REFERENCE_GEOM_INDIRECTNESS;
  /** Clamp range on the indirectness compensation factor (±20% of geometric base). */
  private static final double IND_COMPENSATION_MIN = 0.80;
  private static final double IND_COMPENSATION_MAX = 1.20;

  /**
   * Isochrone Dijkstra cost-budget calibration.
   *
   * <p>A fixed budget {@code searchRadius × ISO_BUDGET_FLOOR_FACTOR} produces
   * wildly different physical pool depths depending on the profile's effective
   * cost-per-air-meter: ~2× searchRadius air reach for fastbike (costfactor
   * ≈ 1.3 × road indirectness ≈ 1.5), ~1.3× for mtb (costfactor ≈ 3), and a
   * starvation collapse for high-penalty profiles (an escape-class profile
   * measured at cost/m 8.9 reached only ~0.45× searchRadius — a 5.7km pool for
   * a 12.7km radius). A shallow, clustered pool makes the greedy planner
   * collapse to roughly half the requested loop length.
   *
   * <p>Fix: single-pass in-flight calibration. Dijkstra pops arrive in strictly
   * increasing cost order, so the pops whose cost falls in
   * {@code [ISO_CALIBRATION_SAMPLE_LO, 1.0] × searchRadius} form exactly the
   * frontier band a separate probe expansion would measure. Their median
   * cost-per-air-meter ({@code path.cost / airDist}) estimates the terrain's
   * effective costfactor × indirectness. At the first pop past the checkpoint
   * ({@code cost > searchRadius}) the budget is recomputed as
   * {@code ISO_TARGET_REACH_FACTOR × searchRadius × medianCostEff}, clamped to
   * {@code [ISO_BUDGET_FLOOR_FACTOR, ISO_BUDGET_CAP_FACTOR] × searchRadius}.
   *
   * <p>Why this is safe for the contour picks: the floor keeps every possible
   * contour target (25% of a ≥4× budget = ≥1× searchRadius) at or after the
   * checkpoint. When a raise fires, the frontier/contour best-scores are reset;
   * every node that could competitively fit the raised targets pops after the
   * checkpoint, so the reset discards nothing that could have won. When no
   * raise fires (fastbike lands on the floor), behavior is bit-identical to
   * the historical fixed budget. The geographic cutoff (1.5× searchRadius)
   * stays the outer physical bound — the raised budget lets slow directions
   * reach it instead of starving; {@code maxNodes} and the expansion deadline
   * still bound worst-case work.
   */
  static final double ISO_BUDGET_FLOOR_FACTOR = 4.0;
  static final double ISO_BUDGET_CAP_FACTOR = 12.0;
  /** Target air reach as a multiple of searchRadius (33% margin past the 1.5× geo cutoff). */
  static final double ISO_TARGET_REACH_FACTOR = 2.0;
  /** Lower edge of the calibration sampling band, as a fraction of searchRadius (upper edge = 1.0). */
  static final double ISO_CALIBRATION_SAMPLE_LO = 0.7;
  /** Below this many band samples the calibration is skipped (sparse graph → keep the floor). */
  static final int ISO_CALIBRATION_MIN_SAMPLES = 30;

  private int MAX_DYNAMIC_RANGE = 60000;

  protected OsmTrack foundTrack = new OsmTrack();
  private OsmTrack foundRawTrack = null;
  /**
   * The round-trip track that was rejected by the quality gate, if any.
   * {@link #foundTrack} is nulled on rejection so callers see a clean
   * "no track" outcome; this field preserves the rejected geometry for
   * post-mortem analysis (visual route inspection, failure
   * categorization, regression tests). Only populated for round-trip
   * mode; null in plain routing.
   */
  private OsmTrack lastRejectedTrack;
  private RoundTripResult lastRoundTripResult;
  /**
   * Set by {@link #doGreedyRoundTrip} when the adopted loop is a forced
   * same-way-back corridor (no clean alternative in constrained terrain). The
   * uniform round-trip gate then evaluates it with allowSamewayback=true so the
   * rideable corridor is accepted (disclosed) rather than rejected.
   */
  private boolean roundTripForcedCorridorAccepted;
  private int alternativeIndex = 0;

  protected String outputMessage = null;
  protected String errorMessage = null;

  private volatile boolean terminated;

  protected File segmentDir;
  private String outfileBase;
  private String logfileBase;
  private boolean infoLogEnabled;
  private Writer infoLogWriter;
  private StackSampler stackSampler;
  protected RoutingContext routingContext;

  public double airDistanceCostFactor;
  public double lastAirDistanceCostFactor;

  private OsmTrack guideTrack;

  OsmTrack[] greedyLegTracks; // per-leg cost-cutting tracks from greedy planner

  private OsmPathElement matchPath;

  // Saved/restored across leg attempts by GreedyRoundTripPlanner.timedFindTrack
  // and read by the _findTrack timeout arithmetic — all on the same worker
  // thread (the cross-thread watchdog channel is the `terminated` flag, not
  // these fields). volatile is defensive: it keeps the 64-bit reads/writes
  // atomic should a watchdog ever read them, and is harmless otherwise.
  volatile long startTime;
  volatile long maxRunningTime;
  // Wall-clock budget (ms) for the routing legs of a round trip, captured from
  // doRun() so the WAYPOINT/ISOCHRONE/greedy-fallthrough doRouting() calls are
  // bounded. 0 (the CLI default) keeps the legacy no-timeout behaviour.
  private long roundTripRoutingBudgetMs;
  /**
   * Absolute wall-clock deadline (epoch ms) for the whole round-trip request,
   * set once by doRun(). Every retry layer (subRouteCount ladder, Phase 2.1
   * axis retry, ISO_GREEDY→GREEDY recursion, fallback doRouting) and the
   * isochrone expansion loop consult it, so the retry machinery can no longer
   * multiply the request budget into minutes. 0 = unbounded (CLI / doRun(0)).
   */
  volatile long roundTripRequestDeadline;

  /** Milliseconds left until {@link #roundTripRequestDeadline} (MAX_VALUE when unbounded). */
  long remainingRequestBudgetMs() {
    return roundTripRequestDeadline == 0
      ? Long.MAX_VALUE
      : roundTripRequestDeadline - System.currentTimeMillis();
  }

  /**
   * The request's resolved effort configuration. BALANCED pins the BOUNDED
   * preset (reduced top-K, hard 8s-per-slice budget, no retry ladders),
   * QUALITY pins MAX (both planners always, top-K 4/6, doubled plan budget),
   * and AUTO resolves a preset from context — profile class, length class,
   * resources (see {@link RoundTripEffortPolicy#resolveAuto}). Planners
   * constructed under this engine read their knobs from it; AUTO child
   * engines inherit it.
   */
  RoundTripEffortPolicy roundTripEffortPolicy = RoundTripEffortPolicy.STANDARD_PRESET;

  /**
   * Per-call wall-clock bound for the next {@link #runIsochroneExpansion}
   * (epoch ms, 0 = none), set/cleared by the greedy planner around
   * candidatesForStep. The expansion loop historically had NO time or
   * termination check at all — only cost/geo/node caps — so a single
   * dense-area expansion (up to 1.5M pops) could neither respect the plan
   * deadline nor be killed by the watchdog.
   */
  volatile long transientExpansionDeadline;
  public SearchBoundary boundary;

  public boolean quite = false;

  /**
   * Reachability-cloud cell size for pocket-avoiding waypoint placement: every
   * node an isochrone expansion pops is bucketed into cells of roughly this
   * many meters. ~150m keeps a 5×5 neighborhood at ~750m — local enough that a
   * dead-end corridor (cells along one line) is distinguishable from a
   * junction-rich neighborhood (filled square).
   */
  static final int REACHABILITY_CELL_M = 150;
  private boolean suppressRoutingIslandGuard = false;

  private Object[] extract;

  private boolean directWeaving = !Boolean.getBoolean("disableDirectWeaving");
  private String outfile;

  double roundTripSearchRadius = 0;

  /**
   * Greedy route-choice threshold for clear accept. If ISO_GREEDY scores
   * below this, AUTO normally runs the plain GREEDY candidate as a comparison
   * before considering the legacy WAYPOINT fallback. Graph-native absorption
   * inside ISO_GREEDY is the measured exception.
   *
   * <p>Absorption path (issue #26): ISO_GREEDY now carries an internal
   * graph-native fallback (see {@link IsoPoolHealth}) that demotes a thin,
   * bunched, or repeatedly-losing iso pool mid-plan, so this separate GREEDY
   * comparison should win less and less. It is deliberately retained until
   * winner-attribution evidence proves it is no longer needed — grep AUTO
   * competition logs for GREEDY winners and check the ISO_GREEDY candidate's
   * {@code quotaAccepted}/{@code poolHealth}/{@code demotedAtStep} suffix
   * ({@link RoundTripCandidateResult#toString}) before removing it.
   */
  private static final double CLEAR_ACCEPT_THRESHOLD = 0.85;

  enum IsoStartPolicy {
    BLEND,
    GRAPH_NATIVE_ONLY
  }

  // AUTO competition runs its candidates sequentially in the calling thread and
  // cannot interrupt a child mid-run, so it shares one wall-clock budget across
  // all candidates instead of giving each the full timeout. DEFAULT applies when
  // the caller passes no timeout (maxRunningTime <= 0); MIN_CHILD guarantees a
  // spawned candidate still gets a usable slice.
  private static final long DEFAULT_AUTO_BUDGET_MS = 60_000;
  private static final long MIN_CHILD_BUDGET_MS = 5_000;
  /**
   * Minimum remaining request budget worth starting another subRouteCount
   * ladder rung, Phase-2.1 retry or ISO_GREEDY→GREEDY recursion for. Below
   * this a fresh plan() could not route even a couple of legs, so the time is
   * better left to the fallback/adoption path.
   */
  private static final long MIN_LADDER_RUNG_BUDGET_MS = 3_000;
  /**
   * Product sizing (2026-07): loops up to this length must work with the
   * standard request budget; anything longer requires the caller to opt in
   * with a raised timeout (see the gate in doRoundTrip).
   */
  static final double MAX_STANDARD_LOOP_METERS = 200_000;
  /** Minimum request budget accepted for loops above {@link #MAX_STANDARD_LOOP_METERS}. */
  static final long LONG_LOOP_MIN_BUDGET_MS = 120_000;
  /**
   * Unwind margin for the parallel AUTO GREEDY child join: how long past its
   * own budget the request thread waits for the child to stop before
   * terminating it and moving on. Bounds the join so a wedged or
   * budget-overshooting child can never hang the request thread.
   */
  private static final long AUTO_CHILD_JOIN_UNWIND_MS = 3_000;
  /**
   * Issue #26 default: do not start plain GREEDY speculatively before
   * ISO_GREEDY has proved it is needed. This avoids duplicate production
   * algorithm runs on strong or provider-level graph-native-absorbed
   * ISO_GREEDY results. Operators who prefer the old single-request latency
   * tradeoff can opt back in with {@code -DroundTripSpeculativeAutoGreedy=true}.
   */
  private static final boolean SPECULATIVE_AUTO_GREEDY =
    Boolean.getBoolean("roundTripSpeculativeAutoGreedy");
  /**
   * Global bound on how many AUTO round-trip requests may run their speculative
   * GREEDY child IN PARALLEL at once when {@link #SPECULATIVE_AUTO_GREEDY} is
   * enabled (a non-blocking permit pool). Routing is CPU-bound, so this caps
   * the extra CPU-bound threads the opt-in parallelism adds across the whole
   * JVM. Tunable via {@code -DroundTripParallelAutoPermits}; set 0 to force
   * fully-sequential AUTO competition even when speculative mode is enabled.
   */
  private static final java.util.concurrent.Semaphore PARALLEL_AUTO_SEMAPHORE =
    new java.util.concurrent.Semaphore(Math.max(0,
      Integer.getInteger("roundTripParallelAutoPermits",
        Runtime.getRuntime().availableProcessors() - 1)));
  /**
   * Set by {@link #doExplicitViaRoundTrip} when the request supplies user
   * via points in round-trip mode. Routing-time micro-detour and back-and-forth
   * removal must be skipped in this mode — those passes were designed for
   * auto-generated loops with many rt* waypoints, and they aggressively
   * delete the entire route when the closing waypoint sits at the same
   * position as the start (crow-fly = 0, which always trips the ratio
   * threshold). User-via routes are also typically short and shape-
   * preserving by intent; the user picked them, the engine should not
   * post-edit them away.
   */
  boolean explicitViaRoundTrip = false;

  public RoutingEngine(String outfileBase, String logfileBase, File segmentDir,
                       List<OsmNodeNamed> waypoints, RoutingContext rc) {
    this(outfileBase, logfileBase, segmentDir, waypoints, rc, 0);
  }

  public RoutingEngine(String outfileBase, String logfileBase, File segmentDir,
                       List<OsmNodeNamed> waypoints, RoutingContext rc, int engineMode) {
    this.segmentDir = segmentDir;
    this.outfileBase = outfileBase;
    this.logfileBase = logfileBase;
    this.waypoints = waypoints;
    this.infoLogEnabled = outfileBase != null;
    this.routingContext = rc;
    this.engineMode = engineMode;
    if (engineMode == BROUTER_ENGINEMODE_ROUNDTRIP) {
      // Mark the context as round-trip up front: this gates the anti-reuse
      // refTrack penalty in OsmPath to its edge-membership form (see
      // RoutingContext.roundTrip) so loop legs avoid retracing traveled ways,
      // while general routing keeps the historic node-membership test unchanged.
      rc.roundTrip = true;
      applyRoundTripProfileDefaults(rc);
    }

    File baseFolder = new File(routingContext.localFunction).getParentFile();
    baseFolder = baseFolder == null ? null : baseFolder.getParentFile();
    if (baseFolder != null) {
      try {
        File debugLog = new File(baseFolder, "debug.txt");
        if (debugLog.exists()) {
          infoLogWriter = new FileWriter(debugLog, true);
          logInfo("********** start request at ");
          logInfo("********** " + new Date());
        }
      } catch (IOException ioe) {
        throw new RuntimeException("cannot open debug-log:" + ioe);
      }

      File stackLog = new File(baseFolder, "stacks.txt");
      if (stackLog.exists()) {
        stackSampler = new StackSampler(stackLog, 1000);
        stackSampler.start();
        logInfo("********** started stacksampling");
      }
    }
    boolean cachedProfile = ProfileCache.parseProfile(rc);
    if (hasInfo()) {
      logInfo("parsed profile " + rc.localFunction + " cached=" + cachedProfile);
    }

  }

  /**
   * Generated cycling loops default to no ferries. Point-to-point routing keeps
   * the profile defaults, but loop generation must not discover an attractive
   * ferry shortcut unless the caller explicitly opts in via
   * {@code profile:allow_ferries=true}.
   */
  private static void applyRoundTripProfileDefaults(RoutingContext rc) {
    if (rc == null) return;
    if (rc.keyValues == null) {
      rc.keyValues = new HashMap<>();
      rc.keyValues.put(PROFILE_PARAM_ALLOW_FERRIES, "0");
      return;
    }
    if (!rc.keyValues.containsKey(PROFILE_PARAM_ALLOW_FERRIES)) {
      rc.keyValues = new HashMap<>(rc.keyValues);
      rc.keyValues.put(PROFILE_PARAM_ALLOW_FERRIES, "0");
    }
  }

  private boolean roundTripFerriesAllowed() {
    if (routingContext == null || routingContext.keyValues == null) return false;
    String v = routingContext.keyValues.get(PROFILE_PARAM_ALLOW_FERRIES);
    return v != null && ("true".equalsIgnoreCase(v) || "1".equals(v) || "yes".equalsIgnoreCase(v));
  }

  private boolean hasInfo() {
    return infoLogEnabled || infoLogWriter != null;
  }

  void logInfo(String s) {
    if (infoLogEnabled) {
      System.out.println(s);
    }
    if (infoLogWriter != null) {
      try {
        infoLogWriter.write(s);
        infoLogWriter.write('\n');
        infoLogWriter.flush();
      } catch (IOException io) {
        infoLogWriter = null;
      }
    }
  }

  private void logThrowable(Throwable t) {
    StringWriter sw = new StringWriter();
    PrintWriter pw = new PrintWriter(sw);
    t.printStackTrace(pw);
    logInfo(sw.toString());
  }

  public void run() {
    doRun(0);
  }

  public void doRun(long maxRunningTime) {
    // Note: this.maxRunningTime is set by the branches that route (doRouting
    // sets it; the round-trip branch sets it for the competition). GETINFO/
    // GETELEV deliberately leave it at its default so they stay untimed.
    switch (engineMode) {
      case BROUTER_ENGINEMODE_ROUTING:
        if (waypoints.size() < 2) {
          throw new IllegalArgumentException("we need two lat/lon points at least!");
        }
        doRouting(maxRunningTime);
        break;
      case BROUTER_ENGINEMODE_SEED: /* do nothing, handled the old way */
        throw new IllegalArgumentException("not a valid engine mode");
      case BROUTER_ENGINEMODE_GETELEV:
      case BROUTER_ENGINEMODE_GETINFO:
        if (waypoints.size() < 1) {
          throw new IllegalArgumentException("we need one lat/lon point at least!");
        }
        doGetInfo();
        break;
      case BROUTER_ENGINEMODE_ROUNDTRIP:
        if (waypoints.size() < 1)
          throw new IllegalArgumentException("we need one lat/lon point at least!");
        // Capture the request's wall-clock budget so the round-trip routing
        // legs (WAYPOINT/ISOCHRONE/greedy fallthrough) honour it instead of
        // running untimed, and so the AUTO competition can share it. 0 keeps
        // the legacy unbounded behaviour for the CLI.
        this.maxRunningTime = maxRunningTime;
        roundTripRoutingBudgetMs = maxRunningTime;
        // Anchor the engine clock for searches that run outside doRouting /
        // timedFindTrack (e.g. the repairViaPinnedBulges connector search in
        // the greedy bypass path). Before this, engine.startTime stayed 0 in
        // that path, so with maxRunningTime > 0 the connector's timeout check
        // (now - startTime > budget) fired instantly and every bulge repair
        // silently failed on servers.
        this.startTime = System.currentTimeMillis();
        // Absolute wall-clock deadline for the WHOLE round-trip request. This
        // is what the greedy planner ladder, the isochrone expansions and the
        // fallback doRouting consult so retries can never multiply the
        // request budget (the historical minutes-long worst case). 0 keeps
        // untimed callers (CLI, doRun(0) tests) unbounded.
        roundTripRequestDeadline = maxRunningTime > 0
          ? this.startTime + maxRunningTime : 0;
        doRoundTrip();
        break;
      default:
        throw new IllegalArgumentException("not a valid engine mode");
    }
  }


  public void doRouting(long maxRunningTime) {
    try {
      startTime = System.currentTimeMillis();
      long startTime0 = startTime;
      this.maxRunningTime = maxRunningTime;

      if (routingContext.allowSamewayback) {
        if (waypoints.size() == 2) {
          OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(waypoints.get(0).ilon, waypoints.get(0).ilat));
          onn.name = "to";
          waypoints.add(onn);
        } else {
          waypoints.get(waypoints.size() - 1).name = "via" + (waypoints.size() - 1) + "_center";
          List<OsmNodeNamed> newpoints = new ArrayList<>();
          for (int i = waypoints.size() - 2; i >= 0; i--) {
            // System.out.println("back " + waypoints.get(i));
            OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(waypoints.get(i).ilon, waypoints.get(i).ilat));
            onn.name = "via";
            newpoints.add(onn);
          }
          newpoints.get(newpoints.size() - 1).name = "to";
          waypoints.addAll(newpoints);
        }
      }

      int nsections = waypoints.size() - 1;
      OsmTrack[] refTracks = new OsmTrack[nsections]; // used ways for alternatives
      OsmTrack[] lastTracks = new OsmTrack[nsections];
      OsmTrack track = null;
      List<String> messageList = new ArrayList<>();
      for (int i = 0; ; i++) {
        track = findTrack(refTracks, lastTracks);

        // we are only looking for info
        if (routingContext.ai != null) return;

        track.message = "track-length = " + track.distance + " filtered ascend = " + track.ascend
          + " plain-ascend = " + track.plainAscend + " cost=" + track.cost;
        if (track.energy != 0) {
          track.message += " energy=" + Formatter.getFormattedEnergy(track.energy) + " time=" + Formatter.getFormattedTime2(track.getTotalSeconds());
        }
        track.name = "brouter_" + routingContext.getProfileName() + "_" + i;

        messageList.add(track.message);
        track.messageList = messageList;
        if (outfileBase != null) {
          String filename = outfileBase + i + "." + routingContext.outputFormat;
          OsmTrack oldTrack = null;
          switch (routingContext.outputFormat) {
            case "gpx":
              oldTrack = new FormatGpx(routingContext).read(filename);
              break;
            case "geojson": // read only gpx at the moment
            case "json":
              // oldTrack = new FormatJson(routingContext).read(filename);
              break;
            case "kml":
              // oldTrack = new FormatJson(routingContext).read(filename);
              break;
            default:
              break;
          }
          if (oldTrack != null && track.equalsTrack(oldTrack)) {
            continue;
          }
          oldTrack = null;
          track.exportWaypoints = routingContext.exportWaypoints;
          track.exportCorrectedWaypoints = routingContext.exportCorrectedWaypoints;
          filename = outfileBase + i + "." + routingContext.outputFormat;
          switch (routingContext.outputFormat) {
            case "gpx":
              outputMessage = new FormatGpx(routingContext).format(track);
              break;
            case "geojson":
            case "json":
              outputMessage = new FormatJson(routingContext).format(track);
              break;
            case "kml":
              outputMessage = new FormatKml(routingContext).format(track);
              break;
            case "csv":
            default:
              outputMessage = null;
              break;
          }
          if (outputMessage != null) {
            File out = new File(filename);
            FileWriter fw = new FileWriter(filename);
            fw.write(outputMessage);
            fw.close();
            outputMessage = null;
          }

          foundTrack = track;
          alternativeIndex = i;
          outfile = filename;
        } else {
          if (i == routingContext.getAlternativeIdx(0, 3)) {
            if ("CSV".equals(System.getProperty("reportFormat"))) {
              String filename = outfileBase + i + ".csv";
              new FormatCsv(routingContext).write(filename, track);
            } else {
              if (!quite) {
                System.out.println(new FormatGpx(routingContext).format(track));
              }
            }
            foundTrack = track;
          } else {
            continue;
          }
        }
        if (logfileBase != null) {
          String logfilename = logfileBase + i + ".csv";
          new FormatCsv(routingContext).write(logfilename, track);
        }
        break;
      }
      long endTime = System.currentTimeMillis();
      logInfo("execution time = " + (endTime - startTime0) / 1000. + " seconds");
    } catch (IllegalArgumentException e) {
      logException(e);
    } catch (Exception e) {
      logException(e);
      logThrowable(e);
    } catch (Error e) {
      cleanOnOOM();
      logException(e);
      logThrowable(e);
    } finally {
      if (hasInfo() && routingContext.expctxWay != null) {
        logInfo("expression cache stats=" + routingContext.expctxWay.cacheStats());
      }

      ProfileCache.releaseProfile(routingContext);

      if (nodesCache != null) {
        if (hasInfo() && nodesCache != null) {
          logInfo("NodesCache status before close=" + nodesCache.formatStatus());
        }
        nodesCache.close();
        nodesCache = null;
      }
      openSet.clear();
      // Signal termination to outside pollers — but NOT for the round-trip path.
      // In round-trip mode doRouting only produces the raw loop skeleton; the
      // outer doRoundTrip still runs the quality gate afterwards and can null
      // foundTrack / set errorMessage. Publishing `finished` here would let a
      // polling caller (e.g. BRouterView) read an intermediate result. The
      // round-trip path publishes `finished` in cleanupRoutingResources(), which
      // runs in doRoundTrip's finally after the gate has decided.
      if (engineMode != BROUTER_ENGINEMODE_ROUNDTRIP) {
        finished = true; // this signals termination to outside
      }

      if (infoLogWriter != null) {
        try {
          infoLogWriter.close();
        } catch (Exception e) {
        }
        infoLogWriter = null;
      }

      if (stackSampler != null) {
        try {
          stackSampler.close();
        } catch (Exception e) {
        }
        stackSampler = null;
      }

    }
  }

  public void doGetInfo() {
    try {
      startTime = System.currentTimeMillis();

      routingContext.freeNoWays();

      MatchedWaypoint wpt1 = new MatchedWaypoint();
      wpt1.waypoint = waypoints.get(0);
      wpt1.name = "wpt_info";
      List<MatchedWaypoint> listOne = new ArrayList<>();
      listOne.add(wpt1);
      matchWaypointsToNodes(listOne);

      resetCache(true);
      nodesCache.nodesMap.cleanupMode = 0;

      OsmNode start1 = nodesCache.getGraphNode(listOne.get(0).node1);
      boolean b = nodesCache.obtainNonHollowNode(start1);

      guideTrack = new OsmTrack();
      guideTrack.addNode(OsmPathElement.create(wpt1.node2.ilon, wpt1.node2.ilat, (short) 0, null));
      guideTrack.addNode(OsmPathElement.create(wpt1.node1.ilon, wpt1.node1.ilat, (short) 0, null));

      matchedWaypoints = new ArrayList<>();
      MatchedWaypoint wp1 = new MatchedWaypoint();
      wp1.crosspoint = new OsmNode(wpt1.node1.ilon, wpt1.node1.ilat);
      wp1.node1 = new OsmNode(wpt1.node1.ilon, wpt1.node1.ilat);
      wp1.node2 = new OsmNode(wpt1.node2.ilon, wpt1.node2.ilat);
      matchedWaypoints.add(wp1);
      MatchedWaypoint wp2 = new MatchedWaypoint();
      wp2.crosspoint = new OsmNode(wpt1.node2.ilon, wpt1.node2.ilat);
      wp2.node1 = new OsmNode(wpt1.node1.ilon, wpt1.node1.ilat);
      wp2.node2 = new OsmNode(wpt1.node2.ilon, wpt1.node2.ilat);
      matchedWaypoints.add(wp2);

      OsmTrack t = findTrack("getinfo", wp1, wp2, null, null, false);
      if (t != null) {
        t.messageList = new ArrayList<>();
        t.matchedWaypoints = matchedWaypoints;
        t.name = (outfileBase == null ? "getinfo" : outfileBase);

        // find nearest point
        int mindist = 99999;
        int minIdx = -1;
        for (int i = 0; i < t.nodes.size(); i++) {
          OsmPathElement ope = t.nodes.get(i);
          int dist = ope.calcDistance(listOne.get(0).crosspoint);
          if (mindist > dist) {
            mindist = dist;
            minIdx = i;
          }
        }
        int otherIdx = 0;
        if (minIdx == t.nodes.size() - 1) {
          otherIdx = minIdx - 1;
        } else {
          otherIdx = minIdx + 1;
        }
        int otherdist = t.nodes.get(otherIdx).calcDistance(listOne.get(0).crosspoint);
        int minSElev = t.nodes.get(minIdx).getSElev();
        int otherSElev = t.nodes.get(otherIdx).getSElev();
        int diffSElev = 0;
        diffSElev = otherSElev - minSElev;
        double diff = (double) mindist / (mindist + otherdist) * diffSElev;


        OsmNodeNamed n = new OsmNodeNamed(listOne.get(0).crosspoint);
        n.name = wpt1.name;
        n.selev = minIdx != -1 ? (short) (minSElev + (int) diff) : Short.MIN_VALUE;
        if (engineMode == BROUTER_ENGINEMODE_GETINFO) {
          n.nodeDescription = (start1 != null && start1.firstlink != null ? start1.firstlink.descriptionBitmap : null);
          t.pois.add(n);
          //t.message = "get_info";
          //t.messageList.add(t.message);
          t.matchedWaypoints = listOne;
          t.exportWaypoints = routingContext.exportWaypoints;
        }

        switch (routingContext.outputFormat) {
          case "gpx":
            if (engineMode == BROUTER_ENGINEMODE_GETELEV) {
              outputMessage = new FormatGpx(routingContext).formatAsWaypoint(n);
            } else {
              outputMessage = new FormatGpx(routingContext).format(t);
            }
            break;
          case "geojson":
          case "json":
            if (engineMode == BROUTER_ENGINEMODE_GETELEV) {
              outputMessage = new FormatJson(routingContext).formatAsWaypoint(n);
            } else {
              outputMessage = new FormatJson(routingContext).format(t);
            }
            break;
          case "kml":
          case "csv":
          default:
            outputMessage = null;
            break;
        }
        if (outfileBase != null) {
          String filename = outfileBase + "." + routingContext.outputFormat;
          File out = new File(filename);
          FileWriter fw = new FileWriter(filename);
          fw.write(outputMessage);
          fw.close();
          outputMessage = null;
        } else {
          if (!quite && outputMessage != null) {
            System.out.println(outputMessage);
          }
        }

      } else {
        if (errorMessage == null) errorMessage = "no track found";
      }
      long endTime = System.currentTimeMillis();
      logInfo("execution time = " + (endTime - startTime) / 1000. + " seconds");
    } catch (Exception e) {
      e.getStackTrace();
      logException(e);
    }
  }

  /**
   * Top-level driver for round-trip (loop) generation, called from {@code doRun}
   * for {@link #BROUTER_ENGINEMODE_ROUNDTRIP}. Steps:
   * <ol>
   *   <li>Derive the internal {@code searchRadius} from {@code roundTripLength}
   *       (total loop distance / 2π) or {@code roundTripDistance} (radius).</li>
   *   <li>Resolve the start bearing (user-supplied or data-driven random draw).</li>
   *   <li>Dispatch: explicit-via mode when the caller supplied via points
   *       ({@link #doExplicitViaRoundTrip}); otherwise pick the planner —
   *       AUTO normally runs {@link #runAutoCandidateCompetition} (which writes
   *       {@code foundTrack}/{@code errorMessage} and returns early), else one of
   *       the greedy ({@link #doGreedyRoundTrip}) or waypoint
   *       ({@link #doWaypointBasedRoundTrip}) planners.</li>
   *   <li>Run the uniform acceptance gate ({@link RoundTripQualityGate#evaluate})
   *       on the produced loop, then either hard-reject (nulling {@code foundTrack}
   *       into {@code lastRejectedTrack}) or keep it and surface advisory/disclosure
   *       messages. Lenient by default; {@code roundTripStrictQuality=1} hard-rejects
   *       QUALITY-tier failures.</li>
   *   <li>{@link #ensureInfoMessage} syncs the messages onto the track so they reach
   *       GPX/JSON output.</li>
   * </ol>
   * The result is the loop in {@code foundTrack}, or {@code errorMessage} set and
   * {@code foundTrack} null on failure.
   */
  public void doRoundTrip() {
    try {
      long startTime = System.currentTimeMillis();

      routingContext.useDynamicDistance = true;
      // Classify the profile's surface policy once, from its cost model (not its
      // name), so the quality gate and planner hostility checks use a consistent,
      // name-independent verdict for the rest of this request.
      RoundTripQualityGate.classifyPavedProfile(routingContext.expctxWay, routingContext.getProfileName());
      double searchRadius;
      if (routingContext.roundTripLength != null) {
        // roundTripLength is the desired total loop distance — convert to internal search radius.
        // The waypoint strategies place points at searchRadius from start and route between them,
        // so the loop traces roughly the circle circumference: total ≈ 2*PI * searchRadius.
        // Do NOT raise this factor toward L/2 (the out-and-back relation) thinking it gives a
        // "wider" loop: a closed loop traces the circumference, so a larger radius overshoots.
        // Measured across 4 real regions (urban/alpine/coastal/rural) for a 40km target, the
        // distance ratio climbs monotonically with the factor — L/2π≈0.91, 0.20→1.3, 0.25→1.6,
        // 0.33→2.1, L/2→3.2 — so L/2π is the calibrated optimum (closest to 1.0, best composite).
        searchRadius = routingContext.roundTripLength / (2 * Math.PI);
      } else {
        // Defensive floor: a non-positive roundTripDistance (e.g. set directly on
        // the context, bypassing the param-layer guard) would otherwise become a
        // zero/negative searchRadius. That ships a wrong-scale loop with the
        // distance gate silently disabled — the ratio check is skipped when
        // expectedDistance (2*PI*searchRadius) <= 0 — so floor it to the default.
        searchRadius = (routingContext.roundTripDistance == null
          || routingContext.roundTripDistance <= 0) ? 1500 : routingContext.roundTripDistance;
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
      OsmNodeNamed rtStart = waypoints.get(0);
      if (nodesCache == null) {
        resetCache(false);
      }
      if (!nodesCache.hasSegmentFor(rtStart.ilon, rtStart.ilat)) {
        errorMessage = "datafile " + nodesCache.getSegmentFileName(rtStart.ilon, rtStart.ilat) + " not found";
        logInfo(errorMessage);
        return;
      }

      double direction = (routingContext.startDirection == null ? -1 :routingContext.startDirection);
      double directionAdd = (routingContext.roundTripDirectionAdd == null ? ROUNDTRIP_DEFAULT_DIRECTIONADD :routingContext.roundTripDirectionAdd);
      if (direction == -1) {
        direction = getRandomDirectionFromData(waypoints.get(0), searchRadius);
        direction += directionAdd;
      }
      // Normalize to a [0,360) compass bearing: getRandomDirectionFromData()+directionAdd
      // can exceed 360 (e.g. 332+45=377), and a user-supplied startDirection may be out of
      // range, while downstream bearing comparisons assume a normalized value.
      direction = CheapAngleMeter.normalize(direction);

      // Explicit-via round-trip: when the caller supplied via points (any
      // waypoint beyond the start), treat those vias as a hard route
      // skeleton and bypass all generated-loop placement, regardless of
      // roundTripAlgorithm. User vias express stronger intent than any AUTO
      // heuristic, so they win. Generated rt* points are never added; the via
      // order is preserved exactly; distance settings become advisory.
      boolean explicitViaMode = waypoints.size() > 1;
      if (explicitViaMode) {
        logInfo("round trip: explicit-via mode (" + (waypoints.size() - 1) + " user via points)");
        // Variety-seed disclosure: user vias are a hard skeleton expressing
        // stronger intent than any heuristic, so the alternativeidx seed is ignored.
        if (routingContext.getRoundTripSeed() > 0) {
          logInfo("alternativeidx has no effect in explicit-via round trips");
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
            && maxRunningTime > 0 && maxRunningTime < LONG_LOOP_MIN_BUDGET_MS) {
          errorMessage = "round trips above " + (int) (MAX_STANDARD_LOOP_METERS / 1000)
            + "km need an explicitly increased calculation budget: requested "
            + Math.round(requestedLoopMeters / 1000.0) + "km with a "
            + (maxRunningTime / 1000) + "s timeout; raise maxRunningTime to at least "
            + (LONG_LOOP_MIN_BUDGET_MS / 1000) + "s";
          logInfo(errorMessage);
          return;
        }
        // Resolve the roundTripIsochrone shortcut into the canonical
        // roundTripAlgorithm ONCE, so the algorithm is the single source of
        // truth from here down and the boolean never has to propagate to child
        // contexts. Honoured only when no explicit algorithm was chosen — an
        // explicit algorithm always wins.
        if (routingContext.roundTripAlgorithm == RoundTripAlgorithm.AUTO
            && routingContext.roundTripIsochrone) {
          routingContext.roundTripAlgorithm = RoundTripAlgorithm.ISOCHRONE;
        }
        RoundTripAlgorithm algo = routingContext.roundTripAlgorithm;

        // Request context for the effort policy: profile class from the
        // profile's own validFor* globals (name-independent), coarse length
        // class, and resources. Logged once so future policy rules land on
        // recorded evidence.
        RoundTripEffortPolicy.ProfileClass profileClass = classifyProfileClass();
        RoundTripEffortPolicy.LengthClass lengthClass =
          RoundTripEffortPolicy.classifyLength(2 * Math.PI * searchRadius);
        if (profileClass == RoundTripEffortPolicy.ProfileClass.MOTOR) {
          logInfo("round trip: profile class MOTOR — loop quality is unvalidated for"
            + " motorized profiles; using bike-derived policies (provisional)");
        }

        boolean greedyCapable = greedySupports(routingContext.allowSamewayback, waypoints.size());

        // QUALITY: the full competition at max effort — both planners always
        // run, wider routed top-K, doubled plan budget. NOT an ISO_GREEDY
        // alias: greedy wins ~a quarter of competition cells.
        if (algo == RoundTripAlgorithm.QUALITY && greedyCapable) {
          roundTripEffortPolicy = RoundTripEffortPolicy.MAX_PRESET;
          logInfo("round trip effort: " + roundTripEffortPolicy.rationale);
          runAutoCandidateCompetition(searchRadius, direction);
          return;
        }
        if (algo == RoundTripAlgorithm.QUALITY) {
          // Same constraint as the greedy/BALANCED branches: the planners do
          // not honor allowSamewayback. Name the tier in the log — the silent
          // rewrite below (QUALITY -> selectRoundTripAlgorithm -> WAYPOINT)
          // otherwise hides that the MAX effort request was downgraded.
          logInfo("QUALITY round trip does not support allowSamewayback, falling back to waypoint algorithm");
        }

        // AUTO candidate competition, effort resolved from context.
        //
        // Generated loops default to greedy Dijkstra construction. AUTO runs
        // ISO_GREEDY first, then GREEDY, and considers the legacy
        // WAYPOINT/probe path only as a separately scored fallback candidate
        // if greedy cannot produce an accepted route (see
        // runAutoCandidateCompetition for the full competition policy).
        // Constrained resources (short request budget, memory-constrained
        // device) resolve to the BOUNDED preset — the bounded dispatch below
        // instead of the full competition.
        if (algo == RoundTripAlgorithm.AUTO && greedyCapable) {
          RoundTripEffortPolicy resolved = RoundTripEffortPolicy.resolveAuto(
            profileClass, lengthClass, routingContext.memoryclass, maxRunningTime);
          logInfo("round trip effort: " + resolved.rationale);
          if (resolved.preset != RoundTripEffortPolicy.Preset.BOUNDED) {
            roundTripEffortPolicy = resolved;
            runAutoCandidateCompetition(searchRadius, direction);
            // The competition method writes foundTrack / errorMessage directly
            // (its children are gated inside the competition).
            return;
          }
          // Constrained resources: the same bounded dispatch as explicit
          // BALANCED — and the same fall-through to the shared floors and
          // quality gate below. The bounded tier adopts best-effort tracks
          // and defers hard-reject to that uniform gate, so an early return
          // here would ship ungated tracks that an identical explicit
          // BALANCED request rejects or returns with a Warning.
          doBoundedRoundTrip(searchRadius, direction, resolved, "AUTO(bounded)", true);
        } else {
          if (algo == RoundTripAlgorithm.AUTO || algo == RoundTripAlgorithm.QUALITY) {
            algo = selectRoundTripAlgorithm(searchRadius);
          }
          logInfo("round trip algorithm: " + algo);

          if (algo == RoundTripAlgorithm.BALANCED) {
            // allowSamewayback is handled inside the bounded tier: the planner
            // slice is skipped, but the waypoint placement keeps the tier
            // budget instead of inheriting the full request budget.
            doBoundedRoundTrip(searchRadius, direction,
              RoundTripEffortPolicy.BOUNDED_PRESET, "BALANCED", greedyCapable);
          } else if (algo == RoundTripAlgorithm.GREEDY || algo == RoundTripAlgorithm.ISO_GREEDY) {
            if (!greedySupports(routingContext.allowSamewayback, waypoints.size())) {
              // Greedy generates its own intermediate points and does not honor
              // allowSamewayback. (User vias are handled in explicitViaMode above.)
              logInfo("greedy round trip does not support allowSamewayback, falling back to waypoint algorithm");
              doWaypointBasedRoundTrip(searchRadius, direction, RoundTripAlgorithm.WAYPOINT);
            } else {
              // ISO_GREEDY: isochrone-derived candidate pool. Falls back to plain
              // GREEDY internally if the candidate pool is insufficient.
              doGreedyRoundTrip(searchRadius, direction, algo);
            }
          } else {
            doWaypointBasedRoundTrip(searchRadius, direction, algo);
          }
        }
      }

      if (foundTrack == null && errorMessage != null) {
        return;
      }

      // A loop needs at least a triangle (start + 2 intermediate waypoints). With a single
      // intermediate the route is only an out-and-back, which closure/detour handling cannot
      // turn into a loop. Same-way-back is the deliberate exception (it IS an out-and-back).
      //
      // Explicit-via mode skips this check: a single user-supplied via is a valid
      // route skeleton (start → via1 → start), even though the result shape is
      // out-and-back. The user is expressing route intent, not a loop request.
      int intermediateWaypoints = (matchedWaypoints == null) ? 0 : matchedWaypoints.size() - 2;
      if (!routingContext.allowSamewayback && !explicitViaMode
          && intermediateWaypoints < MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS) {
        errorMessage = "round-trip could not place enough waypoints to form a loop (need "
          + MIN_ROUNDTRIP_INTERMEDIATE_WAYPOINTS + " intermediate, got " + Math.max(0, intermediateWaypoints)
          + ") for direction " + (int) direction + " at radius " + (int) searchRadius + "m";
        logInfo(errorMessage);
        foundTrack = null;
        return;
      }

      // Contract: a round-trip must yield an actual loop. When intermediate waypoints
      // cannot be placed on reachable roads (e.g. the requested direction has no roads
      // within this radius), routing collapses to a 1-3 node stub. Report that as a
      // failure rather than returning a non-loop as success.
      //
      // Explicit-via mode also bypasses the strict node/length floors: a short
      // one-via route may produce fewer than MIN_ROUNDTRIP_LOOP_NODES if the
      // via is right next to the start. We still reject null/no-track outcomes
      // below as a safety net.
      if (foundTrack == null || foundTrack.nodes == null
          || (!explicitViaMode && (foundTrack.nodes.size() < MIN_ROUNDTRIP_LOOP_NODES
                                || foundTrack.distance < MIN_ROUNDTRIP_LOOP_METERS))) {
        int n = (foundTrack == null || foundTrack.nodes == null) ? 0 : foundTrack.nodes.size();
        int d = foundTrack == null ? 0 : foundTrack.distance;
        errorMessage = "round-trip could not form a loop for direction " + (int) direction
          + " at radius " + (int) searchRadius + "m (only " + n + " nodes, " + d
          + "m) — no reachable roads in that direction at this distance";
        logInfo(errorMessage);
        lastRejectedTrack = foundTrack; // preserve stub for post-mortem
        foundTrack = null;
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
      RoundTripQualityResult quality = boundedGateVerdict != null
        ? boundedGateVerdict
        : evaluateRoundTripGate(foundTrack, searchRadius, explicitViaMode);
      boundedGateVerdict = null;
      if (!quality.isAccepted()) {
        // STRUCTURAL failures (broken / un-routable / not-a-loop) are always
        // hard-rejected — there is nothing usable to offer. QUALITY failures
        // (distance off-target, self-crossing/hairpin chaos, hostile surface,
        // mid-route backtracking) are advisory by default: the route is
        // rideable, so we return it with a Warning and let the user decide.
        // roundTripStrictQuality=1 restores the old hard-reject behaviour.
        boolean hardReject = roundTripQualityHardReject(quality);
        if (hardReject) {
          errorMessage = "round-trip rejected by quality gate (direction " + (int) direction
            + ", radius " + (int) searchRadius + "m, shape=" + quality.getShape() + "): "
            + quality.getRejectionReason();
          logInfo(errorMessage);
          lastRejectedTrack = foundTrack;
          foundTrack = null;
          return;
        }
        // Lenient default: surface the quality issue as a warning and keep the
        // route. The planner already searched strictly and shipped its best
        // effort; we disclose the problem rather than discard a rideable loop.
        String advisory = "Warning: " + quality.getRejectionReason()
          + " (shape=" + quality.getShape() + ") — route returned anyway; ride at your"
          + " discretion, or set roundTripStrictQuality=1 to reject it.";
        logInfo("round-trip quality advisory (lenient): " + advisory);
        appendRouteMessage(foundTrack, advisory);
        // fall through to disclosure surfacing + success
      }
      // Surface the route shape + disclosures (e.g. "contains retraced
      // scenic spur: 4.2km") so the cyclist isn't surprised to find
      // they're returning the same way along a stretch. Stays in the
      // route message stream so it propagates to GPX/JSON exports.
      logInfo("round-trip quality: " + quality);
      for (String d : quality.getDisclosures()) {
        appendRouteMessage(foundTrack, d);
      }

      // Transparency for the silent band: 1..MAX crossings and guard-blocked
      // spurs pass the gate without any message, yet the cyclist sees them on
      // the map. Disclose every nonzero count — informational only, the route
      // ships either way (lenient product policy: odd-but-cycleable > nothing).
      // The whole decoration block runs under its own guard: the loop is
      // complete and gate-accepted at this point, and the outer catch nulls
      // foundTrack — an exception in a cosmetic advisory must never destroy
      // a rideable result.
      try {
        int shippedCrossings = RoundTripQualityGate.countSelfIntersections(foundTrack);
        if (shippedCrossings > 0) {
          appendRouteMessage(foundTrack, String.format(Locale.US,
            "Note: route crosses its own path %d time%s.",
            shippedCrossings, shippedCrossings == 1 ? "" : "s"));
        }
        if (foundTrack.nodes != null) {
          int[] spurInfo = LoopQualityMetrics.computeSpurInfo(foundTrack.nodes);
          if (spurInfo[0] > 0 && spurInfo[1] > 600) {
            appendRouteMessage(foundTrack, String.format(Locale.US,
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
        int chordMeters = LoopQualityMetrics.maxSingleNullEdgeMeters(foundTrack);
        if (chordMeters > GreedyRoundTripPlanner.MAX_UNDETAILED_EDGE_METERS) {
          appendRouteMessage(foundTrack, String.format(Locale.US,
            "Note: route contains an undetailed straight-line section of ~%dm "
              + "(way detail missing in the map data; the actual road may curve).",
            chordMeters));
        }

        // Soft advisory: even within the [0.5, 1.8] ratio band, a >1.5
        // overshoot is worth flagging so the caller can suggest a shorter
        // distance. This stays informational because the hard gate above
        // already rejects ratios outside the safe range.
        if (foundTrack.distance > 0) {
          double ratio = foundTrack.distance / expectedDistance;
          if (ratio > 1.5) {
            String warning = String.format(
              "Warning: route distance (%dkm) exceeds requested loop distance (%dkm) by %.0f%%. "
              + "The road network in this area is too constrained for a compact loop at this distance. "
              + "Consider a shorter distance or an out-and-back route.",
              foundTrack.distance / 1000, (int) (expectedDistance / 1000), (ratio - 1) * 100);
            logInfo(warning);
            appendRouteMessage(foundTrack, warning);
          }
        }

        // The advisory/disclosures above were appended to foundTrack.message, but
        // FormatGpx emits <brouter:info> and its message comments from
        // messageList, not message. Sync messageList[0] so the quality warning
        // actually reaches GPX/JSON consumers. Idempotent; no-op for the AUTO
        // path (which returns earlier and syncs via adoptCandidateWinner).
        trackCleanup().ensureInfoMessage(foundTrack);
      } catch (RuntimeException advisoryFailure) {
        logInfo("round-trip advisory decoration failed ("
          + advisoryFailure.getClass().getSimpleName()
          + "); returning the track without advisories");
        logThrowable(advisoryFailure);
      }

      long endTime = System.currentTimeMillis();
      logInfo("round trip execution time = " + (endTime - startTime) / 1000. + " seconds");
    } catch (Exception e) {
      logException(e);
      logThrowable(e);
      // Contract: a round trip ends with a usable track XOR a clean error. An
      // exception can land here before any assignment, leaving foundTrack as
      // the constructor's initial empty OsmTrack (or a partial one) — and
      // logException copies e.getMessage(), which is null for message-less
      // exceptions. Guarantee both halves of the contract: a non-empty error
      // and no degenerate "success" track. Non-empty geometry is preserved on
      // lastRejectedTrack for post-mortem inspection like other reject paths.
      if (errorMessage == null || errorMessage.isEmpty()) {
        errorMessage = "round trip failed: " + e.getClass().getSimpleName();
      }
      if (foundTrack != null && foundTrack.nodes != null && !foundTrack.nodes.isEmpty()) {
        lastRejectedTrack = foundTrack;
      }
      foundTrack = null;
    } finally {
      cleanupRoutingResources();
    }

  }

  /**
   * Append a one-line message to {@code track.message}, space-separated.
   * Used to surface advisories and quality-gate disclosures so that
   * downstream GPX/JSON formatters carry the information to the cyclist.
   * No-op if either argument is null/empty.
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
   * The uniform round-trip gate evaluation — single source of truth for the
   * gate flags, shared by the verdict in {@code doRoundTrip} and the bounded
   * tier's fallback decision so the two sites can never drift.
   *
   * <p>Explicit-via mode treats the requested distance as advisory only — the
   * user-supplied skeleton defines the route, not the distance target; the
   * gate still enforces beeline / closure / profile-hostility checks. A forced
   * same-way-back corridor (planner found nothing clean in constrained
   * terrain) is accepted as a disclosed OUT_AND_BACK rather than rejected —
   * keep-when-forced; gratuitous corridors never reach here because the
   * planner only sets the flag when no clean alternative exists.
   */
  private RoundTripQualityResult evaluateRoundTripGate(OsmTrack track, double searchRadius,
                                                       boolean explicitViaMode) {
    return evaluateRoundTripGate(track, searchRadius, explicitViaMode,
      roundTripForcedCorridorAccepted);
  }

  /** Overload for verdicts on a CANDIDATE's track, whose forced-corridor
   *  marker lives on the candidate rather than the engine field. */
  private RoundTripQualityResult evaluateRoundTripGate(OsmTrack track, double searchRadius,
                                                       boolean explicitViaMode,
                                                       boolean forcedCorridorAccepted) {
    boolean allowSamewayback = routingContext.allowSamewayback || forcedCorridorAccepted;
    return RoundTripQualityGate.evaluate(track, 2 * Math.PI * searchRadius,
      routingContext.getProfileName(), allowSamewayback, explicitViaMode,
      roundTripFerriesAllowed());
  }

  /** Bounded-tier verdict handoff: set by doBoundedRoundTrip when its
   *  pre-gate accepted the planner track, consumed (once) by the shared gate
   *  in doRoundTrip so the same track is not fully evaluated twice. */
  private RoundTripQualityResult boundedGateVerdict;

  /**
   * Single source of truth for the round-trip lenient/strict policy: whether a
   * non-accepted quality verdict must be hard-rejected rather than returned with
   * an advisory. STRUCTURAL failures (broken / un-routable / not-a-loop) are
   * always hard-rejected; QUALITY failures (rideable but suboptimal) are
   * advisory by default and hard-rejected only in strict mode
   * ({@link RoutingContext#roundTripStrictQuality}). Used by the gate path and
   * the AUTO best-effort fallback so the two never drift apart.
   */
  private boolean roundTripQualityHardReject(RoundTripQualityResult quality) {
    return quality.getRejectionTier() != RoundTripQualityResult.RejectionTier.QUALITY
      || routingContext.roundTripStrictQuality;
  }

  /**
   * Keep the round-trip lifecycle equivalent to the normal {@link #doRouting(long)}
   * finally block: release the parsed profile, close any cache/log resources,
   * clear search state, and signal {@link #isFinished()}.
   *
   * <p>This is intentionally idempotent. Some round-trip paths delegate through
   * {@code doRouting()}, which already performs the same cleanup, while direct
   * planner-track adoption paths never enter {@code doRouting()} at all.
   */
  private void cleanupRoutingResources() {
    if (hasInfo() && routingContext.expctxWay != null) {
      logInfo("expression cache stats=" + routingContext.expctxWay.cacheStats());
    }
    ProfileCache.releaseProfile(routingContext);
    if (nodesCache != null) {
      if (hasInfo()) {
        logInfo("NodesCache status before close=" + nodesCache.formatStatus());
      }
      nodesCache.close();
      nodesCache = null;
    }
    openSet.clear();
    finished = true;

    if (infoLogWriter != null) {
      try {
        infoLogWriter.close();
      } catch (Exception e) {
      }
      infoLogWriter = null;
    }
    if (stackSampler != null) {
      try {
        stackSampler.close();
      } catch (Exception e) {
      }
      stackSampler = null;
    }
  }

  static RoundTripAlgorithm selectRoundTripAlgorithm(double searchRadius) {
    // Cheap fallback selector. The full AUTO policy lives in
    // {@link #runAutoCandidateCompetition}; this helper remains as a stable
    // entry point for direct callers and unsupported AUTO modes.
    return RoundTripAlgorithm.GREEDY;
  }

  /**
   * AUTO candidate competition for generated round trips (no user vias).
   *
   * <p>Runs greedy candidates first and uses the old probe/WAYPOINT generator
   * only as a fallback:
   * <ol>
   *   <li>ISO_GREEDY — iso-derived candidates fed to the greedy planner.
   *       Profile-aware by construction.</li>
   *   <li>GREEDY — plain greedy graph-native/top-k planner if ISO_GREEDY fails or is weak.</li>
   *   <li>WAYPOINT/probe — legacy fallback only if greedy produced no accepted route.</li>
   * </ol>
   *
   * <p>Each candidate runs inside an isolated <em>child</em> {@link RoutingEngine}
   * built from a request-fields-only copy of the parent's
   * {@link RoutingContext} — no parsed/runtime state is shared. Child output
   * is suppressed (no GPX/log written). After all candidates have run, the
   * highest-scoring accepted candidate's {@link OsmTrack} is adopted as
   * this engine's {@code foundTrack} and its disclosures are surfaced.
   *
   * <p>If no candidate passes strict validation, the lenient default adopts the
   * least-bad QUALITY-tier best-effort track (see {@link #selectBestEffortCandidate});
   * strict mode instead leaves {@code foundTrack} null and sets {@code errorMessage}.
   */
  private void runAutoCandidateCompetition(double searchRadius, double direction) {
    long t0 = System.currentTimeMillis();
    // One wall-clock budget shared across the sequentially-run candidates, so
    // the competition cannot run ~Nx the requested timeout. Each child gets the
    // remaining slice (see runChildCandidate); once it is exhausted we stop
    // spawning further candidates.
    long deadline = t0 + (maxRunningTime > 0 ? maxRunningTime : DEFAULT_AUTO_BUDGET_MS);
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
    boolean greedyNeeded = roundTripEffortPolicy.runGreedyAlways
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
          logInfo("AUTO: GREEDY child overstayed its budget; terminating");
          if (greedyChild != null) {
            greedyChild.terminate();
          }
          greedyThread.join(AUTO_CHILD_JOIN_UNWIND_MS);
        }
      } catch (InterruptedException ie) {
        currentThread().interrupt();
      }
      // If the (needed) child is STILL alive its result slot is not safely
      // published — treat it as no candidate rather than reading a
      // half-written result. The daemon thread cannot block JVM exit.
      if (greedyThread.isAlive()) {
        greedyNeeded = false;
        greedyResultIgnored = true;
        logInfo("AUTO: GREEDY child did not stop in time; ignoring its result");
      }
    }
    results.add(isoGreedyR);
    logInfo("AUTO candidate: " + isoGreedyR);

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
      logInfo("AUTO candidate: " + parallel[1]);
    } else if (greedyThread != null && !greedyResultIgnored) {
      logInfo("AUTO: speculative GREEDY child discarded ("
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
      logInfo("AUTO candidate: " + waypointR);
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
      logInfo("AUTO candidate: " + isochroneR);
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
            && !roundTripQualityHardReject(r.gateVerdict)) {
          bestEffort.add(r);
        }
      }
      winner = selectBestEffortCandidate(bestEffort, 2 * Math.PI * searchRadius,
        routingContext.getProfileName(), direction);
      if (winner != null) {
        logInfo("AUTO: no strictly-accepted route; adopting best-effort " + winner.algorithm
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
      errorMessage = "AUTO competition produced no acceptable route "
        + "(tried " + results.size() + " candidates in " + totalMs + "ms): "
        + (err == null ? "unknown" : err);
      logInfo(errorMessage);
      // Surface the best-geometry rejected candidate for post-mortem inspection,
      // mirroring the direct-dispatch path which sets lastRejectedTrack before
      // nulling foundTrack. Candidates are in algorithm-quality order, so the
      // first with a track is the best available rejected geometry.
      for (RoundTripCandidateResult r : results) {
        if (r.track != null) {
          lastRejectedTrack = r.track;
          break;
        }
      }
      foundTrack = null;
      return;
    }
    adoptCandidateWinner(winner, results, totalMs);
  }

  /**
   * AUTO's plain-GREEDY entitlement check.
   *
   * <p>Issue #26 absorption rule: a below-threshold ISO_GREEDY result does not
   * automatically imply a useful second plain-GREEDY run. If ISO_GREEDY's
   * provider fell back to graph-native candidates before planning, or if
   * ISO_GREEDY already compared an internal graph-native-only branch, it has
   * already used the same source truth as plain GREEDY. Running GREEDY again is
   * duplicate work, not extra source truth.
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
   * A blended verdict below the clear-accept bar (or none) warrants the
   * internal graph-native comparison. The trigger and the selection both work
   * on verdicts from {@link #scoreInternalGreedyResult}, so they can never
   * judge a track differently — the flags drifted once (ferries hard-coded
   * off in a separate trigger pipeline) and every ferry-using accepted loop
   * paid a full spurious extra ladder. Package-visible so the trigger tests
   * exercise the same predicate production calls.
   */
  static boolean internalBranchNeeded(RouteChoiceScore.Verdict blendedVerdict) {
    return blendedVerdict == null || blendedVerdict.score() < CLEAR_ACCEPT_THRESHOLD;
  }

  /**
   * Selection between the blended result and the internal graph-native branch,
   * on verdicts computed ONCE at the call site (each full-track gate+score
   * pass rebuilds the crossing grid and corridor index — two per comparison
   * suffice, not four).
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

  private RouteChoiceScore.Verdict scoreInternalGreedyResult(RoundTripResult result,
                                                            double desiredDistance,
                                                            double direction) {
    return scoreInternalGreedyResult(result, desiredDistance,
      routingContext.getProfileName(), direction,
      routingContext.allowSamewayback, roundTripFerriesAllowed());
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

  private static boolean isoGreedyAbsorbedGraphNativeTruth(RoundTripCandidateResult isoGreedyR) {
    // The child's explicit start-policy decision: a graph-native-only plan
    // already used the same candidate source as plain GREEDY, so a separate
    // GREEDY child would duplicate it. (This used to be inferred from three
    // telemetry sentinels — no iso legs + some non-iso legs + NaN health —
    // which any telemetry-semantics change could silently flip.)
    return isoGreedyR.algorithm == RoundTripAlgorithm.ISO_GREEDY
      && isoGreedyR.graphNativeOnlyStart();
  }

  /**
   * Rank degraded best-effort round-trip candidates and return the most rideable,
   * or {@code null} if none have a track. Scores each with the multi-factor
   * {@link RouteChoiceScore#scoreBestEffort}, which bypasses the scorer's
   * accepted-only zero-guard (so a rejected track is ranked on its real geometry
   * instead of collapsing to 0) while still consuming the candidate's gate
   * verdict for the shape disclosure penalty — a rejected LOLLIPOP/OUT_AND_BACK
   * must not rank as if it were a strict loop. Because every QUALITY failure
   * also corresponds to a weak component in the score (distance miss → low
   * distance term, hostile surface → low cost/m term, chaos/retrace → low reuse
   * term), the least-bad overall candidate wins. Ties keep {@code candidates}
   * order (the AUTO algorithm-quality order). Does no routing — the tracks are
   * already built.
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
   * Budget (ms) for the next sequential AUTO candidate: the time remaining to
   * the shared competition deadline, floored at {@link #MIN_CHILD_BUDGET_MS} so
   * a candidate that is still spawned gets a usable slice rather than ~0.
   */
  static long childCandidateBudgetMs(long deadline, long now) {
    return Math.max(MIN_CHILD_BUDGET_MS, deadline - now);
  }

  /**
   * Run one AUTO candidate in an isolated child engine, score the result,
   * and return the wrapper. Never throws — failures land in
   * {@link RoundTripCandidateResult#errorMessage}.
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline) {
    return runChildCandidate(algo, searchRadius, direction, deadline, null);
  }

  /**
   * As above, additionally publishing the child engine into
   * {@code engineOut[0]} as soon as it is constructed, so a concurrent
   * coordinator can {@link #terminate()} a speculative child whose result is
   * no longer needed (the volatile kill flag is honoured per search pop and
   * per expansion pop).
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline,
                                                     java.util.concurrent.atomic.AtomicReference<RoutingEngine> engineOut) {
    long t0 = System.currentTimeMillis();
    RoundTripCandidateResult r = new RoundTripCandidateResult(algo);
    try {
      RoutingContext childCtx = routingContext.copyRequestFields();
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
      List<OsmNodeNamed> childWps = new ArrayList<>(waypoints.size());
      for (OsmNodeNamed wp : waypoints) {
        OsmNodeNamed copy = new OsmNodeNamed(new OsmNode(wp.ilon, wp.ilat));
        copy.name = wp.name;
        childWps.add(copy);
      }
      // Output suppressed (null outfileBase). Child runs its own pipeline
      // including post-routing checks + quality gate; we just inspect the
      // result.
      RoutingEngine child = new RoutingEngine(null, null, segmentDir, childWps, childCtx,
        BROUTER_ENGINEMODE_ROUNDTRIP);
      child.quite = true;
      // The child plans with the parent's resolved effort (QUALITY's raised
      // top-K / plan budget must reach the planners it spawns).
      child.roundTripEffortPolicy = roundTripEffortPolicy;
      if (engineOut != null) {
        engineOut.set(child);
      }
      // Give the child only the remaining shared budget (floored so a spawned
      // candidate still gets a usable slice), not the full request timeout.
      long budget = childCandidateBudgetMs(deadline, System.currentTimeMillis());
      child.doRun(budget);
      r.track = child.foundTrack;
      r.errorMessage = child.errorMessage;
      r.runtimeMillis = System.currentTimeMillis() - t0;
      // Aggregate the child's expansion work into the parent so
      // getLinksProcessed() reports request-level totals (the perf budget
      // suite's work metric). Same-thread for sequential children; the
      // speculative parallel child is joined before its result is read.
      addLinksProcessed(child.linksProcessed);
      // All winner-attribution telemetry (incl. the keep-when-forced marker
      // the re-gate below honors) reads through this reference — no
      // field-by-field copy to forget when RoundTripResult grows.
      r.planner = child.lastRoundTripResult;

      if (r.track != null) {
        // Score against the parent's expected loop distance. This produces
        // a verdict that may differ from the child's internal gate result
        // because the parent's routingContext is the source of truth (e.g.
        // for profile-name lookup), but in practice both agree.
        double expectedDist = 2 * Math.PI * searchRadius;
        String profileName = routingContext.getProfileName();
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
      logThrowable(e);
      r.errorMessage = "candidate " + algo + " threw: " + e.getClass().getSimpleName()
        + (e.getMessage() == null ? "" : ": " + e.getMessage());
      r.runtimeMillis = System.currentTimeMillis() - t0;
    }
    return r;
  }

  /**
   * Adopt the winning candidate's track as this engine's result, attach
   * a summary diagnostic listing what was tried and which won.
   */
  private void adoptCandidateWinner(RoundTripCandidateResult winner,
                                    List<RoundTripCandidateResult> all, long totalMs) {
    foundTrack = winner.track;
    errorMessage = null;
    trackCleanup().finalizeAdoptedRoundTripTrack(foundTrack, foundTrack == null ? null : foundTrack.matchedWaypoints);
    // Best-effort (quality-failed) winner adopted under lenient mode: make sure
    // the user-facing quality Warning is present. The child engine usually
    // attaches it, but when the parent's gate re-evaluation in runChildCandidate
    // disagrees with the child's own verdict the child may not have — so attach
    // it here if absent, mirroring the direct-dispatch advisory (and skip when a
    // "Warning:" is already present to avoid a duplicate).
    if (foundTrack != null && !winner.accepted() && winner.gateVerdict != null
        && (foundTrack.message == null || !foundTrack.message.contains("Warning:"))) {
      appendRouteMessage(foundTrack, "Warning: " + winner.gateVerdict.getRejectionReason()
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
    if (foundTrack != null) {
      // foundTrack is nullable here (a best-effort winner can carry no track —
      // see the null-guards above at adoption and the warning block); only
      // attach the AUTO summary when there is a track to annotate.
      if (foundTrack.message == null || foundTrack.message.isEmpty()) {
        foundTrack.message = summary.toString();
      } else {
        foundTrack.message += " " + summary.toString();
      }
    }
    // Keep messageList.get(0) in sync with the just-extended message so the
    // GPX <brouter:info> / comment block reflects the AUTO summary too.
    trackCleanup().ensureInfoMessage(foundTrack);
    logInfo(summary.toString());
    if (winner.score != null) {
      logInfo("AUTO winner score breakdown:\n" + winner.score.describe());
    }
    // Format + persist the adopted track if the caller asked for an
    // output file. The child engines ran with null outfileBase (output
    // suppressed); the parent does the single final write.
    writeAdoptedTrackOutput(foundTrack);
  }

  /**
   * Format and persist a candidate-adopted track in the configured
   * {@code outputFormat}. When {@link #outfileBase} is null, keep the formatted
   * output in {@link #outputMessage} and print it unless {@link #quite} is set.
   * Mirrors the per-iteration write logic from {@link #doRouting} so the
   * AUTO-competition path produces the same output artefacts as the direct
   * algorithm dispatch.
   */
  private void writeAdoptedTrackOutput(OsmTrack track) {
    if (track == null) return;
    if (track.name == null) {
      track.name = "brouter_" + routingContext.getProfileName() + "_0";
    }
    track.exportWaypoints = routingContext.exportWaypoints;
    track.exportCorrectedWaypoints = routingContext.exportCorrectedWaypoints;
    String output;
    try {
      switch (routingContext.outputFormat) {
        case "gpx":     output = new FormatGpx(routingContext).format(track); break;
        case "geojson":
        case "json":
          output = new FormatJson(routingContext).format(track);
          break;
        case "kml":     output = new FormatKml(routingContext).format(track); break;
        case "csv":     output = null; break;
        default:        output = null;
      }
      outputMessage = output;
      if (outfileBase == null) {
        if (!quite && output != null) {
          System.out.println(output);
        }
        return;
      }
      String filename = outfileBase + "0." + routingContext.outputFormat;
      if ("csv".equals(routingContext.outputFormat)) {
        new FormatCsv(routingContext).write(filename, track);
      }
      if (output != null) {
        try (FileWriter fw = new FileWriter(filename)) {
          fw.write(output);
        }
      }
      outfile = filename;
      alternativeIndex = 0;
    } catch (Exception e) {
      logInfo("AUTO: failed to write adopted track: " + e.getClass().getSimpleName()
        + (e.getMessage() == null ? "" : ": " + e.getMessage()));
    }
  }

  /**
   * Whether greedy round-trip planning can be applied with the given inputs.
   * Greedy currently generates its own intermediate waypoints from the start,
   * so user-supplied via points and allowSamewayback are not honored.
   */
  static boolean greedySupports(boolean allowSamewayback, int waypointCount) {
    return !allowSamewayback && waypointCount <= 1;
  }

  /**
   * Explicit-via round-trip: the caller supplied via points; route through
   * them exactly, in input order, with no generated {@code rt*} waypoints.
   *
   * <p>Routing skeleton:
   * <ul>
   *   <li>{@code allowSamewayback=false}: {@code start → via1 → ... → viaN → start}</li>
   *   <li>{@code allowSamewayback=true}:  {@code start → via1 → ... → viaN → viaN-1 → ... → via1 → start}
   *       (mirroring is applied by the existing {@code doRouting} expansion;
   *       this helper only supplies the forward chain).</li>
   * </ul>
   *
   * <p>{@code roundTripPoints} is ignored. {@code roundTripDistance} /
   * {@code roundTripLength} become advisory targets only — the quality gate
   * runs in explicit-via mode and converts distance-ratio mismatch to a
   * disclosure rather than a rejection. {@code startDirection} is logged but
   * does not influence the via order.
   *
   * <p>A user via that cannot be snapped within range fails with an error
   * naming the via (the no-beeline invariant is preserved). The helper never
   * silently drops a user via.
   *
   * @param searchRadius used only to size the snap tolerance and for logging
   * @param direction    logged for diagnostics; not used to reorder vias
   */
  private void doExplicitViaRoundTrip(double searchRadius, double direction) {
    OsmNodeNamed start = waypoints.get(0);
    List<OsmNodeNamed> userVias = new ArrayList<>(waypoints.subList(1, waypoints.size()));
    waypoints.subList(1, waypoints.size()).clear();
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
    // Note: `waypointSnapper().snapStartToRoad(waypoints, ...)` short-circuits when
    // waypoints.size() < 2, so we snap the start directly via the
    // single-waypoint helper to avoid that early-return.
    double userSnapDist = Math.min(searchRadius * 0.3, 2000);
    waypointSnapper().snapStartProfileAware(start, userSnapDist);
    List<Boolean> matched = waypointSnapper().snapWaypointsToRoad(userVias, userSnapDist, "snapUserVia");
    for (int i = 0; i < userVias.size(); i++) {
      if (!matched.get(i)) {
        throw new IllegalArgumentException("user waypoint " + userVias.get(i).name
          + " has no road within " + (int) userSnapDist + "m");
      }
    }
    // Densification gate (ship A gated). OFF by default: inserting generated bulge points
    // would violate the user-via skeleton contract (no generated waypoints, order preserved),
    // so it must be explicitly opted into ({@code roundTripDensify=1} →
    // {@code explicitViaDensifyOverride=TRUE}) — a "length-honoring loop" mode. Even when opted
    // in it is gated to NON-PAVED profiles: for a road bike in sparse terrain a retracing paved
    // lollipop beats a one-way track loop the quality gate would reject, so paved keeps the
    // plain route.
    routingContext.explicitViaDensify =
      Boolean.TRUE.equals(routingContext.explicitViaDensifyOverride)
        && !RoundTripQualityGate.isPavedProfile(routingContext.getProfileName());

    // Anchor cycle [start, via1, ..., viaN]. With densification on, insert generated
    // arc-following "bulge" points between consecutive anchors so legs follow the loop
    // perimeter instead of cutting the chord (corner-cut undershoot fix).
    List<OsmNodeNamed> anchors = new ArrayList<>();
    anchors.add(start);
    anchors.addAll(userVias);

    waypoints.clear();
    if (routingContext.explicitViaDensify && !routingContext.allowSamewayback && anchors.size() >= 2) {
      waypoints.addAll(waypointSnapper().densifyViaArcs(anchors, searchRadius, userSnapDist));
    } else {
      waypoints.addAll(anchors);
    }

    // For allowSamewayback=false append the closing start copy so the route
    // forms a closed loop. For allowSamewayback=true the existing doRouting
    // expansion at the top of {@link #doRouting} mirrors the chain back —
    // we must NOT add a closing copy here or we'd double-close.
    if (!routingContext.allowSamewayback) {
      OsmNodeNamed closing = new OsmNodeNamed(new OsmNode(start.ilon, start.ilat));
      closing.name = "to";
      waypoints.add(closing);
    }

    routingContext.waypointCatchingRange = 250;
    roundTripSearchRadius = searchRadius;
    explicitViaRoundTrip = true;
    logInfo("explicit-via round-trip: " + userVias.size() + " user via(s), "
      + "allowSamewayback=" + routingContext.allowSamewayback
      + ", direction=" + (int) direction + " (advisory only)");
    doRouting(roundTripRoutingBudgetMs);
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

  private static final java.util.concurrent.atomic.AtomicLongArray PLACEMENT_PATH_COUNTS =
    new java.util.concurrent.atomic.AtomicLongArray(PlacementPath.values().length);

  private void recordPlacementPath(PlacementPath path) {
    PLACEMENT_PATH_COUNTS.incrementAndGet(path.ordinal());
    logInfo("roundtrip placement path: " + path); // no-op for quite child engines
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
   * Merge isochrone frontier data with probe directions for gap-filling.
   * Isochrone entries are 6-element {@code [direction, airDist, cost, hits,
   * ilon, ilat]} (the last two carry the road-native frontier coord);
   * probe-only entries are 4-element {@code [direction, searchRadius,
   * searchRadius*1.3, 0]} (estimated cost, no road-native data, hits=0).
   * Existing isochrone entries dominate overlapping probe directions.
   *
   * @param frontier        isochrone entries (may be null)
   * @param probeDirections probe viable directions in degrees (may be null)
   * @param searchRadius    fallback distance for probe-only directions
   * @return merged frontier entries; {@code null} if both inputs empty.
   *         Entry length varies: 6 for isochrone-sourced, 4 for probe-only.
   */
  static double[][] mergeIsochroneWithProbe(double[][] frontier, double[] probeDirections, double searchRadius) {
    Map<Integer, double[]> merged = new HashMap<>();

    if (frontier != null) {
      for (double[] entry : frontier) {
        int bucket = (int) Math.round(entry[0]);
        merged.put(bucket, entry);
      }
    }

    // Add probe directions where isochrone has no data
    if (probeDirections != null) {
      for (double dir : probeDirections) {
        int bucket = (int) Math.round(dir);
        boolean covered = false;
        for (int key : merged.keySet()) {
          if (PlacementGeometry.angleDiff(key, bucket) <= 5) {
            covered = true;
            break;
          }
        }
        if (!covered) {
          // Probe-only: use searchRadius as airDist, estimate cost with default indirectness.
          // hits=0 marks this as "probed but not observed by isochrone" — lower confidence.
          merged.put(bucket, new double[]{dir, searchRadius, searchRadius * DEFAULT_PROBE_INDIRECTNESS, 0});
        }
      }
    }

    if (merged.isEmpty()) return null;

    List<double[]> result = new ArrayList<>(merged.values());
    Collections.sort(result, (a, b) -> Double.compare(a[0], b[0]));
    return result.toArray(new double[0][]);
  }

  /**
   * One bounded tier slice: the tier budget clamped to the remaining request
   * budget, floored at {@link #MIN_LADDER_RUNG_BUDGET_MS} — mirroring the
   * competition's childCandidateBudgetMs contract, a nearly-spent request
   * still funds ONE bounded run (a deliberate, bounded overrun: the caller
   * gets a loop instead of a guaranteed instant timeout). An untimed request
   * (deadline 0) gets the full tier budget.
   */
  private static long tierSliceMs(long tierBudgetMs, long requestDeadline, long now) {
    return Math.min(tierBudgetMs, requestDeadline == 0 ? tierBudgetMs
      : Math.max(requestDeadline - now, MIN_LADDER_RUNG_BUDGET_MS));
  }

  /**
   * Bounded-effort dispatch (issue #27): one bounded, graph-aware planning run
   * with predictable latency. Used by the BALANCED tier and by AUTO when the
   * effort policy resolves BOUNDED (constrained resources).
   *
   * <p>Policy: a single ISO_GREEDY dispatch (its internal graph-native
   * comparison branch from issue #26 stays available) under a hard
   * {@code min(request budget, tierBudget)} deadline and a reduced per-step
   * routed top-K. The Phase 2.1 axis retry and the ISO_GREEDY→GREEDY
   * recursion are skipped ({@link RoundTripEffortPolicy#skipRetryLayers});
   * a degraded-but-rideable planner loop is adopted best-effort for the
   * lenient gate to grade. When the planner produces no track at all — or a
   * track the uniform gate would hard-reject — the tier falls back to one
   * FAST/WAYPOINT attempt, run under a fresh tier budget slice, because
   * always returning some loop beats strict adherence to a single slice
   * (the fallback fires mostly in constrained terrain where the greedy run
   * burned its budget without closing a loop). With
   * {@code greedyCapable == false} (allowSamewayback requests) the planner
   * slice is skipped and only the budgeted fallback runs. The caller falls
   * through to the shared floors and quality gate in {@code doRoundTrip} —
   * this method never returns an ungated success.
   */
  private void doBoundedRoundTrip(double searchRadius, double direction,
                                  RoundTripEffortPolicy policy, String tierLabel,
                                  boolean greedyCapable) {
    long tierBudgetMs = policy.tierBudgetMs;
    long t0 = System.currentTimeMillis();
    long savedDeadline = roundTripRequestDeadline;
    long plannerMs = 0;
    if (!greedyCapable) {
      // Same constraint as the greedy dispatch: the planner generates its own
      // intermediate points and does not honor allowSamewayback. The waypoint
      // placement below still runs under the tier budget — bypassing the tier
      // would hand this input the full request budget.
      logInfo(tierLabel + ": planner does not support allowSamewayback,"
        + " using waypoint placement under the tier budget");
    } else {
      RoundTripEffortPolicy savedPolicy = roundTripEffortPolicy;
      long effectiveMs = tierSliceMs(tierBudgetMs, savedDeadline, t0);
      roundTripRequestDeadline = t0 + effectiveMs;
      roundTripEffortPolicy = policy;
      // The engine-level timers (island check, leg searches) run in THIS engine
      // and consult maxRunningTime — floor it to the slice too, or a nearly-
      // spent request budget times out the matching before the planner starts.
      // (The competition path achieves the same by flooring each child's doRun
      // budget.) 0 stays 0: an untimed request keeps engine timers off here;
      // the planner slice is still bounded by roundTripRequestDeadline.
      long savedMaxRunningTime = maxRunningTime;
      if (maxRunningTime > 0) {
        maxRunningTime = (t0 + effectiveMs) - startTime;
      }
      try {
        doGreedyRoundTrip(searchRadius, direction, RoundTripAlgorithm.ISO_GREEDY);
      } finally {
        roundTripEffortPolicy = savedPolicy;
        roundTripRequestDeadline = savedDeadline;
        maxRunningTime = savedMaxRunningTime;
      }
      plannerMs = System.currentTimeMillis() - t0;
      if (foundTrack != null) {
        // The bounded planner adopts degraded best-effort snapshots and defers
        // the verdict to the uniform gate in doRoundTrip. Take that verdict
        // now: a track the gate will hard-reject must not suppress the tier's
        // geometric fallback — by the time the shared gate nulls the track,
        // the chance to fall back is gone and the tier returns a hard error
        // instead of the loop it promises.
        // explicitViaMode == false by construction: the bounded tier is only
        // dispatched in generated-loop mode (the explicit-via skeleton
        // branches off before the tier dispatch).
        RoundTripQualityResult verdict = evaluateRoundTripGate(foundTrack, searchRadius, false);
        if (!verdict.isAccepted() && roundTripQualityHardReject(verdict)) {
          logInfo(tierLabel + ": bounded planner track fails the quality gate ("
            + verdict.getRejectionReason() + "); falling back to waypoint placement");
          lastRejectedTrack = foundTrack;
          foundTrack = null;
        } else {
          // The surviving track flows unchanged to the shared gate in
          // doRoundTrip — stash the verdict so that gate consumes it instead
          // of paying a second full-track evaluation (crossing grid, corridor
          // index) on every interactive bounded request. The fallback path
          // leaves this null: its track needs a fresh verdict.
          boundedGateVerdict = verdict;
        }
      }
      if (foundTrack == null) {
        logInfo(tierLabel + ": bounded planner produced no accepted loop in " + plannerMs
          + "ms (budget " + tierBudgetMs + "ms)"
          + (errorMessage == null ? "" : " — " + errorMessage)
          + "; falling back to waypoint placement");
      }
    }
    if (foundTrack == null) {
      errorMessage = null;
      // Fresh tier slice for the fallback (see method javadoc). Worst case is
      // two slices; the request-level watchdog still applies on top. Same
      // minimum-slice floor as above so a spent budget still funds the one
      // cheap geometric attempt.
      long fallbackStart = System.currentTimeMillis();
      long fallbackMs = tierSliceMs(tierBudgetMs, savedDeadline, fallbackStart);
      roundTripRequestDeadline = fallbackStart + fallbackMs;
      long savedRoutingBudget = roundTripRoutingBudgetMs;
      long savedMaxRunningTime = maxRunningTime;
      // Scope the engine timers to the fallback slice, UNCONDITIONALLY. The
      // placement phase (probing + the islanded-via guard) runs before
      // doRouting re-arms startTime/maxRunningTime from the routing budget:
      // under the request-scoped timer a spent budget makes every placement
      // engine call throw instantly — the island guard degrades to
      // keep-every-via and routing then dies on "target island detected" —
      // and an untimed request (all timer fields 0) would run the fallback
      // with no bound at all. Both violate the tier's slice contract.
      roundTripRoutingBudgetMs = fallbackMs;
      maxRunningTime = (fallbackStart + fallbackMs) - startTime;
      try {
        doWaypointBasedRoundTrip(searchRadius, direction, RoundTripAlgorithm.WAYPOINT);
      } finally {
        roundTripRequestDeadline = savedDeadline;
        roundTripRoutingBudgetMs = savedRoutingBudget;
        maxRunningTime = savedMaxRunningTime;
      }
      if (foundTrack != null) {
        // The shipped track came from the waypoint fallback, not the planner —
        // keeping the FAILED planner result would attribute its counters and
        // pool-health telemetry to a loop the planner never produced.
        lastRoundTripResult = null;
      }
    }
    logInfo(tierLabel + ": finished in " + (System.currentTimeMillis() - t0)
      + "ms (planner " + plannerMs + "ms, budget " + tierBudgetMs + "ms/slice, "
      + (foundTrack == null ? "no track" : "track " + foundTrack.distance + "m") + ")");
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
    int varietySeed = routingContext.getRoundTripSeed();
    if (varietySeed > 0) {
      double phaseShiftDeg = 15.0 * GreedyRoundTripPlanner.seededUnit(varietySeed, 1, 0);
      double radiusScale = 1.0 + 0.03 * GreedyRoundTripPlanner.seededUnit(varietySeed, 2, 0);
      direction = CheapAngleMeter.normalize(direction + phaseShiftDeg);
      searchRadius *= radiusScale;
      logInfo("round trip variety seed " + varietySeed + ": phase shift " + (int) phaseShiftDeg
        + " deg, radius scale " + radiusScale);
    }
    if (routingContext.allowSamewayback) {
      int[] pos = CheapRuler.destination(waypoints.get(0).ilon, waypoints.get(0).ilat, searchRadius, direction);
      OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
      onn.name = "rt1";
      waypoints.add(onn);
      // No-beeline invariant: snap the tip before final matchWaypointsToNodes.
      // On snap failure the tip stays at the raw geometric point and the return
      // leg can degrade to a straight-line beeline — surface it rather than
      // silently discarding the result (cf. snapWaypointsToRoad for user vias).
      if (!waypointSnapper().snapWaypointToRoad(onn, Math.min(searchRadius * 0.3, 2000), "snapSamewaybackTip")) {
        logInfo("snapSamewaybackTip: no road within snap range; samewayback return leg may include a beeline");
      }
    } else {
      // INVARIANT: this branch runs only in non-explicit-via mode, which is
      // reached only when waypoints.size() == 1 (user vias are handled earlier by
      // doExplicitViaRoundTrip). Fail fast if a future refactor ever routes user
      // vias here, rather than silently re-running the old bearing-sorted via
      // injection that doExplicitViaRoundTrip was built to replace.
      if (waypoints.size() > 1) {
        throw new IllegalStateException(
          "doWaypointBasedRoundTrip expects a single start waypoint; user vias must be "
            + "handled by doExplicitViaRoundTrip (got " + waypoints.size() + ")");
      }

      int targetPoints = routingContext.roundTripPoints == null ?
        Math.max(5, Math.min(15, (int) (searchRadius / 1500) + 3)) :
        routingContext.roundTripPoints;
      // Variety seed knob: ±1 via-point count, only when the count is derived —
      // an explicit roundTripPoints is a user decision the seed must not override.
      if (varietySeed > 0 && routingContext.roundTripPoints == null) {
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
        ProbeResult probe = waypointSnapper().probeReachableDirections(waypoints.get(0), searchRadius);
        double[] probeDirections = (probe != null) ? probe.viableDirections : null;
        IsochroneExpansionResult iso = runIsochroneExpansion(waypoints.get(0), searchRadius);
        double[][] frontier = (iso != null) ? iso.frontier : null;
        double[][] merged = mergeIsochroneWithProbe(frontier, probeDirections, searchRadius);
        if (merged != null && merged.length >= 3) {
          List<IsoCandidate> isoCandidates = (iso != null) ? iso.candidates : null;
          recordPlacementPath(PlacementPath.ISOCHRONE);
          placeWaypointsFromIsochrone(waypoints, merged, isoCandidates, searchRadius, direction, targetPoints);
        } else if (probeDirections != null && probeDirections.length >= 3) {
          logInfo("isochrone merge insufficient, falling back to probe directions");
          recordPlacementPath(PlacementPath.ENVELOPE_ISO_FALLBACK);
          placeWaypointsFromEnvelope(waypoints, probeDirections, searchRadius, direction, targetPoints);
        } else {
          logInfo("both isochrone and probe insufficient, falling back to circle");
          recordPlacementPath(PlacementPath.CIRCLE);
          buildPointsFromCircle(waypoints, direction, searchRadius, targetPoints);
        }
      } else if (fastOptimized) {
        // Directional lobe (opt-in): head the loop toward the requested bearing like
        // the pre-903 routine, instead of encircling the start. Off by default while
        // the sparse-terrain robustness (routing between forward-arc vias can fail
        // where an encircling ring would not) is finished via a post-routing retry.
        FastPlacementRequest fastRequest = new FastPlacementRequest(
          waypoints.get(0), searchRadius, direction, targetPoints,
          direction >= 0
            && "true".equals(System.getProperty("roundtrip.fast.directional", "false")),
          Integer.getInteger("roundtrip.fast.maxvias", 5));
        // Placement builds its skeleton on a local list and the outcome is
        // committed in one step — a degraded or failed attempt can never
        // leave partial vias in the live waypoint list.
        FastPlacementOutcome fastOutcome =
          new FastWaypointPlanner(fastPlacementOps()).place(fastRequest);
        waypoints.clear();
        waypoints.addAll(fastOutcome.skeleton);
        recordPlacementPath(fastOutcome.optimizedPlacement()
          ? PlacementPath.ENVELOPE_FAST : PlacementPath.CIRCLE);
      } else {
        ProbeResult probe = waypointSnapper().probeReachableDirections(waypoints.get(0), searchRadius);
        // FAST tier: drop single-probe-success directions when enough strong
        // alternatives exist. Avoids fragile sea-edge/dead-end picks.
        double[] viableDirections = PlacementGeometry.filterByProbeConfidence(probe, targetPoints);
        if (viableDirections != null && viableDirections.length >= 3) {
          recordPlacementPath(PlacementPath.ENVELOPE_FAST);
          placeWaypointsFromEnvelope(waypoints, viableDirections, searchRadius, direction, targetPoints);
        } else {
          logInfo("reachability probe returned < 3 directions, falling back to circle");
          recordPlacementPath(PlacementPath.CIRCLE);
          buildPointsFromCircle(waypoints, direction, searchRadius, targetPoints);
        }
      }

      // Idea 4: the optimized FAST module fully validates its own skeleton —
      // probe-snapped vias are pre-validated, and its circle fallback runs this
      // pass behind FastPlacementOps.circleFallbackValidated. Only the ISOCHRONE
      // and legacy A/B placements need the caller-side matching pass.
      if (algo == RoundTripAlgorithm.ISOCHRONE || !fastOptimized) {
        waypointSnapper().validateAndAdjustWaypoints(waypoints, searchRadius);
      }

      // Snap start/end waypoints to nearest road to prevent beeline segments.
      // Without this, if the user's click position is >250m from a road (park,
      // water, etc.), the routing engine inserts straight-line beelines.
      waypointSnapper().snapStartToRoad(waypoints, searchRadius);
    }

    routingContext.waypointCatchingRange = 250;
    roundTripSearchRadius = searchRadius;
    doRouting(roundTripRoutingBudgetMs);
  }

  static int selectGreedySubRouteCount(double desiredDistance, String profileName) {
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

  static int[] greedySubRouteCountPlan(int base) {
    return greedySubRouteCountPlan(base, IsoStartPolicy.BLEND);
  }

  static int[] greedySubRouteCountPlan(int base, IsoStartPolicy policy) {
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

  static IsoStartPolicy selectIsoStartPolicy(IsoPoolHealth.PoolShape poolShape) {
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

  private static void addUniqueCount(List<Integer> counts, int n) {
    if (n < 3 || n > 6 || counts.contains(n)) return;
    counts.add(n);
  }

  private static boolean isDegradedGreedyResult(RoundTripResult result) {
    return result != null
      && result.getFallbackReason() != null
      && result.getFallbackReason().startsWith(GreedyRoundTripPlanner.DEGRADED_FALLBACK_PREFIX);
  }

  /**
   * Build the appropriate candidate provider for the chosen mode. GREEDY uses
   * per-step graph-native candidates. ISO_GREEDY blends a bounded start-centered
   * isochrone pool with that same per-step graph-native provider. Geometric
   * radial placement is intentionally not used by production greedy paths.
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
      logInfo("ISO_GREEDY: insufficient isochrone data ("
        + (iso == null ? 0 : iso.frontier.length) + " buckets, "
        + (iso == null ? 0 : iso.candidates.size()) + " raw candidates), using graph-native candidates");
      return graphNative;
    }
    IsochroneCandidateProvider isoProvider =
      IsochroneCandidateProvider.fromPool(searchRadius, startDirection, iso.candidates);
    if (isoProvider.poolSize() < 6) {
      logInfo("ISO_GREEDY: candidate pool too small after filtering ("
        + isoProvider.poolSize() + "), using graph-native candidates");
      return graphNative;
    }
    if (!isoProvider.isDiverse()) {
      logInfo("ISO_GREEDY: candidate pool concentrated in a narrow corridor ("
        + isoProvider.poolSize() + " candidates), using graph-native candidates");
      return graphNative;
    }
    // ISO_GREEDY: blend start-centered iso depth with per-step graph-native
    // candidates. Both sources are road-native; neither invents coordinates.
    logInfo("ISO_GREEDY: blended isochrone+graph-native provider (iso pool="
      + isoProvider.poolSize() + ")");
    return new BlendedCandidateProvider(isoProvider, graphNative);
  }

  void doGreedyRoundTrip(double searchRadius, double direction, RoundTripAlgorithm algo) {
    // Initialize nodesCache — needed before the planner can match waypoints to the graph.
    resetCache(false);
    roundTripForcedCorridorAccepted = false;
    // Loop scale for the via-relocation bound (profileAwareMatchPoint): must be
    // set BEFORE planner via matching — the doRouting fallthrough below used to
    // set it only late, leaving the bound inert during greedy placement.
    roundTripSearchRadius = searchRadius;

    OsmNodeNamed start = waypoints.get(0);
    double desiredDistance = 2 * Math.PI * searchRadius;
    logInfo("greedy round trip: desired distance=" + (int) desiredDistance
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
      ? runIsochroneExpansion(start, searchRadius)
      : null;
    double effectiveDirection = direction;
    IsoAsymmetryBias bias = IsoAsymmetryBias.NONE;
    if (algo == RoundTripAlgorithm.ISO_GREEDY && direction < 0 && iso != null) {
      bias = computeIsoAsymmetryBearing(iso.frontier, searchRadius);
      if (bias.applied) {
        effectiveDirection = bias.bearingDegrees;
        logInfo("ISO_GREEDY: iso-asymmetry bias selected bearing="
          + (int) bias.bearingDegrees + "° (indirectness=" + String.format("%.2f", bias.indirectness)
          + ", hits=" + bias.hits + ", airDist=" + bias.airDistMeters + "m)");
      }
    }
    GraphNativeCandidateProvider graphNativeProvider = new GraphNativeCandidateProvider(roundTripOps());
    RoundTripCandidateProvider provider = buildCandidateProvider(algo, start, searchRadius,
      effectiveDirection, iso, graphNativeProvider);
    int baseSubRouteCount = selectGreedySubRouteCount(desiredDistance, routingContext.getProfileName());

    // Return-distance oracle (F6): sector-resolved return estimates from the
    // start-centered pool expansion when one exists (ISO_GREEDY — largest
    // coverage). Plain GREEDY deliberately has no oracle: a step-1 expansion
    // oracle was measured quality-negative, so null means the planner falls
    // back to the global-EMA estimate everywhere.
    ReturnDistanceOracle returnOracle = ReturnDistanceOracle.build(iso, start.ilon, start.ilat);
    if (returnOracle != null) {
      logInfo("greedy: return oracle from pool expansion (kappa="
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
      logInfo("ISO_GREEDY: iso-pool shape: " + poolShape.describe());
    }

    FrontierAxis frontierAxis = (algo == RoundTripAlgorithm.ISO_GREEDY && iso != null)
      ? computeFrontierAxis(iso.frontier, searchRadius) : FrontierAxis.NONE;
    IsoStartPolicy isoStartPolicy = algo == RoundTripAlgorithm.ISO_GREEDY
      ? selectIsoStartPolicy(poolShape)
      : IsoStartPolicy.BLEND;
    if (algo == RoundTripAlgorithm.ISO_GREEDY) {
      logInfo("ISO_GREEDY: start policy " + isoStartPolicy);
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
    if (!roundTripEffortPolicy.skipRetryLayers
        && isDegradedGreedyResult(result)
        && direction >= 0
        && frontierAxis.hasStrongAxis
        && isPerpendicularToAxis(direction, frontierAxis.axisBearingDegrees)
        // Request-budget gate: the axis retry re-runs the whole subRouteCount
        // ladder — only worth starting when the request can still fund it.
        && remainingRequestBudgetMs() >= MIN_LADDER_RUNG_BUDGET_MS) {
      phase21Triggered = true;
      phase21RetryDir = chooseAxisBearing(frontierAxis.axisBearingDegrees, direction);
      logInfo("ISO_GREEDY: Phase 2.1 axis retry — user direction " + (int) direction
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
        logInfo("ISO_GREEDY: Phase 2.1 axis retry ALSO degraded — geographic infeasibility detected");
      }
    }

    RouteChoiceScore.Verdict blendedInternalVerdict = null;
    boolean runInternalBranch = false;
    if (algo == RoundTripAlgorithm.ISO_GREEDY
        && routingContext.roundTripInternalCompare
        && !startGraphNativeOnly
        // QUALITY (runGreedyAlways) already fields a dedicated plain-GREEDY
        // child in the parent competition — this internal comparison would run
        // materially the same graph-native ladder a second time.
        && !roundTripEffortPolicy.runGreedyAlways
        && provider instanceof BlendedCandidateProvider
        && System.currentTimeMillis() < (roundTripRequestDeadline == 0
            ? Long.MAX_VALUE : roundTripRequestDeadline)) {
      // Evaluate the blended verdict ONCE; the selection below reuses it.
      blendedInternalVerdict = scoreInternalGreedyResult(result, desiredDistance, effectiveDirection);
      runInternalBranch = internalBranchNeeded(blendedInternalVerdict);
    }
    if (runInternalBranch) {
      logInfo("ISO_GREEDY: running internal graph-native-only comparison branch");
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
        logInfo("ISO_GREEDY: internal graph-native branch selected");
      } else if (comparable) {
        logInfo("ISO_GREEDY: blended branch kept after internal graph-native comparison");
      } else {
        logInfo("ISO_GREEDY: internal graph-native branch produced no comparable track");
      }
      result = selected;
      if (comparable && result != null) {
        result.setInternalGraphNativeCompared(true);
      }
      lastRoundTripResult = result;
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

    // Phase 2.1 used to also set errorMessage when both attempts degraded
    // (the spec's "refuse with infeasibility error" option). That cut off
    // doRoundTrip's later fallback path (waypoint algorithm), losing 2
    // iso_greedy/gravel scenarios on the broader corpus that the legacy
    // waypoint fallback had been salvaging. Drop the errorMessage write;
    // let the result return as degraded so the caller can fall back as
    // before. The axis info is still surfaced via the Phase 2.1 telemetry
    // fields on RoundTripResult for diagnostic purposes.
    if (phase21Triggered && !phase21Succeeded) {
      logInfo("ISO_GREEDY: Phase 2.1 axis retry also degraded — geographic"
        + " infeasibility (axis " + axisName(frontierAxis.axisBearingDegrees)
        + ", strength " + String.format("%.1fx", frontierAxis.strength)
        + "); falling through to legacy fallback chain");
    }

    // A real loop needs at least a triangle: start + 2 intermediate waypoints + closing
    // start (>= 4 entries). A single intermediate is just an out-and-back, so reject it
    // rather than attributing a legacy waypoint/probe fallback route to GREEDY.
    // Reject loops the planner explicitly flagged as failing its quality gates
    // (DEGRADED_FALLBACK_PREFIX) — shipping a 180% overshoot or 60%-reused
    // forced-closure loop as success would silently fool downstream consumers.
    roundTripForcedCorridorAccepted = result != null && result.isForcedCorridorAccepted();
    boolean degradedFallback = isDegradedGreedyResult(result);
    if (degradedFallback) {
      logInfo("greedy: rejecting degraded fallback (" + result.getFallbackReason()
        + ")");
    }
    if (!degradedFallback
        && result != null && result.getLoopWaypoints() != null
        && result.getLoopWaypoints().size() >= 4) {
      for (String diag : result.getDiagnostics()) {
        logInfo("greedy: " + diag);
      }
      // Spec §10 telemetry — compute-budget audit.
      logInfo("greedy telemetry: candidatesGenerated=" + result.getCandidatesGenerated()
        + ", candidatesRouted=" + result.getCandidatesRouted()
        + ", returnChecks=" + result.getReturnChecksPerformed()
        + ", runtimeMs=" + result.getRuntimeMillis()
        + ", fallbackReason=" + (result.getFallbackReason() == null ? "none" : result.getFallbackReason()));
      // Issue #26 source attribution — the aggregate view of the per-leg
      // "leg N source:" diagnostics logged above.
      logInfo("greedy source attribution: acceptedIso=" + result.getAcceptedIsoLegs()
        + ", acceptedGraphNative=" + result.getAcceptedNonIsoLegs()
        + ", quotaInjectedAccepted=" + result.getAcceptedQuotaInjectedLegs()
        + ", poolHealth=" + (Double.isNaN(result.getIsoPoolHealthScore())
            ? "n/a" : String.format(Locale.US, "%.2f", result.getIsoPoolHealthScore()))
        + ", poolDemotedAtStep=" + result.getPoolDemotedAtStep());
      if (!result.isWithinTolerance()) {
        logInfo("greedy: fallback — " + result.getFallbackReason());
      }
      logInfo("greedy: planned " + result.getLoopWaypoints().size() + " waypoints"
        + ", estimated distance=" + result.getTotalDistanceMeters() + "m");

      // Route through the greedy waypoints with the standard routing engine.
      // The greedy planner's lookahead ensures waypoints are in well-connected
      // areas (not dead-end valleys), so doRouting() produces gap-free tracks
      // following roads appropriate for the profile.
      waypoints.clear();
      waypoints.addAll(result.getLoopWaypoints());

      if (result.getMatchedWaypoints() != null) {
        matchedWaypoints = result.getMatchedWaypoints();
      }

      if (result.getLegTracks() != null) {
        List<OsmTrack> legs = result.getLegTracks();
        greedyLegTracks = legs.toArray(new OsmTrack[0]);
      }

      // Phase 2 v3: the planner now retracks each committed leg, so its
      // merged track has full per-edge MessageData. Use that directly
      // instead of running doRouting() which re-routes via a fragile
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
          foundTrack = result.getTrack();
          if (result.getMatchedWaypoints() != null) {
            matchedWaypoints = result.getMatchedWaypoints();
          }
          trackCleanup().finalizeAdoptedRoundTripTrack(foundTrack, matchedWaypoints);
        } catch (Exception e) {
          logInfo("greedy: bypass path failed (" + e.getClass().getSimpleName() + ": " + e.getMessage() + "), falling back to doRouting");
          useDetailedPlannerTrack = false;
        }
      }
      if (!useDetailedPlannerTrack) {
        routingContext.waypointCatchingRange = 250;
        roundTripSearchRadius = searchRadius;
        // Honor the request deadline: once it has fully passed, do NOT start
        // the fallback re-route at all (doRouting resets startTime, so any
        // budget handed to it is a real overrun). While budget remains, fund
        // the fallback with the REMAINING budget, floored so a nearly-spent
        // request still gets a usable (bounded, < MIN_LADDER_RUNG_BUDGET_MS
        // overrun) salvage slice rather than a guaranteed instant timeout.
        long remaining = remainingRequestBudgetMs();
        if (roundTripRoutingBudgetMs > 0 && remaining <= 0) {
          errorMessage = "round-trip request budget exhausted before the fallback re-route ("
            + remaining + "ms remaining)";
          logInfo(errorMessage);
          foundTrack = null;
          greedyLegTracks = null;
          return;
        }
        try {
          long fallbackBudget = roundTripRoutingBudgetMs <= 0
            ? roundTripRoutingBudgetMs
            : Math.min(roundTripRoutingBudgetMs,
                Math.max(MIN_LADDER_RUNG_BUDGET_MS, remaining));
          doRouting(fallbackBudget);
        } catch (Exception e) {
          logInfo("greedy: doRouting failed (" + e.getClass().getSimpleName() + ": " + e.getMessage() + ")");
          throw e;
        } finally {
          greedyLegTracks = null;
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
          && !roundTripEffortPolicy.skipRetryLayers
          && remainingRequestBudgetMs() >= MIN_LADDER_RUNG_BUDGET_MS) {
        logInfo("ISO_GREEDY produced no loop, falling back to GREEDY with graph-native candidates");
        doGreedyRoundTrip(searchRadius, direction, RoundTripAlgorithm.GREEDY);
      } else if (algo == RoundTripAlgorithm.ISO_GREEDY && !roundTripEffortPolicy.skipRetryLayers) {
        // Same recursion, but the request budget is spent — adopt/report what
        // we have instead of starting another multi-plan GREEDY ladder.
        logInfo("ISO_GREEDY produced no loop and request budget is exhausted ("
          + remainingRequestBudgetMs() + "ms left), skipping GREEDY fallback ladder");
        errorMessage = "greedy round trip planner produced no acceptable loop within the request budget"
          + (result == null || result.getFallbackReason() == null ? "" : ": " + result.getFallbackReason());
        lastRejectedTrack = result == null ? null : result.getTrack();
        foundTrack = null;
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
          logInfo("greedy: adopting best-effort loop for the quality gate to grade ("
            + (result.getFallbackReason() == null ? "?" : result.getFallbackReason()) + ")");
          foundTrack = bestEffort;
          if (result.getMatchedWaypoints() != null) {
            matchedWaypoints = result.getMatchedWaypoints();
          }
          // finalize can throw (voice hints / speed profile / spur removal). Guard
          // it like the bypass path above: an exception here would otherwise
          // unwind past doRoundTrip's floor + quality gate (its catch does not
          // null foundTrack), shipping this un-gated best-effort track as a
          // success. On failure, reject instead so nothing skips the gate.
          try {
            trackCleanup().finalizeAdoptedRoundTripTrack(foundTrack, matchedWaypoints);
            // errorMessage stays null: the floor check + quality gate in
            // doRoundTrip reject (and set errorMessage) if the loop is too small,
            // structurally broken, or strict mode is on; else it ships with a warning.
          } catch (Exception e) {
            errorMessage = "greedy best-effort finalize failed ("
              + e.getClass().getSimpleName() + ": " + e.getMessage() + ")";
            logInfo(errorMessage);
            lastRejectedTrack = bestEffort;
            foundTrack = null;
          }
        } else {
          // Reached by plain GREEDY and by BALANCED's bounded ISO_GREEDY run
          // (which skips the GREEDY recursion) — keep the wording source-neutral.
          errorMessage = "greedy round trip planner produced no acceptable loop"
            + (result == null || result.getFallbackReason() == null ? "" : ": " + result.getFallbackReason());
          logInfo(errorMessage);
          lastRejectedTrack = result == null ? null : result.getTrack();
          foundTrack = null;
        }
      }
    }
  }

  /**
   * Profile family from the profile's own validFor* globals — every standard
   * profile declares exactly one (fastbike/trekking/mtb: validForBikes,
   * hiking: validForFoot, car/moped: validForCars). Name-independent, so
   * custom profiles classify correctly as long as they declare the global;
   * undeclared profiles read UNKNOWN and keep standard-effort behavior.
   */
  private RoundTripEffortPolicy.ProfileClass classifyProfileClass() {
    if (routingContext == null || routingContext.expctxWay == null) {
      return RoundTripEffortPolicy.ProfileClass.UNKNOWN;
    }
    return RoundTripEffortPolicy.classifyProfile(
      routingContext.expctxWay.getVariableValue("validForFoot", 0f) == 1f,
      routingContext.expctxWay.getVariableValue("validForBikes", 0f) == 1f,
      routingContext.expctxWay.getVariableValue("validForCars", 0f) == 1f);
  }

  // Hints closer together than this are treated as one maneuver for round-trip cleanup.
  private static final double ROUNDTRIP_VOICEHINT_MERGE_DIST = 25.0; // meters

  /**
   * Collapse clusters of voice hints produced by synthetic round-trip geometry
   * (waypoint-snapping wiggles and curves reported as several turns). Within a run
   * of hints spaced closer than {@link #ROUNDTRIP_VOICEHINT_MERGE_DIST}, if the net
   * turn is near-straight the whole cluster is dropped; otherwise only the single
   * dominant turn is kept. Roundabouts, beelines and the end marker are never merged,
   * and the conservative distance threshold leaves genuine close turns intact.
   * Round-trip only — does not affect normal point-to-point routes.
   */
  private void consolidateRoundTripVoiceHints(OsmTrack track) {
    if (track.voiceHints == null || track.voiceHints.list.size() < 2) return;
    List<VoiceHint> in = track.voiceHints.list;
    List<VoiceHint> out = new ArrayList<>();
    int i = 0;
    while (i < in.size()) {
      VoiceHint cur = in.get(i);
      if (cur.cmd == VoiceHint.BL || cur.cmd == VoiceHint.END || cur.isRoundabout()) {
        out.add(cur);
        i++;
        continue;
      }
      int j = i;
      float netAngle = (cur.angle == Float.MAX_VALUE) ? 0f : cur.angle;
      VoiceHint dominant = cur;
      while (j + 1 < in.size()) {
        VoiceHint next = in.get(j + 1);
        if (next.cmd == VoiceHint.BL || next.cmd == VoiceHint.END || next.isRoundabout()) break;
        if (in.get(j).distanceToNext >= ROUNDTRIP_VOICEHINT_MERGE_DIST) break;
        netAngle += (next.angle == Float.MAX_VALUE) ? 0f : next.angle;
        if (Math.abs(next.angle) > Math.abs(dominant.angle)) dominant = next;
        j++;
      }
      if (j > i) {
        if (Math.abs(netAngle) >= VoiceHintProcessor.SIGNIFICANT_ANGLE) {
          // keep the cluster's sharpest turn, carrying the trailing distance forward
          dominant.distanceToNext = in.get(j).distanceToNext;
          out.add(dominant);
        } else if (!out.isEmpty()) {
          // net-straight wiggle — drop the cluster, but preserve its distance so the
          // previous instruction's "distance to next" still reaches the following hint.
          double dropped = 0;
          for (int k = i; k <= j; k++) dropped += in.get(k).distanceToNext;
          out.get(out.size() - 1).distanceToNext += dropped;
        }
        i = j + 1;
      } else {
        out.add(cur);
        i++;
      }
    }
    if (out.size() != in.size()) {
      logInfo("roundtrip voicehints: consolidated " + in.size() + " -> " + out.size());
      track.voiceHints.list.clear();
      track.voiceHints.list.addAll(out);
    }
  }

  /**
   * Probe the surrounding area for road reachability in all directions.
   * Sends probes at 15° intervals (24 directions) at three distances
   * (0.7R, 1.0R, 1.3R) and snaps each to the road network. Returns the
   * viable bearings plus a per-direction successful-probe count (the FAST tier
   * consumes that count via {@link PlacementGeometry#filterByProbeConfidence} to drop one-shot
   * weak picks).
   *
   * @param start        the start waypoint
   * @param searchRadius the round-trip search radius in meters
   * @return viable bearings + per-direction scoring; {@code null} on probe failure
   */
  /**
   * Reachability guard (optimization idea 3): {@code true} unless {@code viaMatch}'s
   * road component is a small island that cannot reach the start within
   * {@link #MAXNODES_ISLAND_CHECK} nodes. Reuses the exact bounded-{@code findTrack}
   * primitive as the routing-time "target island" check, so the FAST placement can
   * drop islanded vias before routing instead of failing the whole loop. Cheap: a
   * bounded search that exhausts a small island quickly and gives up on large
   * (reachable) components at the node budget.
   */
  private boolean isViaReachableFromStart(MatchedWaypoint viaMatch, MatchedWaypoint startMatch) {
    if (viaMatch == null || viaMatch.node1 == null || viaMatch.node2 == null
        || startMatch == null || startMatch.node1 == null || startMatch.node2 == null) {
      return true; // cannot test -> keep (conservative)
    }
    boolean savedInverse = routingContext.inverseDirection;
    double savedAir = airDistanceCostFactor;
    int savedNodeLimit = nodeLimit;
    try {
      routingContext.inverseDirection = true;
      airDistanceCostFactor = 0.0;
      nodeLimit = MAXNODES_ISLAND_CHECK;
      OsmTrack seg = findTrack("rt-fast-island-check", viaMatch, startMatch, null, null, false);
      // Reachable if a bounded path was found. null with budget left also means the
      // via's whole component is a small island -> unreachable.
      return !(seg == null && nodeLimit > 0);
    } catch (RoutingIslandException rie) {
      // The bounded search exhausted a small island around the via -> unreachable.
      return false;
    } catch (RuntimeException e) {
      // Best-effort guard: a budget timeout or any other transient failure must
      // not fail the request — keep the via (conservative) and let routing decide.
      return true;
    } finally {
      routingContext.inverseDirection = savedInverse;
      airDistanceCostFactor = savedAir;
      nodeLimit = savedNodeLimit;
    }
  }

  /**
   * Production adapter for the {@link FastWaypointPlanner} seam: coarse
   * delegates to the engine's shared probe/snap/island/circle primitives, so
   * the planner never touches the node cache, {@code findTrack}, or mutable
   * engine fields directly. Package-private so tests can drive the planner
   * through a real engine.
   */

  /**
   * Production adapter for the round-trip planners' engine seam
   * ({@link RoundTripEngineOps}): coarse delegates to the engine's internal
   * leg router, matcher, expansion, timers, and logging. This single public
   * accessor replaces direct cross-package access to engine members, which
   * stay package-private. Delegates qualify with {@code RoutingEngine.this}
   * so an engine subclass override (tests) still receives the call.
   */
  private RoundTripTrackCleanup trackCleanup;

  /** Round-trip track post-processing, extracted behind the ops seam. */
  RoundTripTrackCleanup trackCleanup() {
    if (trackCleanup == null) {
      trackCleanup = new RoundTripTrackCleanup(roundTripOps());
    }
    return trackCleanup;
  }

  private WaypointSnapper waypointSnapper;

  /** Round-trip snap/validate/probe helpers, extracted behind the ops seam. */
  WaypointSnapper waypointSnapper() {
    if (waypointSnapper == null) {
      waypointSnapper = new WaypointSnapper(roundTripOps());
    }
    return waypointSnapper;
  }

  public RoundTripEngineOps roundTripOps() {
    return new RoundTripEngineOps() {
      @Override
      public RoutingContext routingContext() {
        return RoutingEngine.this.routingContext;
      }

      @Override
      public void logInfo(String msg) {
        RoutingEngine.this.logInfo(msg);
      }

      @Override
      public boolean isTerminated() {
        return RoutingEngine.this.isTerminated();
      }

      @Override
      public long startTime() {
        return RoutingEngine.this.startTime;
      }

      @Override
      public long maxRunningTime() {
        return RoutingEngine.this.maxRunningTime;
      }

      @Override
      public double roundTripSearchRadius() {
        return RoutingEngine.this.roundTripSearchRadius;
      }

      @Override
      public boolean isRoundTripMode() {
        return engineMode == BROUTER_ENGINEMODE_ROUNDTRIP;
      }

      @Override
      public boolean explicitViaRoundTrip() {
        return RoutingEngine.this.explicitViaRoundTrip;
      }

      @Override
      public void recalcTrack(OsmTrack track) {
        RoutingEngine.this.recalcTrack(track);
      }

      @Override
      public void consolidateRoundTripVoiceHints(OsmTrack track) {
        RoutingEngine.this.consolidateRoundTripVoiceHints(track);
      }

      @Override
      public void setMatchedWaypoints(List<MatchedWaypoint> waypoints) {
        matchedWaypoints = waypoints;
      }

      @Override
      public void setStartTime(long startTimeMillis) {
        RoutingEngine.this.startTime = startTimeMillis;
      }

      @Override
      public void setMaxRunningTime(long maxRunningTimeMillis) {
        RoutingEngine.this.maxRunningTime = maxRunningTimeMillis;
      }

      @Override
      public void setTransientExpansionDeadline(long deadlineMillis) {
        RoutingEngine.this.transientExpansionDeadline = deadlineMillis;
      }

      @Override
      public double airDistanceCostFactor() {
        return RoutingEngine.this.airDistanceCostFactor;
      }

      @Override
      public void setAirDistanceCostFactor(double factor) {
        RoutingEngine.this.airDistanceCostFactor = factor;
      }

      @Override
      public OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                                OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc) {
        return RoutingEngine.this.findTrack(operationName, startWp, endWp,
          costCuttingTrack, refTrack, fastPartialRecalc);
      }

      @Override
      public OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                                       OsmTrack refTrack) {
        return RoutingEngine.this.retrackForDetail(rawTrack, startWp, endWp, refTrack);
      }

      @Override
      public MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist) {
        return waypointSnapper().profileAwareMatchPoint(ilon, ilat, name, maxSnapDist);
      }

      @Override
      public void resetCache(boolean detailed) {
        RoutingEngine.this.resetCache(detailed);
      }

      @Override
      public OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp,
                                        MatchedWaypoint endWp) {
        OsmTrack savedGuide = guideTrack;
        guideTrack = null; // a live guide track would corrupt the local search
        try {
          return RoutingEngine.this.findTrack(operationName, startWp, endWp, null, null, false);
        } finally {
          guideTrack = savedGuide;
        }
      }

      @Override
      public void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance) {
        nodesCache.matchWaypointsToNodes(waypoints, maxDistance, islandNodePairs);
      }

      @Override
      public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                            OsmTrack refTrack, boolean includeCandidateTracks) {
        return RoutingEngine.this.runIsochroneExpansion(start, searchRadius, refTrack, includeCandidateTracks);
      }
    };
  }

  /**
   * Weight of the air-distance "reach bonus" in the cost-contour scoring rule.
   * The bonus is a soft tiebreaker; this weight is the trade-off threshold,
   * i.e. a 10% normalized cost error completely cancels the max air-reach
   * bonus (chosen so cost dominates whenever it's meaningfully different).
   */
  static final double AIR_REACH_BONUS_WEIGHT = 0.10;

  /** Frontier-entry layout indices for the 6-element isochrone form. */
  private static final int FRONTIER_IDX_ILON = 4;
  private static final int FRONTIER_IDX_ILAT = 5;
  private static final int FRONTIER_LENGTH_ROAD_NATIVE = 6;

  /**
   * Score a Dijkstra-popped node against a target cost level. Lower wins.
   * {@code costError} is the normalized distance from {@code targetCost};
   * {@code airReachBonus} rewards farther-reached nodes as a soft tiebreaker.
   * Used by {@link #runIsochroneExpansion} to pick the per-bucket frontier
   * node and the 25/50/75% contour candidates.
   *
   * @param pathCost     Dijkstra path cost from start, in cost-units
   * @param targetCost   target cost level (costBudget or a contour fraction of it)
   * @param dist         air-distance from start to the popped node, in meters
   * @param searchRadius round-trip search radius, used to normalize air-reach
   */
  static double costContourScore(int pathCost, int targetCost, double dist, double searchRadius) {
    return costContourScore(pathCost, targetCost, clampedAirReachBonus(dist, searchRadius));
  }

  /**
   * Hot-loop overload: caller has already computed {@code airReachBonus} via
   * {@link #clampedAirReachBonus} so the same value can be reused across the
   * frontier + 3 contour evaluations per Dijkstra pop.
   */
  static double costContourScore(int pathCost, int targetCost, double airReachBonus) {
    if (targetCost <= 0) return Double.POSITIVE_INFINITY;
    double costError = Math.abs((double) pathCost - targetCost) / targetCost;
    return costError - AIR_REACH_BONUS_WEIGHT * airReachBonus;
  }

  /**
   * Calibrated isochrone cost budget from the sampled frontier band (see the
   * ISO_BUDGET_* class comment): {@code ISO_TARGET_REACH_FACTOR × searchRadius
   * × median cost-per-air-meter}. Returns the floor when the band is too
   * sparse to trust ({@code sampleCount < ISO_CALIBRATION_MIN_SAMPLES}).
   * Never below the floor (the historical fixed budget), never above the cap.
   */
  static int calibratedIsoBudget(double[] samples, int sampleCount, double searchRadius) {
    int floor = (int) (searchRadius * ISO_BUDGET_FLOOR_FACTOR);
    if (sampleCount < ISO_CALIBRATION_MIN_SAMPLES) return floor;
    double[] band = Arrays.copyOf(samples, sampleCount);
    Arrays.sort(band);
    double medianCostEff = band[sampleCount / 2];
    double budget = ISO_TARGET_REACH_FACTOR * searchRadius * medianCostEff;
    double cap = searchRadius * ISO_BUDGET_CAP_FACTOR;
    return (int) Math.min(cap, Math.max(floor, budget));
  }

  /** {@code clamp(dist / searchRadius, 0, 1)}; 0 when searchRadius is non-positive (avoids a 0/0 NaN). */
  static double clampedAirReachBonus(double dist, double searchRadius) {
    if (searchRadius <= 0.0) {
      return 0.0;
    }
    return Math.min(1.0, Math.max(0.0, dist / searchRadius));
  }

  /**
   * Decide whether the new candidate replaces the current best. Lower score wins;
   * ties broken in order by (1) higher path cost, (2) higher air-distance, (3)
   * existing candidate remains. See {@link #costContourScore}.
   */
  static boolean isBetterCandidate(double newScore, int newCost, double newDist,
                                   double bestScore, int bestCost, double bestDist) {
    if (newScore < bestScore) return true;
    if (newScore > bestScore) return false;
    if (newCost > bestCost) return true;
    if (newCost < bestCost) return false;
    return newDist > bestDist;
  }

  /**
   * Extract the road-native coordinate ({@code [ilon, ilat]}) from a frontier
   * entry, or {@code null} if the entry doesn't carry one. The frontier coord
   * is the cost-budget-envelope node — a fallback for callers that don't pass
   * the full candidate pool to {@link #placeWaypointsFromIsochrone}; production
   * placement prefers {@link #nearestCandidateByAirDist} (airDist-aware).
   *
   * <p>Isochrone-produced entries are 6-element; probe-only entries from
   * {@link #mergeIsochroneWithProbe} are 4-element and have no road-native data.
   */
  static int[] frontierRoadNativeCoord(double[] entry) {
    if (entry == null || entry.length < FRONTIER_LENGTH_ROAD_NATIVE) return null;
    return new int[]{(int) entry[FRONTIER_IDX_ILON], (int) entry[FRONTIER_IDX_ILAT]};
  }

  /**
   * Pick the candidate in {@code bucketCandidates} whose air-distance from start
   * is closest to {@code targetAirDist}, or {@code null} if the bucket has no
   * candidates. Used by {@link #placeWaypointsFromIsochrone} to preserve the
   * indirectness-compensated placement radius while still using a road-native
   * point (each bucket carries one frontier-max + up to three contour candidates
   * at distinct cost depths, so a close airDist match is usually available).
   */
  static IsoCandidate nearestCandidateByAirDist(List<IsoCandidate> bucketCandidates, double targetAirDist) {
    if (bucketCandidates == null || bucketCandidates.isEmpty()) return null;
    IsoCandidate best = null;
    double bestDiff = Double.MAX_VALUE;
    for (IsoCandidate c : bucketCandidates) {
      double diff = Math.abs(c.airDistanceFromStart - targetAirDist);
      if (diff < bestDiff) {
        bestDiff = diff;
        best = c;
      }
    }
    return best;
  }

  /**
   * Run a cost-limited Dijkstra expansion from the start point to discover
   * the reachable road network frontier in all directions.
   *
   * <p>Uses the match → resetCache → getGraphNode pattern from _findTrack to
   * correctly initialize graph nodes from production segment files.
   *
   * @param start        the start waypoint
   * @param searchRadius the round-trip search radius in meters
   * @return frontier table + road-native candidate pool; {@code null} on failure
   */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius) {
    // Start-centered expansion (ISO_GREEDY pool, frontier table): budget
    // calibration ON — searchRadius here is the loop radius the reach target
    // is defined against.
    return runIsochroneExpansion(start, searchRadius, null, false, true);
  }

  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                 OsmTrack refTrack,
                                                 boolean includeCandidateTracks) {
    // Per-step callers (GraphNativeCandidateProvider expands a local disk
    // around the current node each step): calibration OFF — their radius is a
    // step window, not the loop radius, so the reach-target formula does not
    // apply and the historical fixed budget is the correct sizing.
    return runIsochroneExpansion(start, searchRadius, refTrack, includeCandidateTracks, false);
  }

  private IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                         OsmTrack refTrack,
                                                         boolean includeCandidateTracks,
                                                         boolean calibrateBudget) {
    // Phase 1: Match start point (loads segments via directWeaving, consumes node data)
    resetCache(false);
    MatchedWaypoint startMwp = new MatchedWaypoint();
    startMwp.waypoint = new OsmNode(start.ilon, start.ilat);
    startMwp.name = "iso_start";
    List<MatchedWaypoint> mwpList = new ArrayList<>();
    mwpList.add(startMwp);
    double maxSnapDist = Math.min(searchRadius * 0.3, 2000);
    try {
      nodesCache.matchWaypointsToNodes(mwpList, maxSnapDist, islandNodePairs);
    } catch (Exception e) {
      logInfo("isochrone: match failed: " + e.getMessage());
      return null;
    }
    if (startMwp.crosspoint == null || startMwp.node1 == null || startMwp.node2 == null) {
      logInfo("isochrone: start match incomplete");
      return null;
    }

    // Phase 2: Reset cache — creates fresh nodesMap but preserves fileRows (cached segments).
    // This is the critical step: matchWaypointsToNodes consumed segment data via directWeaving,
    // so obtainNonHollowNode would fail without this reset. The reset makes the segments
    // re-parseable while keeping file handles open. Same pattern as findTrack → _findTrack.
    resetCache(false);
    nodesCache.nodesMap.cleanupMode = 1;

    // Phase 3: Get graph nodes — now obtainNonHollowNode can re-parse from cached segments
    OsmNode n1 = nodesCache.getGraphNode(startMwp.node1);
    OsmNode n2 = nodesCache.getGraphNode(startMwp.node2);
    if (!nodesCache.obtainNonHollowNode(n1) || !nodesCache.obtainNonHollowNode(n2)) {
      logInfo("isochrone: could not obtain start nodes");
      return null;
    }
    nodesCache.expandHollowLinkTargets(n1);
    nodesCache.expandHollowLinkTargets(n2);

    OsmPath startPath1 = getStartPath(n1, n2, startMwp, null, false);
    OsmPath startPath2 = getStartPath(n2, n1, startMwp, null, false);

    // Provisional cost budget = the floor (the historical fixed constant). The
    // in-flight calibration below can only RAISE it — see the class-level
    // ISO_BUDGET_* comment. Healthy profiles (fastbike: costEff ≈ 2.0 →
    // calibrated 4×) land on the floor and keep bit-identical behavior.
    int costBudget = (int) (searchRadius * ISO_BUDGET_FLOOR_FACTOR);
    // Calibration state: sample cost-per-air-meter in the band
    // [ISO_CALIBRATION_SAMPLE_LO, 1.0] × searchRadius, finalize at the first
    // pop past the checkpoint. Pops arrive in increasing cost order, so the
    // band is populated completely before the checkpoint fires.
    final int calibrationCheckpointCost = (int) searchRadius;
    final int calibrationSampleLoCost = (int) (searchRadius * ISO_CALIBRATION_SAMPLE_LO);
    // Starting "already calibrated" disables both the sampling and the
    // finalize hook — per-step callers keep the fixed floor budget.
    boolean isoBudgetCalibrated = !calibrateBudget;
    double[] costEffSamples = calibrateBudget ? new double[256] : null;
    int costEffSampleCount = 0;
    // Geographic cutoff: don't expand beyond 1.5× searchRadius (prevents runaway)
    double geoRadiusCutoff = searchRadius * 1.5;
    // Scale maxNodes with search area so dense regions (Berlin) reach the cost
    // budget instead of getting cut off at ~1/3 of it — without that headroom
    // the indirectness signal is dominated by per-link amortization noise.
    double radiusRatio = searchRadius / REFERENCE_LOOP_RADIUS_M;
    double areaScale = Math.max(1.0, radiusRatio * radiusRatio);
    int maxNodes = (int) Math.min(CEILING_ISOCHRONE_MAX_NODES, BASE_ISOCHRONE_MAX_NODES * areaScale);

    // Angular bucketing: 36 buckets of 10 degrees. Per-bucket "best frontier
    // candidate" is picked by cost-contour score — a far-by-air dead-end can
    // sit at low cost and would outrank a budget-cost node on a usable road if
    // we sorted by air-distance alone. See costContourScore + isBetterCandidate.
    int bucketCount = 36;
    double bucketSize = 360.0 / bucketCount;
    double[] bucketBestScore = new double[bucketCount];
    Arrays.fill(bucketBestScore, Double.POSITIVE_INFINITY);
    double[] bucketBestDist = new double[bucketCount];
    int[] bucketBestCost = new int[bucketCount];
    int[] bucketBestIlon = new int[bucketCount];
    int[] bucketBestIlat = new int[bucketCount];
    OsmPath[] bucketBestPath = new OsmPath[bucketCount];
    int[] bucketHits = new int[bucketCount]; // population count per bucket (sparseness signal)

    // Cost contours for ISO_GREEDY candidate extraction. Per bucket, record the
    // node whose path.cost is closest to each intermediate cost level — yields a
    // road-native pool spread across both direction and cost depth.
    int[] contourLabels = {25, 50, 75};
    int contourCount = contourLabels.length;
    int[] contourCosts = new int[contourCount];
    for (int k = 0; k < contourCount; k++) contourCosts[k] = (int) (contourLabels[k] * 0.01 * costBudget);
    double[][] bucketContourBestScore = new double[bucketCount][contourCount];
    for (double[] row : bucketContourBestScore) Arrays.fill(row, Double.POSITIVE_INFINITY);
    double[][] bucketContourDist = new double[bucketCount][contourCount];
    int[][] bucketContourCost = new int[bucketCount][contourCount];
    int[][] bucketContourIlon = new int[bucketCount][contourCount];
    int[][] bucketContourIlat = new int[bucketCount][contourCount];
    OsmPath[][] bucketContourPath = new OsmPath[bucketCount][contourCount];

    // Local open set — not the instance field, to avoid state contamination
    SortedHeap<OsmPath> isoOpenSet = new SortedHeap<>();
    if (startPath1 != null) isoOpenSet.add(startPath1.cost, startPath1);
    if (startPath2 != null) isoOpenSet.add(startPath2.cost, startPath2);

    int nodesExpanded = 0;

    // Reachability cloud (pocket-avoiding placement): fixed per-expansion
    // scale, captured once at the start latitude — CheapRuler's banded scale
    // cache could otherwise map one physical point into two cells.
    double[] cellKxKy = CheapRuler.getLonLatToMeterScales(start.ilat);
    int cellDivLon = Math.max(1, (int) (REACHABILITY_CELL_M / cellKxKy[0]));
    int cellDivLat = Math.max(1, (int) (REACHABILITY_CELL_M / cellKxKy[1]));
    // Cell -> min Dijkstra cost. Pops arrive in cost order, so the first touch
    // of a cell records its minimum (putIfAbsent); key presence doubles as the
    // reachability cloud, the value feeds the ReturnDistanceOracle.
    Map<Long, Integer> cellMinCost = new HashMap<>(4096);

    long expansionDeadline = transientExpansionDeadline;
    if (roundTripRequestDeadline > 0) {
      expansionDeadline = expansionDeadline > 0
        ? Math.min(expansionDeadline, roundTripRequestDeadline) : roundTripRequestDeadline;
    }

    int popTick = 0;
    for (;;) {
      // Wall-clock + watchdog guard (same contract as _findTrack's pop loop):
      // stop expanding and return the partial frontier — callers already
      // handle sparse candidate sets gracefully, and a partial frontier beats
      // an un-killable multi-second expansion overrunning every deadline.
      // The volatile kill flag is checked every pop; the wall clock only every
      // 4096 pops (a currentTimeMillis per pop is measurable at ~1.5M pops,
      // and 4096 pops complete in well under any deadline granularity).
      if (terminated
          || (expansionDeadline > 0 && (++popTick & 0xFFF) == 0
              && System.currentTimeMillis() > expansionDeadline)) {
        logInfo("isochrone: expansion stopped early (" + (terminated ? "terminated" : "deadline")
          + ") after " + nodesExpanded + " nodes");
        break;
      }

      OsmPath path = isoOpenSet.popLowestKeyValue();
      if (path == null) break;
      if (path.airdistance == -1) continue; // invalidated

      // In-flight budget calibration: finalize at the first pop past the
      // checkpoint (pops arrive in increasing cost order, so the sample band
      // below is complete here). Raising the budget resets the frontier and
      // contour picks — every competitive fit for the raised targets (all
      // ≥ checkpoint, guaranteed by the floor) pops after this point, so the
      // reset discards nothing that could have won. No raise = bit-identical
      // to the historical fixed budget.
      if (!isoBudgetCalibrated && path.cost > calibrationCheckpointCost) {
        isoBudgetCalibrated = true;
        int calibrated = calibratedIsoBudget(costEffSamples, costEffSampleCount, searchRadius);
        if (calibrated > costBudget) {
          logInfo("isochrone: calibrated cost budget " + costBudget + " -> " + calibrated
            + " (x" + String.format(Locale.ROOT, "%.1f",
              calibrated / searchRadius) + " searchRadius, "
            + costEffSampleCount + " band samples)");
          costBudget = calibrated;
          for (int k = 0; k < contourCount; k++) {
            contourCosts[k] = (int) (contourLabels[k] * 0.01 * costBudget);
          }
          Arrays.fill(bucketBestScore, Double.POSITIVE_INFINITY);
          for (double[] row : bucketContourBestScore) {
            Arrays.fill(row, Double.POSITIVE_INFINITY);
          }
        }
      }

      // Cost cutoff — Dijkstra: once popped cost exceeds budget, all remaining do too
      if (path.cost > costBudget) break;

      OsmLink currentLink = path.getLink();
      OsmNode sourceNode = path.getSourceNode();
      OsmNode currentNode = path.getTargetNode();
      if (currentLink.isLinkUnused()) continue;

      // Count expansions only for real link processing — skipped links shouldn't
      // consume the budget (could prematurely truncate exploration in dense graphs).
      nodesExpanded++;
      if (nodesExpanded > maxNodes) break;

      // Record this node in angular buckets using true bearing (longitude-scaled).
      // Selection is by cost-contour score; air-distance is only a soft tiebreaker
      // (see AIR_REACH_BONUS_WEIGHT).
      int curIlon = currentNode.getILon();
      int curIlat = currentNode.getILat();
      long cmcKey = (((long) (curIlon / cellDivLon)) << 32) | ((curIlat / cellDivLat) & 0xFFFFFFFFL);
      if (!cellMinCost.containsKey(cmcKey)) {
        cellMinCost.put(cmcKey, path.cost);
      }
      double dist = CheapRuler.distance(start.ilon, start.ilat, curIlon, curIlat);
      if (dist > 50) { // skip very close nodes (noisy bearings)
        int pcost = path.cost;
        // Calibration band sample: cost-per-air-meter of frontier-band pops.
        // The finalize check above flips the flag at the first pop past the
        // checkpoint, so this band is exactly [SAMPLE_LO, 1.0] × searchRadius.
        if (!isoBudgetCalibrated && pcost >= calibrationSampleLoCost) {
          if (costEffSampleCount == costEffSamples.length) {
            costEffSamples = Arrays.copyOf(costEffSamples, costEffSamples.length * 2);
          }
          costEffSamples[costEffSampleCount++] = pcost / dist;
        }
        double bearing = CheapRuler.getScaledBearing(start.ilon, start.ilat, curIlon, curIlat);
        int bucket = ((int) (bearing / bucketSize)) % bucketCount;
        if (bucket < 0) bucket += bucketCount;
        bucketHits[bucket]++;
        double airReachBonus = clampedAirReachBonus(dist, searchRadius);

        // Frontier candidate: target = full cost budget (cost envelope edge).
        double frontierScore = costContourScore(pcost, costBudget, airReachBonus);
        if (isBetterCandidate(frontierScore, pcost, dist,
          bucketBestScore[bucket], bucketBestCost[bucket], bucketBestDist[bucket])) {
          bucketBestScore[bucket] = frontierScore;
          bucketBestCost[bucket] = pcost;
          bucketBestDist[bucket] = dist;
          bucketBestIlon[bucket] = curIlon;
          bucketBestIlat[bucket] = curIlat;
          bucketBestPath[bucket] = path;
        }

        // Contour candidates: targets at 25/50/75% of budget. Score-based
        // selection allows above-contour wins, so every pop is evaluated against
        // every contour (3 cheap compares). Row hoist keeps bounds-check
        // elimination working on the inner index.
        double[] rowScore = bucketContourBestScore[bucket];
        int[]    rowCost  = bucketContourCost[bucket];
        double[] rowDist  = bucketContourDist[bucket];
        int[]    rowIlon  = bucketContourIlon[bucket];
        int[]    rowIlat  = bucketContourIlat[bucket];
        for (int k = 0; k < contourCount; k++) {
          double cscore = costContourScore(pcost, contourCosts[k], airReachBonus);
          if (isBetterCandidate(cscore, pcost, dist, rowScore[k], rowCost[k], rowDist[k])) {
            rowScore[k] = cscore;
            rowCost[k] = pcost;
            rowDist[k] = dist;
            rowIlon[k] = curIlon;
            rowIlat[k] = curIlat;
            bucketContourPath[bucket][k] = path;
          }
        }
      }

      // Invalidate existing path holders for this link
      OsmLinkHolder firstLinkHolder = currentLink.getFirstLinkHolder(sourceNode);
      for (OsmLinkHolder lh = firstLinkHolder; lh != null; lh = lh.getNextForLink()) {
        ((OsmPath) lh).airdistance = -1;
      }

      // Unlink processed link
      if (path.treedepth > 1) {
        boolean isBidir = currentLink.isBidirectional();
        sourceNode.unlinkLink(currentLink);
        if (isBidir && currentLink.getFirstLinkHolder(currentNode) == null
          && !routingContext.considerTurnRestrictions) {
          currentNode.unlinkLink(currentLink);
        }
      }

      // Don't expand beyond geographic radius
      if (dist > geoRadiusCutoff) continue;

      // Two-pass neighbor expansion (prePath + path creation)
      routingContext.firstPrePath = null;
      for (OsmLink link = currentNode.firstlink; link != null; link = link.getNext(currentNode)) {
        OsmNode nextNode = link.getTarget(currentNode);
        if (!nodesCache.obtainNonHollowNode(nextNode)) continue;
        if (nextNode.firstlink == null) continue;
        if (nextNode == sourceNode) continue;

        OsmPrePath prePath = routingContext.createPrePath(path, link);
        if (prePath != null) {
          prePath.next = routingContext.firstPrePath;
          routingContext.firstPrePath = prePath;
        }
      }

      for (OsmLink link = currentNode.firstlink; link != null; link = link.getNext(currentNode)) {
        OsmNode nextNode = link.getTarget(currentNode);
        if (!nodesCache.obtainNonHollowNode(nextNode)) continue;
        if (nextNode.firstlink == null) continue;
        if (nextNode == sourceNode) continue;

        OsmPath bestPath = null;
        for (OsmLinkHolder lh = firstLinkHolder; lh != null; lh = lh.getNextForLink()) {
          OsmPath otherPath = (OsmPath) lh;
          OsmPath testPath = routingContext.createPath(otherPath, link, refTrack, false);
          if (testPath.cost >= 0 && (bestPath == null || testPath.cost < bestPath.cost)
            && testPath.sourceNode.getIdFromPos() != testPath.targetNode.getIdFromPos()) {
            bestPath = testPath;
          }
        }

        if (bestPath != null) {
          bestPath.airdistance = 0; // pure Dijkstra — no heuristic

          // Domination check
          OsmLinkHolder dominator = link.getFirstLinkHolder(currentNode);
          while (dominator != null) {
            OsmPath dp = (OsmPath) dominator;
            if (dp.airdistance != -1 && bestPath.definitlyWorseThan(dp)) break;
            dominator = dominator.getNextForLink();
          }
          if (dominator == null) {
            bestPath.treedepth = path.treedepth + 1;
            link.addLinkHolder(bestPath, currentNode);
            isoOpenSet.add(bestPath.cost, bestPath);
          }
        }
      }
    }

    // Compile per-bucket frontier entries — see IsochroneExpansionResult.frontier.
    // hits<3 is the dead-end signal used by downstream filters.
    List<double[]> results = new ArrayList<>();
    // Road-native candidate list for ISO_GREEDY. Each populated bucket
    // contributes one candidate per contour plus the frontier-max.
    List<IsoCandidate> candidatePool = new ArrayList<>();
    for (int b = 0; b < bucketCount; b++) {
      if (bucketBestScore[b] < Double.POSITIVE_INFINITY) {
        double bucketBearing = b * bucketSize + bucketSize / 2.0;
        results.add(new double[]{
          bucketBearing,
          bucketBestDist[b],
          bucketBestCost[b],
          bucketHits[b],
          bucketBestIlon[b],
          bucketBestIlat[b]});
        for (int k = 0; k < contourCount; k++) {
          if (bucketContourBestScore[b][k] < Double.POSITIVE_INFINITY) {
            candidatePool.add(new IsoCandidate(
              bucketContourIlon[b][k], bucketContourIlat[b][k],
              bucketBearing, bucketContourDist[b][k], bucketContourCost[b][k],
              b, bucketHits[b], contourLabels[k],
              includeCandidateTracks ? compileCandidateTrack(bucketContourPath[b][k]) : null));
          }
        }
        candidatePool.add(new IsoCandidate(
          bucketBestIlon[b], bucketBestIlat[b],
          bucketBearing, bucketBestDist[b], bucketBestCost[b],
          b, bucketHits[b], 100,
          includeCandidateTracks ? compileCandidateTrack(bucketBestPath[b]) : null));
      }
    }
    logInfo("isochrone: " + nodesExpanded + " nodes expanded"
      + (nodesExpanded >= maxNodes ? " (maxNodes limit)" : "")
      + ", " + results.size() + "/" + bucketCount + " buckets populated");
    if (results.isEmpty()) return null;
    return new IsochroneExpansionResult(results.toArray(new double[0][]), candidatePool,
      cellMinCost, cellDivLon, cellDivLat);
  }

  private OsmTrack compileCandidateTrack(OsmPath path) {
    if (path == null) return null;
    try {
      return compileTrack(path, false);
    } catch (RuntimeException e) {
      logInfo("graph-native candidate track compile failed: " + e.getMessage());
      return null;
    }
  }

  /**
   * Place waypoints using per-direction indirectness from the isochrone expansion.
   *
   * The isochrone gives [direction, airDistance, routeCost] per angular bucket.
   * The ratio cost/airDist is the road indirectness factor for that direction:
   * - In a flat valley (E-W at Innsbruck): indirectness ~1.3 (roads follow valley)
   * - Across mountains (N-S at Innsbruck): indirectness ~3-5× (switchbacks)
   *
   * To hit the target loop distance, we compute a per-leg route-distance budget
   * and convert it to air distance using the per-direction indirectness:
   *   airDist = targetLegRouteDistance / indirectness
   *
   * This naturally produces elongated loops in valleys and compact loops in open terrain.
   */
  /** ISOCHRONE direction-bulge strength: the per-direction placement radius is
   *  scaled by 1 + alpha*cos(theta - heading), a mean-preserving cardioid toward
   *  the requested heading (0 = legacy even ring). Package-private and non-final
   *  only so tests can drive both ends — it is not a runtime knob. */
  static double isochroneDirBulgeAlpha = 0.35;

  private FastPlacementOps fastPlacementOps() {
    return new FastPlacementOps() {
      @Override
      public ProbeResult probe(OsmNodeNamed start, double searchRadius, double[] bearings) {
        return waypointSnapper().probeReachableDirectionsFast(start, searchRadius, bearings);
      }

      @Override
      public SnapUsability snapUsability(MatchedWaypoint m) {
        return waypointSnapper().snapUsability(m);
      }

      @Override
      public boolean isViaReachable(MatchedWaypoint via, MatchedWaypoint startMatch) {
        return isViaReachableFromStart(via, startMatch);
      }

      @Override
      public void circleFallbackValidated(List<OsmNodeNamed> skeleton, double direction,
                                          double searchRadius, int targetPoints) {
        buildPointsFromCircle(skeleton, direction, searchRadius, targetPoints);
        waypointSnapper().validateAndAdjustWaypoints(skeleton, searchRadius);
      }

      @Override
      public void log(String msg) {
        logInfo(msg);
      }
    };
  }

  void placeWaypointsFromIsochrone(List<OsmNodeNamed> waypoints, double[][] frontierData,
                                   List<IsoCandidate> isoCandidates,
                                   double searchRadius, double startDirection, int targetPoints) {
    OsmNodeNamed start = waypoints.get(0);
    int needed = targetPoints - 1;
    if (needed < 2) needed = 2;

    // Group the road-native candidate pool by bucket so per-direction placement
    // can pick the candidate (frontier-max or 25/50/75 contour) whose
    // air-distance is closest to the indirectness-compensated target — preserves
    // the loop-size scaling while keeping the waypoint on a real road.
    Map<Integer, List<IsoCandidate>> candidatesByBucket;
    if (isoCandidates != null && !isoCandidates.isEmpty()) {
      candidatesByBucket = new HashMap<>();
      for (IsoCandidate c : isoCandidates) {
        List<IsoCandidate> bucket = candidatesByBucket.get(c.bucket);
        if (bucket == null) {
          bucket = new ArrayList<>();
          candidatesByBucket.put(c.bucket, bucket);
        }
        bucket.add(c);
      }
    } else {
      candidatesByBucket = Collections.emptyMap();
    }

    // Pre-filter the frontier data:
    //   1. Drop sea-blocked / dead-end directions whose airDist is far below the
    //      target placement radius. Selecting these would put a waypoint in the
    //      ocean (coastal_nice 50km failure mode) or at a one-shot dead-end
    //      (rural_lozere garbage signal).
    //   2. Drop low-population buckets (hits < 3) — likely one-shot dead-ends.
    //   3. Keep at least 4 directions even if filtering would leave fewer, so
    //      the loop can still be constructed.
    double minFrontierReach = searchRadius * 0.4;
    int minHits = 3;
    List<double[]> usable = new ArrayList<>();
    for (double[] entry : frontierData) {
      double airDist = entry[1];
      int hits = entry.length > 3 ? (int) entry[3] : 1;
      if (airDist >= minFrontierReach && hits >= minHits) {
        usable.add(entry);
      }
    }
    if (usable.size() < 4) {
      // Signal too thin — relax filters and take whatever we have.
      usable.clear();
      for (double[] entry : frontierData) {
        if (entry[1] >= searchRadius * 0.2) usable.add(entry);
      }
    }

    int n = usable.size();
    if (needed > n) needed = n;
    if (needed < 2) needed = 2;

    double[] directions = new double[n];
    Map<Double, double[]> dirToData = new HashMap<>(); // dir -> entry array
    for (int i = 0; i < n; i++) {
      double[] entry = usable.get(i);
      directions[i] = entry[0];
      dirToData.put(entry[0], entry);
    }

    double[] selected;
    if (needed >= n) {
      selected = directions;
    } else {
      selected = PlacementGeometry.selectSpreadDirections(directions, needed, startDirection);
    }
    selected = PlacementGeometry.sortDirectionsForLoop(selected, startDirection);

    // Base radius from polygon geometry (legacy v1.7.8-compatible), then
    // compensated by observed road-geometry indirectness. The cost/airDist
    // ratio from the isochrone equals (roadDist/airDist) × profileCostFactor.
    // To isolate the road geometry part we estimate profileCostFactor from the
    // minimum observed indirectness across all usable directions (the easiest
    // road's indirectness ≈ pure profile costfactor on flat direct road).
    double geomBase = searchRadius * PlacementGeometry.computeRadiusScale(selected, targetPoints);

    // Per-direction observed cost/airDist ratio. Median (not mean) so a single
    // outlier direction doesn't dominate redistribution.
    double[] selectedInd = new double[selected.length];
    for (int i = 0; i < selected.length; i++) {
      double[] data = dirToData.get(selected[i]);
      double airDist = data[1];
      double cost = data[2];
      selectedInd[i] = (airDist > 50) ? Math.max(1.0, cost / airDist) : 1.5;
    }
    double[] sortedInd = selectedInd.clone();
    Arrays.sort(sortedInd);
    double medianInd = Math.max(1.0, sortedInd[selectedInd.length / 2]);

    // Estimate profile-only cost factor from the easiest direction across ALL
    // observed directions (not just selected), so directional pre-filter doesn't
    // bias it. Min reasonable value: 1.0 (already clamped during data read).
    double minObservedInd = Double.MAX_VALUE;
    for (double[] entry : usable) {
      double aD = entry[1], c = entry[2];
      if (aD > 50) {
        double ind = Math.max(1.0, c / aD);
        if (ind < minObservedInd) minObservedInd = ind;
      }
    }
    if (minObservedInd == Double.MAX_VALUE) minObservedInd = DEFAULT_PROBE_INDIRECTNESS;
    double profileCostFactor = Math.max(1.0, minObservedInd);
    // Pure road-geometry indirectness (road meters per air meter), profile-free.
    double geomInd = medianInd / profileCostFactor;
    // Compensate the base radius: in indirect terrain (high geomInd) shrink so
    // the actual routed loop matches target distance. Conservative ±20%.
    double indCompensation = REFERENCE_GEOM_INDIRECTNESS / Math.max(1.0, geomInd);
    indCompensation = Math.max(IND_COMPENSATION_MIN, Math.min(IND_COMPENSATION_MAX, indCompensation));
    double baseRadius = geomBase * indCompensation;

    // Per-direction redistribution factors. Indirect dirs (mountains) → factor <
    // 1 → closer; direct dirs (valley floor) → factor > 1 → farther. Normalize
    // so the average factor = 1.0 (mean-preserving).
    double[] rawFactors = new double[selected.length];
    double factorSum = 0;
    for (int i = 0; i < selected.length; i++) {
      rawFactors[i] = medianInd / selectedInd[i];
      factorSum += rawFactors[i];
    }
    double normalization = selected.length / factorSum;

    // Directional bulge: bias the placement radius toward startDirection so the
    // loop heads that way (a cardioid) instead of encircling evenly — this is
    // what lets ISOCHRONE honour the requested direction, which the bare
    // even-spread frontier sampling cannot. Mean-preserving: the per-direction
    // factors are renormalised to average 1.0, so the loop's overall size — and
    // therefore the distance gate — is unchanged; only its shape shifts toward
    // the heading. isochroneDirBulgeAlpha=0 reproduces the legacy even ring.
    double dirBulgeAlpha = isochroneDirBulgeAlpha;
    double[] dirBulge = new double[selected.length];
    double dirBulgeSum = 0;
    for (int i = 0; i < selected.length; i++) {
      dirBulge[i] = 1.0 + dirBulgeAlpha * Math.cos(Math.toRadians(selected[i] - startDirection));
      dirBulgeSum += dirBulge[i];
    }
    double dirBulgeNorm = dirBulgeSum > 0 ? selected.length / dirBulgeSum : 1.0;

    double maxDist = searchRadius * 1.5;
    double minDist = searchRadius * 0.15;
    int roadNativeCount = 0;
    int syntheticCount = 0;
    double bucketSize = 360.0 / 36; // matches runIsochroneExpansion
    for (int i = 0; i < selected.length; i++) {
      double factor = Math.max(0.5, Math.min(2.0,
        rawFactors[i] * normalization * dirBulge[i] * dirBulgeNorm));
      double airDist = baseRadius * factor;
      airDist = Math.max(minDist, Math.min(maxDist, airDist));

      // Pick the road-native candidate in this bucket whose air-distance is
      // closest to airDist — but only if it's within ±2× of the target,
      // otherwise the candidate (typically the cost-budget-envelope frontier-max)
      // would defeat the per-direction indirectness compensation. When out of
      // tolerance, synthesize at the exact target and let matchWaypointsToNodes
      // snap to the nearest road. Frontier-entry coord (entry[4..5]) is a
      // legacy fallback for callers without a candidate pool.
      int bucketIdx = ((int) (selected[i] / bucketSize)) % 36;
      if (bucketIdx < 0) bucketIdx += 36;
      IsoCandidate bestCand = nearestCandidateByAirDist(candidatesByBucket.get(bucketIdx), airDist);
      boolean candAcceptable = bestCand != null
        && bestCand.airDistanceFromStart >= airDist * 0.5
        && bestCand.airDistanceFromStart <= airDist * 2.0;
      int[] pos;
      if (candAcceptable) {
        pos = new int[]{bestCand.ilon, bestCand.ilat};
        roadNativeCount++;
      } else if (bestCand == null) {
        int[] frontierCoord = frontierRoadNativeCoord(dirToData.get(selected[i]));
        if (frontierCoord != null) {
          pos = frontierCoord;
          roadNativeCount++;
        } else {
          pos = CheapRuler.destination(start.ilon, start.ilat, airDist, selected[i]);
          syntheticCount++;
        }
      } else {
        pos = CheapRuler.destination(start.ilon, start.ilat, airDist, selected[i]);
        syntheticCount++;
      }
      OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
      onn.name = "rt" + (i + 1);
      waypoints.add(onn);
    }

    OsmNodeNamed closing = new OsmNodeNamed(start);
    closing.name = "to_rt";
    waypoints.add(closing);

    logInfo("placeWaypointsFromIsochrone: " + selected.length + " waypoints"
      + " (" + roadNativeCount + " road-native, " + syntheticCount + " synthetic)"
      + ", baseRadius=" + (int) baseRadius + "m"
      + ", medianInd=" + String.format("%.2f", medianInd)
      + ", searchRadius=" + (int) searchRadius + "m");
  }

  /**
   * Place waypoints from the reachability envelope at a scaled search radius.
   * Selects N directions from the viable set that maximize angular spread,
   * then scales the radius to match v1.7.8's expected loop distance.
   *
   * <p><b>Known parity gap (P5):</b> unlike {@link #placeWaypointsFromIsochrone},
   * this fallback applies only the geometric {@code computeRadiusScale} correction
   * and does NOT apply terrain-indirectness compensation (it has no per-direction
   * isochrone cost data to derive it from). It is reached precisely when the
   * merged isochrone+probe frontier has &lt;3 usable directions — i.e. indirect
   * terrain (mountains/coast), where unadjusted radii tend to overshoot the target
   * loop distance. Adding a conservative {@link #DEFAULT_PROBE_INDIRECTNESS}-based
   * shrink here is a candidate improvement but is a tuning change that needs
   * loop-quality-corpus validation before landing (cf. the analogous out-of-scope
   * note in docs/features/roundtrip-benchmark-2026-05.md).
   */
  void placeWaypointsFromEnvelope(List<OsmNodeNamed> waypoints, double[] viableDirections,
                                  double searchRadius, double startDirection, int targetPoints) {
    OsmNodeNamed start = waypoints.get(0);
    int n = viableDirections.length;
    int needed = targetPoints - 1;
    if (needed > n) needed = n;
    if (needed < 2) needed = 2;

    double[] selected;
    if (needed >= n) {
      selected = viableDirections;
    } else {
      selected = PlacementGeometry.selectSpreadDirections(viableDirections, needed, startDirection);
    }

    selected = PlacementGeometry.sortDirectionsForLoop(selected, startDirection);

    // Scale radius so the loop perimeter matches v1.7.8's expected distance.
    // v1.7.8 uses buildPointsFromCircle which creates a narrow arc (108-152°).
    // The probe's wider direction spread produces longer loops at the same radius.
    double adjustedRadius = searchRadius * PlacementGeometry.computeRadiusScale(selected, targetPoints);

    for (int i = 0; i < selected.length; i++) {
      int[] pos = CheapRuler.destination(start.ilon, start.ilat, adjustedRadius, selected[i]);
      OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
      onn.name = "rt" + (i + 1);
      waypoints.add(onn);
    }

    OsmNodeNamed closing = new OsmNodeNamed(start);
    closing.name = "to_rt";
    waypoints.add(closing);

    logInfo("placeWaypointsFromEnvelope: " + selected.length + " waypoints, radius "
      + (int) searchRadius + "m -> " + (int) adjustedRadius + "m (scale "
      + String.format("%.2f", adjustedRadius / searchRadius) + ")");
  }

  /**
   * Runs one greedy planning attempt — the inner sub-route-count loop with
   * a single {@code tryDirection}. Used by Phase 2.1 to attempt the same
   * planner twice (user direction first, axis-aligned direction on retry)
   * without code duplication.
   *
   * <p>Stamps Phase 2.0 telemetry on the result and updates
   * {@link #lastRoundTripResult} on every iteration so cross-attempt
   * comparison sees consistent metadata. Returns the final
   * {@link RoundTripResult} produced (which may be degraded — the caller
   * decides whether to accept or retry).
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
      long remaining = remainingRequestBudgetMs();
      if (!firstRung && remaining < MIN_LADDER_RUNG_BUDGET_MS) {
        logInfo("greedy: request budget exhausted (" + remaining
          + "ms left), skipping remaining subRouteCount ladder");
        break;
      }
      firstRung = false;
      logInfo("greedy round trip: subRouteCount=" + subRouteCount + ", direction=" + (int) tryDirection);
      GreedyRoundTripPlanner planner = new GreedyRoundTripPlanner(roundTripOps(), provider,
        new CandidateScorer(), subRouteCount, 0.05, 8);
      planner.setHostilityActive(RoundTripQualityGate.isPavedProfile(routingContext.getProfileName()));
      planner.setProfileName(routingContext.getProfileName());
      planner.setVarietySeed(routingContext.getRoundTripSeed());
      planner.setRouteBudgets(roundTripEffortPolicy.topKNormal, roundTripEffortPolicy.topKLate);
      planner.setPlanBudgetScale(roundTripEffortPolicy.planBudgetScale);
      planner.setReturnOracle(returnOracle);
      // Fresh per-plan health tracker: dynamic evidence must not leak across
      // ladder rungs (a demotion earned at subRouteCount=5 says nothing about
      // the 4-step plan's pool usage).
      planner.setPoolHealth(poolShape == null ? null : new IsoPoolHealth(poolShape));
      planner.setExternalDeadline(roundTripRequestDeadline == 0
        ? Long.MAX_VALUE : roundTripRequestDeadline);
      result = planner.plan(start, desiredDistance, tryDirection);
      if (result != null) {
        result.setIsoAsymmetryBearingApplied(bias.applied);
        result.setIsoAsymmetryBearingDegrees(bias.bearingDegrees);
        result.setIsoAsymmetryBestBucketIndirectness(bias.indirectness);
        result.setIsoAsymmetryBestBucketHits(bias.hits);
        result.setIsoAsymmetryBestBucketAirDistMeters(bias.airDistMeters);
      }
      lastRoundTripResult = result;
      if (!isDegradedGreedyResult(result)
          && result != null && result.getLoopWaypoints() != null
          && result.getLoopWaypoints().size() >= 4) {
        return result;
      }
      logInfo("greedy: attempt with " + subRouteCount + " sub-routes did not produce an acceptable loop"
        + (result == null || result.getFallbackReason() == null ? "" : " (" + result.getFallbackReason() + ")"));
    }
    return result;
  }

  /** Phase 2.1: human-readable axis label for the infeasibility error. */
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
   * Phase 2.0 of the closure-aware planning spec — isochrone-asymmetry
   * initial bearing. Examines the 36-bucket frontier table produced by
   * {@link #runIsochroneExpansion} and selects the most-reaching sector
   * (lowest {@code cost / airDist}) subject to quality thresholds.
   *
   * <p>Returns {@link IsoAsymmetryBias#NONE} when no bucket clears the
   * thresholds. The caller falls back to the legacy direction-selection
   * behavior in that case.
   *
   * <p>Package-private + static for unit testing with synthetic frontier
   * tables.
   */
  static IsoAsymmetryBias computeIsoAsymmetryBearing(double[][] frontier, double searchRadius) {
    if (frontier == null || frontier.length == 0) return IsoAsymmetryBias.NONE;
    final double minAirDist = 0.6 * searchRadius;
    final int minHits = 3;
    int bestIdx = -1;
    double bestIndirectness = Double.POSITIVE_INFINITY;
    for (int i = 0; i < frontier.length; i++) {
      double[] b = frontier[i];
      if (b == null || b.length < 4) continue;
      double airDist = b[1];
      double cost = b[2];
      int hits = (int) b[3];
      if (airDist < minAirDist || hits < minHits || airDist <= 0) continue;
      double indirectness = cost / airDist;
      if (indirectness < bestIndirectness) {
        bestIndirectness = indirectness;
        bestIdx = i;
      }
      // Tie-break: lowest bucket index wins (already enforced by strict <).
    }
    if (bestIdx < 0) return IsoAsymmetryBias.NONE;
    double[] best = frontier[bestIdx];
    return new IsoAsymmetryBias(true, best[0], bestIndirectness,
        (int) best[3], (int) best[1]);
  }

  /**
   * Phase 2.1 of the closure-aware planning spec — frontier-axis detection
   * for axis-aware retry when the user's explicit direction conflicts with
   * the terrain's natural orientation.
   *
   * <p>Computes the principal axis of the reachable-frontier displacement
   * vectors (good-quality buckets only, same thresholds as Phase 2.0).
   * Returns an axis bearing in [0, 180) and the eigenvalue ratio
   * (primary / secondary). An axis is considered "strong" when the ratio
   * exceeds {@link #PHASE_2_1_STRONG_AXIS_RATIO}, indicating the reachable
   * region is markedly elongated (Inn Valley, coast roads, ridge tops).
   */
  static FrontierAxis computeFrontierAxis(double[][] frontier, double searchRadius) {
    if (frontier == null || frontier.length < 6) return FrontierAxis.NONE;
    final double minAirDist = 0.6 * searchRadius;
    final int minHits = 3;
    double sumX2 = 0, sumY2 = 0, sumXY = 0;
    int n = 0;
    for (double[] b : frontier) {
      if (b == null || b.length < 4) continue;
      double airDist = b[1];
      int hits = (int) b[3];
      if (airDist < minAirDist || hits < minHits) continue;
      // Compass bearing → (east, north) Cartesian.
      double rad = Math.toRadians(b[0]);
      double x = airDist * Math.sin(rad); // east
      double y = airDist * Math.cos(rad); // north
      sumX2 += x * x;
      sumY2 += y * y;
      sumXY += x * y;
      n++;
    }
    if (n < 4) return FrontierAxis.NONE;
    double a = sumX2 / n;
    double bb = sumY2 / n;
    double c = sumXY / n;
    double trace = a + bb;
    double det = a * bb - c * c;
    double disc = Math.sqrt(Math.max(0, trace * trace - 4 * det));
    double lambda1 = (trace + disc) / 2;
    double lambda2 = (trace - disc) / 2;
    if (lambda2 <= 0 || lambda1 <= 0) return FrontierAxis.NONE;
    double strength = lambda1 / lambda2;
    // Closed-form principal-axis angle for a 2x2 symmetric covariance:
    // principalAngle = 0.5 * atan2(2c, a-b), in math convention (CCW from
    // east). Robust to c ≈ 0 (avoids the fragile eigenvector-from-eigenvalue
    // path which divides by tiny numbers). Convert to compass bearing
    // (CW from north): bearing = 90° − math_angle.
    double principalAngleDeg = 0.5 * Math.toDegrees(Math.atan2(2 * c, a - bb));
    double bearing = (90 - principalAngleDeg + 360) % 360;
    if (bearing >= 180) bearing -= 180; // canonical [0, 180), axis is bidirectional
    return new FrontierAxis(strength >= PHASE_2_1_STRONG_AXIS_RATIO, bearing, strength);
  }

  /** Phase 2.1: eigenvalue ratio above which we treat the reachable region
   *  as having a strong terrain axis. 3.0 corresponds to the reachable
   *  region being ~1.7x as elongated along the principal axis as
   *  perpendicular (sqrt(3) ≈ 1.73). Tunable; lower values fire more often. */
  static final double PHASE_2_1_STRONG_AXIS_RATIO = 3.0;

  /** Phase 2.1: half-angle (degrees) of the "near-perpendicular" cone.
   *  User direction within 30° of perpendicular to the axis triggers retry. */
  static final double PHASE_2_1_PERPENDICULAR_TOL = 30.0;

  /** Phase 2.1: whether a user-supplied bearing is within
   *  {@link #PHASE_2_1_PERPENDICULAR_TOL} of perpendicular to the given
   *  axis. Both arguments are in compass degrees; axis canonical [0, 180). */
  static boolean isPerpendicularToAxis(double userBearing, double axisBearing) {
    double userMod = ((userBearing % 180) + 180) % 180;
    double axisMod = ((axisBearing % 180) + 180) % 180;
    double diff = Math.abs(userMod - axisMod);
    if (diff > 90) diff = 180 - diff;
    return diff >= (90 - PHASE_2_1_PERPENDICULAR_TOL);
  }

  /** Phase 2.1: pick the axis-aligned bearing (axis or axis+180) whose
   *  half-plane is closer to the user's original direction. Used to retry
   *  with a direction that respects both terrain (axis) and rough user
   *  intent (the original half-plane). Tie-break: prefer the lower bearing. */
  static double chooseAxisBearing(double axisBearing, double userBearing) {
    double opt1 = ((axisBearing % 180) + 180) % 180;       // canonical axis
    double opt2 = (opt1 + 180) % 360;                       // opposing direction
    double user = ((userBearing % 360) + 360) % 360;
    double d1 = angularDiff(opt1, user);
    double d2 = angularDiff(opt2, user);
    if (d1 < d2) return opt1;
    if (d2 < d1) return opt2;
    return Math.min(opt1, opt2);
  }

  private static double angularDiff(double a, double b) {
    double d = Math.abs(a - b) % 360;
    return d > 180 ? 360 - d : d;
  }

  void buildPointsFromCircle(List<OsmNodeNamed> waypoints, double startAngle, double searchRadius, int points) {
    //startAngle -= 90;
    for (int i = 1; i < points; i++) {
      double anAngle = 90 - (180.0 * i / points);
      int[] pos = CheapRuler.destination(waypoints.get(0).ilon, waypoints.get(0).ilat, searchRadius, startAngle - anAngle);
      OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
      onn.name = "rt" + i;
      waypoints.add(onn);
    }

    OsmNodeNamed onn = new OsmNodeNamed(waypoints.get(0));
    onn.name = "to_rt";
    waypoints.add(onn);
  }

  int getRandomDirectionFromData(OsmNodeNamed wp, double searchRadius) {

    long start = System.currentTimeMillis();

    int preferredRandomType = 0;
    boolean consider_elevation = routingContext.expctxWay.getVariableValue("consider_elevation", 0f) == 1f;
    boolean consider_forest = routingContext.expctxWay.getVariableValue("consider_forest", 0f) == 1f;
    boolean consider_river = routingContext.expctxWay.getVariableValue("consider_river", 0f) == 1f;
    if (consider_elevation) {
      preferredRandomType = AreaInfo.RESULT_TYPE_ELEV50;
    } else if (consider_forest) {
      preferredRandomType = AreaInfo.RESULT_TYPE_GREEN;
    } else if (consider_river) {
      preferredRandomType = AreaInfo.RESULT_TYPE_RIVER;
    } else {
      return (int) (Math.random()*360);
    }

    MatchedWaypoint wpt1 = new MatchedWaypoint();
    wpt1.waypoint = wp;
    wpt1.name = "info";
    wpt1.radius = searchRadius * 1.5;

    List<AreaInfo> ais = new ArrayList<>();
    AreaReader areareader = new AreaReader();
    if (routingContext.rawAreaPath != null) {
      File fai = new File(routingContext.rawAreaPath);
      if (fai.exists()) {
        areareader.readAreaInfo(fai, wpt1, ais);
      }
    }

    if (ais.isEmpty()) {
      List<MatchedWaypoint> listStart = new ArrayList<>();
      listStart.add(wpt1);

      List<OsmNodeNamed> wpliststart = new ArrayList<>();
      wpliststart.add(wp);

      List<OsmNodeNamed> listOne = new ArrayList<>();

      for (int a = 45; a < 360; a += 90) {
        int[] pos = CheapRuler.destination(wp.ilon, wp.ilat, searchRadius * 1.5, a);
        OsmNodeNamed onn = new OsmNodeNamed(new OsmNode(pos[0], pos[1]));
        onn.name = "via" + a;
        listOne.add(onn);

        MatchedWaypoint wpt = new MatchedWaypoint();
        wpt.waypoint = onn;
        wpt.name = onn.name;
        listStart.add(wpt);
      }

      RoutingEngine re = null;
      RoutingContext rc = new RoutingContext();
      String name = routingContext.localFunction;
      int idx = name.lastIndexOf(File.separator);
      rc.localFunction = idx == -1 ? "dummy" : name.substring(0, idx + 1) + "dummy.brf";

      re = new RoutingEngine(null, null, segmentDir, wpliststart, rc, BROUTER_ENGINEMODE_ROUNDTRIP);
      rc.useDynamicDistance = true;
      re.matchWaypointsToNodes(listStart);
      re.resetCache(true);

      int numForest = rc.expctxWay.getLookupKey("estimated_forest_class");
      int numRiver = rc.expctxWay.getLookupKey("estimated_river_class");

      OsmNode start1 = re.nodesCache.getStartNode(listStart.get(0).node1.getIdFromPos());

      double elev = (start1 == null ? 0 : start1.getElev()); // listOne.get(0).crosspoint.getElev();

      int maxlon = Integer.MIN_VALUE;
      int minlon = Integer.MAX_VALUE;
      int maxlat = Integer.MIN_VALUE;
      int minlat = Integer.MAX_VALUE;
      for (OsmNodeNamed on : listOne) {
        maxlon = Math.max(on.ilon, maxlon);
        minlon = Math.min(on.ilon, minlon);
        maxlat = Math.max(on.ilat, maxlat);
        minlat = Math.min(on.ilat, minlat);
      }
      OsmNogoPolygon searchRect = new OsmNogoPolygon(true);
      searchRect.addVertex(maxlon, maxlat);
      searchRect.addVertex(maxlon, minlat);
      searchRect.addVertex(minlon, minlat);
      searchRect.addVertex(minlon, maxlat);

      for (int a = 0; a < 4; a++) {
        rc.ai = new AreaInfo(a * 90 + 90);
        rc.ai.elevStart = elev;
        rc.ai.numForest = numForest;
        rc.ai.numRiver = numRiver;

        rc.ai.polygon = new OsmNogoPolygon(true);
        rc.ai.polygon.addVertex(wp.ilon, wp.ilat);
        rc.ai.polygon.addVertex(listOne.get(a).ilon, listOne.get(a).ilat);
        if (a == 3)
          rc.ai.polygon.addVertex(listOne.get(0).ilon, listOne.get(0).ilat);
        else
          rc.ai.polygon.addVertex(listOne.get(a + 1).ilon, listOne.get(a + 1).ilat);

        ais.add(rc.ai);
      }

      int maxscale = Math.abs(searchRect.points.get(2).x - searchRect.points.get(0).x);
      maxscale = Math.max(1, Math.round(maxscale / 31250f / 2) + 1);

      areareader.getDirectAllData(segmentDir, rc, wp, maxscale, rc.expctxWay, searchRect, ais);

      if (routingContext.rawAreaPath != null) {
        try {
          wpt1.radius = searchRadius * 1.5;
          areareader.writeAreaInfo(routingContext.rawAreaPath, wpt1, ais);
        } catch (Exception e) {
        }
      }
      rc.ai = null;

    }

    logInfo("round trip execution time = " + (System.currentTimeMillis() - start) / 1000. + " seconds");

    // for (AreaInfo ai: ais) {
    //  System.out.println("\n" + ai.toString());
    //}

    switch (preferredRandomType) {
      case AreaInfo.RESULT_TYPE_ELEV50:
        Collections.sort(ais, new Comparator<>() {
          public int compare(AreaInfo o1, AreaInfo o2) {
            return o2.getElev50Weight() - o1.getElev50Weight();
          }
        });
        break;
      case AreaInfo.RESULT_TYPE_GREEN:
        Collections.sort(ais, new Comparator<>() {
          public int compare(AreaInfo o1, AreaInfo o2) {
            return o2.getGreen() - o1.getGreen();
          }
        });
        break;
      case AreaInfo.RESULT_TYPE_RIVER:
        Collections.sort(ais, new Comparator<>() {
          public int compare(AreaInfo o1, AreaInfo o2) {
            return o2.getRiver() - o1.getRiver();
          }
        });
        break;
      default:
        return (int) (Math.random()*360);
    }

    int angle = ais.get(0).direction;
    return angle - 30 + (int) (Math.random() * 60);
  }



  private void postElevationCheck(OsmTrack track) {
    OsmPathElement lastPt = null;
    OsmPathElement startPt = null;
    short lastElev = Short.MIN_VALUE;
    short startElev = Short.MIN_VALUE;
    short endElev = Short.MIN_VALUE;
    int startIdx = 0;
    int endIdx = -1;
    int dist = 0;
    int ourSize = track.nodes.size();
    for (int idx = 0; idx < ourSize; idx++) {
      OsmPathElement n = track.nodes.get(idx);
      if (n.getSElev() == Short.MIN_VALUE && lastElev != Short.MIN_VALUE && idx < ourSize - 1) {
        // start one point before entry point to get better elevation results
        if (idx > 1)
          startElev = track.nodes.get(idx - 2).getSElev();
        if (startElev == Short.MIN_VALUE)
          startElev = lastElev;
        startIdx = idx;
        startPt = lastPt;
        dist = 0;
        if (lastPt != null)
          dist += n.calcDistance(lastPt);
      } else if (n.getSElev() != Short.MIN_VALUE && lastElev == Short.MIN_VALUE && startElev != Short.MIN_VALUE) {
        // end one point behind exit point to get better elevation results
        if (idx + 1 < track.nodes.size())
          endElev = track.nodes.get(idx + 1).getSElev();
        if (endElev == Short.MIN_VALUE)
          endElev = n.getSElev();
        endIdx = idx;
        OsmPathElement tmpPt = track.nodes.get(startIdx > 1 ? startIdx - 2 : startIdx - 1);
        int diffElev = endElev - startElev;
        dist += tmpPt.calcDistance(startPt);
        dist += n.calcDistance(lastPt);
        int distRest = dist;
        double incline = diffElev / (dist / 100.);
        String lastMsg = "";
        double tmpincline = 0;
        double startincline = 0;
        double selev = track.nodes.get(startIdx > 1 ? startIdx - 2 : startIdx - 1).getSElev();
        boolean hasInclineTags = false;
        for (int i = startIdx - 1; i < endIdx + 1; i++) {
          OsmPathElement tmp = track.nodes.get(i);
          if (tmp.message != null) {
            MessageData md = tmp.message.copy();
            String msg = md.wayKeyValues;
            if (!msg.equals(lastMsg)) {
              boolean revers = msg.contains("reversedirection=yes");
              int pos = msg.indexOf("incline=");
              if (pos != -1) {
                hasInclineTags = true;
                String s = msg.substring(pos + 8);
                pos = s.indexOf(" ");
                if (pos != -1)
                  s = s.substring(0, pos);

                if (s.length() > 0) {
                  try {
                    int ind = s.indexOf("%");
                    if (ind != -1)
                      s = s.substring(0, ind);
                    ind = s.indexOf("°");
                    if (ind != -1)
                      s = s.substring(0, ind);
                    tmpincline = Double.parseDouble(s.trim());
                    if (revers)
                      tmpincline *= -1;
                  } catch (NumberFormatException e) {
                    tmpincline = 0;
                  }
                }
              } else {
                tmpincline = 0;
              }
              if (startincline == 0) {
                startincline = tmpincline;
              } else if (startincline < 0 && tmpincline > 0) {
                // for the way up find the exit point
                double diff = endElev - selev;
                tmpincline = diff / (distRest / 100.);
              }
            }
            lastMsg = msg;
          }
          int tmpdist = tmp.calcDistance(tmpPt);
          distRest -= tmpdist;
          if (hasInclineTags)
            incline = tmpincline;
          selev = (selev + (tmpdist / 100. * incline));
          tmp.setSElev((short) selev);
          tmp.message.ele = (short) selev;
          tmpPt = tmp;
        }
        dist = 0;
      } else if (n.getSElev() != Short.MIN_VALUE && lastElev == Short.MIN_VALUE && startIdx == 0) {
        // fill at start
        for (int i = 0; i < idx; i++) {
          track.nodes.get(i).setSElev(n.getSElev());
        }
      } else if (n.getSElev() == Short.MIN_VALUE && idx == track.nodes.size() - 1) {
        // fill at end
        startIdx = idx;
        for (int i = startIdx; i < track.nodes.size(); i++) {
          track.nodes.get(i).setSElev(lastElev);
        }
      } else if (n.getSElev() == Short.MIN_VALUE) {
        if (lastPt != null)
          dist += n.calcDistance(lastPt);
      }
      lastElev = n.getSElev();
      lastPt = n;
    }

  }

  private void logException(Throwable t) {
    errorMessage = t instanceof RuntimeException ? t.getMessage() : t.toString();
    logInfo("Error (linksProcessed=" + linksProcessed + " open paths: " + openSet.getSize() + "): " + errorMessage);
  }


  public void doSearch() {
    try {
      MatchedWaypoint seedPoint = new MatchedWaypoint();
      seedPoint.waypoint = waypoints.get(0);
      List<MatchedWaypoint> listOne = new ArrayList<>();
      listOne.add(seedPoint);
      matchWaypointsToNodes(listOne);

      findTrack("seededSearch", seedPoint, null, null, null, false);
    } catch (IllegalArgumentException e) {
      logException(e);
    } catch (Exception e) {
      logException(e);
      logThrowable(e);
    } catch (Error e) {
      cleanOnOOM();
      logException(e);
      logThrowable(e);
    } finally {
      ProfileCache.releaseProfile(routingContext);
      if (nodesCache != null) {
        nodesCache.close();
        nodesCache = null;
      }
      openSet.clear();
      finished = true; // this signals termination to outside

      if (infoLogWriter != null) {
        try {
          infoLogWriter.close();
        } catch (Exception e) {
        }
        infoLogWriter = null;
      }
    }
  }

  public void cleanOnOOM() {
    terminate();
  }

  private OsmTrack findTrack(OsmTrack[] refTracks, OsmTrack[] lastTracks) {
    for (; ; ) {
      try {
        return tryFindTrack(refTracks, lastTracks);
      } catch (RoutingIslandException rie) {
        if (routingContext.useDynamicDistance) {
          for (MatchedWaypoint mwp : matchedWaypoints) {
            if (mwp.name.contains("_add")) {
              long n1 = mwp.node1.getIdFromPos();
              long n2 = mwp.node2.getIdFromPos();
              islandNodePairs.addTempPair(n1, n2);
            }
          }
        }
        islandNodePairs.freezeTempPairs();
        nodesCache.clean(true);
        matchedWaypoints = null;
      }
    }
  }

  private OsmTrack tryFindTrack(OsmTrack[] refTracks, OsmTrack[] lastTracks) {
    OsmTrack totaltrack = new OsmTrack();
    int nUnmatched = waypoints.size();
    boolean hasDirectRouting = false;

    if (useNodePoints && extraWaypoints != null) {
      // add extra waypoints from the last broken round
      for (OsmNodeNamed wp : extraWaypoints) {
        if (wp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT) hasDirectRouting = true;
        if (wp.name.startsWith("from")) {
          waypoints.add(1, wp);
          waypoints.get(0).wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
          nUnmatched++;
        } else {
          waypoints.add(waypoints.size() - 1, wp);
          waypoints.get(waypoints.size() - 2).wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
          nUnmatched++;
        }
      }
      extraWaypoints = null;
    }
    if (lastTracks.length < waypoints.size() - 1) {
      refTracks = new OsmTrack[waypoints.size() - 1]; // used ways for alternatives
      lastTracks = new OsmTrack[waypoints.size() - 1];
      hasDirectRouting = true;
    }
    for (OsmNodeNamed wp : waypoints) {
      if (hasInfo()) logInfo("wp=" + wp + (wp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT ? " beeline" : (wp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_MEETING ? " via" : "")));
      if (wp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT) hasDirectRouting = true;
    }

    // check for a track for that target
    OsmTrack nearbyTrack = null;
    if (!hasDirectRouting && lastTracks[waypoints.size() - 2] == null) {
      StringBuilder debugInfo = hasInfo() ? new StringBuilder() : null;
      nearbyTrack = OsmTrack.readBinary(routingContext.rawTrackPath, waypoints.get(waypoints.size() - 1), routingContext.getNogoChecksums(), routingContext.profileTimestamp, debugInfo);
      if (nearbyTrack != null) {
        nUnmatched--;
      }
      if (hasInfo()) {
        boolean found = nearbyTrack != null;
        boolean dirty = found && nearbyTrack.isDirty;
        logInfo("read referenceTrack, found=" + found + " dirty=" + dirty + " " + debugInfo);
      }
    }

    if (matchedWaypoints == null) { // could exist from the previous alternative level
      matchedWaypoints = new ArrayList<>();
      for (int i = 0; i < nUnmatched; i++) {
        MatchedWaypoint mwp = new MatchedWaypoint();
        mwp.waypoint = waypoints.get(i);
        mwp.name = waypoints.get(i).name;
        mwp.wpttype = waypoints.get(i).wpttype;
        mwp.generated = waypoints.get(i).generated;
        matchedWaypoints.add(mwp);
      }
      int startSize = matchedWaypoints.size();
      matchWaypointsToNodes(matchedWaypoints);

      // filter bad round-trip waypoints after matching
      if (roundTripSearchRadius > 0) {
        int beforeFilter = matchedWaypoints.size();
        waypointSnapper().filterRoundTripWaypoints(matchedWaypoints);
        if (matchedWaypoints.size() != beforeFilter) {
          logInfo("filterRoundTrip: reduced waypoints from " + beforeFilter + " to " + matchedWaypoints.size());
          refTracks = new OsmTrack[matchedWaypoints.size() - 1];
          lastTracks = new OsmTrack[matchedWaypoints.size() - 1];
        }
        // Snap intermediate waypoints to nearest intersection to avoid mid-edge detour tails
        waypointSnapper().snapToIntersection(matchedWaypoints);
        // No-beeline invariant: round-trip routes must not contain DIRECT
        // segments. matchWaypointsToNodes flags DIRECT for points beyond
        // catchingRange; fail rather than emit a beeline in a successful loop.
        for (MatchedWaypoint mwp : matchedWaypoints) {
          if (mwp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT) {
            throw new IllegalArgumentException(
              "round-trip waypoint " + mwp.name + " could not be road-matched"
                + " (would force beeline segment); aborting");
          }
        }
      }

      if (startSize < matchedWaypoints.size()) {
        refTracks = new OsmTrack[matchedWaypoints.size() - 1]; // used ways for alternatives
        lastTracks = new OsmTrack[matchedWaypoints.size() - 1];
        hasDirectRouting = true;
      }

      // greedyLegTracks is indexed by leg position and only valid while the
      // matched-waypoint count is unchanged. If matching/filtering above added or
      // removed a waypoint, the leg-to-waypoint correspondence is broken, so drop
      // the corridor constraints rather than route through a misaligned leg track.
      if (greedyLegTracks != null && greedyLegTracks.length != matchedWaypoints.size() - 1) {
        logInfo("greedy leg tracks (" + greedyLegTracks.length + ") no longer match "
          + (matchedWaypoints.size() - 1) + " legs after matching/filtering; "
          + "dropping corridor constraints");
        greedyLegTracks = null;
      }

      for (MatchedWaypoint mwp : matchedWaypoints) {
        if (hasInfo() && matchedWaypoints.size() != nUnmatched)
          logInfo("new wp=" + mwp.waypoint + " " + mwp.crosspoint + (mwp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT ? " beeline" : (mwp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_MEETING ? " via" : "")));
      }

      routingContext.checkMatchedWaypointAgainstNogos(matchedWaypoints);

      // detect target islands: restricted search in inverse direction
      routingContext.inverseDirection = !routingContext.inverseRouting;
      airDistanceCostFactor = 0.;
      for (int i = 0; i < matchedWaypoints.size() - 1; i++) {
        nodeLimit = MAXNODES_ISLAND_CHECK;
        if (matchedWaypoints.get(i).wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT) continue;
        if (routingContext.inverseRouting) {
          OsmTrack seg = findTrack("start-island-check", matchedWaypoints.get(i), matchedWaypoints.get(i + 1), null, null, false);
          if (seg == null && nodeLimit > 0) {
            throw new IllegalArgumentException("start island detected for section " + i);
          }
        } else {
          OsmTrack seg = findTrack("target-island-check", matchedWaypoints.get(i + 1), matchedWaypoints.get(i), null, null, false);
          if (seg == null && nodeLimit > 0) {
            throw new IllegalArgumentException("target island detected for section " + i);
          }
        }
      }
      routingContext.inverseDirection = false;
      nodeLimit = 0;

      if (nearbyTrack != null) {
        matchedWaypoints.add(nearbyTrack.endPoint);
      }
    } else {
      if (lastTracks.length < matchedWaypoints.size() - 1) {
        refTracks = new OsmTrack[matchedWaypoints.size() - 1]; // used ways for alternatives
        lastTracks = new OsmTrack[matchedWaypoints.size() - 1];
        hasDirectRouting = true;
      }
    }
    for (MatchedWaypoint mwp : matchedWaypoints) {
      //System.out.println(FormatGpx.getWaypoint(mwp.waypoint.ilon, mwp.waypoint.ilat, mwp.name, null));
      //System.out.println(FormatGpx.getWaypoint(mwp.crosspoint.ilon, mwp.crosspoint.ilat, mwp.name+"_cp", null));
    }

    routingContext.hasDirectRouting = hasDirectRouting;

    // For roundtrip mode, accumulate all previous legs so each new leg
    // penalizes reuse of edges from earlier legs (similar to GraphHopper's
    // AvoidEdgesWeighting). BRouter's existing refTrack mechanism doubles
    // the cost of edges found in the refTrack, discouraging road reuse.
    OsmTrack roundTripPreviousLegs = (engineMode == BROUTER_ENGINEMODE_ROUNDTRIP) ? new OsmTrack() : null;

    OsmPath.seg = 1; // set segment counter
    for (int i = 0; i < matchedWaypoints.size() - 1; i++) {
      if (lastTracks[i] != null) {
        if (refTracks[i] == null) refTracks[i] = new OsmTrack();
        refTracks[i].addNodes(lastTracks[i]);
      }

      // In roundtrip mode, use accumulated previous legs as the refTrack
      // to discourage reusing roads from earlier legs of the loop.
      // Always create a fresh OsmTrack to avoid mutating refTracks[i] via alias.
      OsmTrack effectiveRefTrack;
      if (roundTripPreviousLegs != null && roundTripPreviousLegs.nodes != null
          && !roundTripPreviousLegs.nodes.isEmpty()) {
        effectiveRefTrack = new OsmTrack();
        if (refTracks[i] != null) {
          effectiveRefTrack.addNodes(refTracks[i]);
        }
        effectiveRefTrack.addNodes(roundTripPreviousLegs);
      } else {
        effectiveRefTrack = refTracks[i];
      }

      OsmTrack seg;
      int wptIndex;
      if (routingContext.inverseRouting) {
        routingContext.inverseDirection = true;
        seg = searchTrack(matchedWaypoints.get(i + 1), matchedWaypoints.get(i), null, effectiveRefTrack);
        routingContext.inverseDirection = false;
        wptIndex = i + 1;
      } else {
        OsmTrack legNearbyTrack = (greedyLegTracks != null && i < greedyLegTracks.length)
          ? greedyLegTracks[i]
          : (i == matchedWaypoints.size() - 2 ? nearbyTrack : null);
        if (legNearbyTrack != null && legNearbyTrack != nearbyTrack) {
          // Corridor-constrained routing: try with greedy leg track first,
          // fall back to unconstrained routing if it fails.
          try {
            seg = searchTrack(matchedWaypoints.get(i), matchedWaypoints.get(i + 1), legNearbyTrack, effectiveRefTrack);
          } catch (IllegalArgumentException e) {
            seg = null;
          }
          if (seg == null) {
            seg = searchTrack(matchedWaypoints.get(i), matchedWaypoints.get(i + 1), null, effectiveRefTrack);
          }
        } else {
          seg = searchTrack(matchedWaypoints.get(i), matchedWaypoints.get(i + 1), legNearbyTrack, effectiveRefTrack);
        }
        wptIndex = i;
        if (routingContext.continueStraight) {
          if (i < matchedWaypoints.size() - 2) {
            OsmNode lastPoint = seg.containsNode(matchedWaypoints.get(i+1).node1) ? matchedWaypoints.get(i+1).node1 : matchedWaypoints.get(i+1).node2;
            OsmNodeNamed nogo = new OsmNodeNamed(lastPoint);
            nogo.radius = 5;
            nogo.name = "nogo" + (i+1);
            nogo.nogoWeight = 9999.;
            nogo.isNogo = true;
            if (routingContext.nogopoints == null) routingContext.nogopoints = new ArrayList<>();
            routingContext.nogopoints.add(nogo);
          }
        }
      }
      if (seg == null)
        return null;

      if (routingContext.ai != null) return null;

      boolean changed = false;
      if (routingContext.correctMisplacedViaPoints &&
          matchedWaypoints.get(i).wpttype != MatchedWaypoint.WAYPOINT_TYPE_DIRECT &&
          matchedWaypoints.get(i).wpttype != MatchedWaypoint.WAYPOINT_TYPE_MEETING &&
          !routingContext.allowSamewayback) {
        changed = snapPathConnection(totaltrack, seg, routingContext.inverseRouting ? matchedWaypoints.get(i + 1) : matchedWaypoints.get(i));
      }
      if (wptIndex > 0)
        matchedWaypoints.get(wptIndex).indexInTrack = totaltrack.nodes.size() - 1;

      totaltrack.appendTrack(seg);
      lastTracks[i] = seg;

      // Accumulate this leg for roundtrip edge-avoidance on subsequent legs
      if (roundTripPreviousLegs != null) {
        roundTripPreviousLegs.addNodes(seg);
      }
    }

    postElevationCheck(totaltrack);

    if (engineMode == BROUTER_ENGINEMODE_ROUNDTRIP) {
      // allowSamewayback is an out-and-back: it intentionally retraces the outbound leg.
      // Back-and-forth/micro-detour removal would see the two legs as an overlap and delete
      // one of them, leaving a one-way segment that no longer closes — so skip it here.
      // (This also affected loops that reduced to a single intermediate waypoint.)
      //
      // explicit-via round-trip mode hits the same problem: the closing waypoint sits at
      // the same position as the start, so crow-fly between the first and last matched
      // waypoint is 0 and removeMicroDetours sees the entire route as a "micro detour"
      // and deletes it. User-via routes are also shape-preserving by intent — the user
      // picked exact via points and does not want the engine to micro-edit them away.
      if (!routingContext.allowSamewayback && !explicitViaRoundTrip) {
        trackCleanup().removeBackAndForthSegments(totaltrack, matchedWaypoints);
        trackCleanup().removeMicroDetours(totaltrack, 1500, matchedWaypoints);
        // Same artifact-repair chain as the greedy adoption path
        // (finalizeAdoptedRoundTripTrack): probe/isochrone are fast fallback
        // algorithms worth keeping, and their generated "rt*" waypoints suffer
        // the same via-pinned bulges and near-revisit petals. Both passes
        // recognize rt-named waypoints as generated and carry the full guard
        // set (user-via protection, distance floor, crossing guard).
        waypointSnapper().repairViaPinnedBulges(totaltrack, matchedWaypoints);
        trackCleanup().removeArtifactSpurSpans(totaltrack, matchedWaypoints);
      } else if (!routingContext.allowSamewayback && explicitViaRoundTrip && routingContext.explicitViaDensify) {
        // Densified explicit-via: strip the out-and-back spurs at GENERATED bulge points
        // only — never user vias. removeMicroDetours is still skipped (it would
        // delete the whole route, since the closing waypoint coincides with the start).
        // (Cleaning ALL waypoint spurs was tested and rejected: it did not fix the
        // leg-hostile cases and shortened load-bearing retraces on 1-via loops.)
        trackCleanup().removeBackAndForthSegments(totaltrack, matchedWaypoints, true);
      }
      // removeBackAndForthSegments/removeMicroDetours edit the nodes list in place but
      // leave each node's origin back-pointer dangling through the removed nodes.
      // processVoiceHints() walks the origin chain (not the list), so a chain longer
      // than the list drives its node counter negative — producing voice hints with
      // negative indexInTrack and stale, out-of-range turn angles at the loop seam.
      // Relink origins to the surviving list order to restore the chain == list invariant.
      trackCleanup().rebuildOriginChain(totaltrack);
    }

    recalcTrack(totaltrack);

    matchedWaypoints.get(matchedWaypoints.size() - 1).indexInTrack = totaltrack.nodes.size() - 1;
    totaltrack.matchedWaypoints = matchedWaypoints;
    totaltrack.processVoiceHints(routingContext);
    if (engineMode == BROUTER_ENGINEMODE_ROUNDTRIP) {
      consolidateRoundTripVoiceHints(totaltrack);
    }
    totaltrack.prepareSpeedProfile(routingContext);

    totaltrack.showTime = routingContext.showTime;
    totaltrack.params = routingContext.keyValues;

    if (routingContext.poipoints != null)
      totaltrack.pois = routingContext.poipoints;

    return totaltrack;
  }

  OsmTrack getExtraSegment(OsmPathElement start, OsmPathElement end) {

    if (start == null || end == null) return null;

    List<MatchedWaypoint> wptlist = new ArrayList<>();
    MatchedWaypoint wpt1 = new MatchedWaypoint();
    wpt1.waypoint = new OsmNode(start.getILon(), start.getILat());
    wpt1.name = "wptx1";
    wpt1.crosspoint = new OsmNode(start.getILon(), start.getILat());
    wpt1.node1 = new OsmNode(start.getILon(), start.getILat());
    wpt1.node2 = new OsmNode(end.getILon(), end.getILat());
    wptlist.add(wpt1);
    MatchedWaypoint wpt2 = new MatchedWaypoint();
    wpt2.waypoint = new OsmNode(end.getILon(), end.getILat());
    wpt2.name = "wptx2";
    wpt2.crosspoint = new OsmNode(end.getILon(), end.getILat());
    wpt2.node2 = new OsmNode(start.getILon(), start.getILat());
    wpt2.node1 = new OsmNode(end.getILon(), end.getILat());
    wptlist.add(wpt2);

    MatchedWaypoint mwp1 = wptlist.get(0);
    MatchedWaypoint mwp2 = wptlist.get(1);

    OsmTrack mid = null;

    boolean corr = routingContext.correctMisplacedViaPoints;
    routingContext.correctMisplacedViaPoints = false;

    guideTrack = new OsmTrack();
    guideTrack.addNode(start);
    guideTrack.addNode(end);

    mid = findTrack("getinfo", mwp1, mwp2, null, null, false);

    guideTrack = null;
    routingContext.correctMisplacedViaPoints = corr;

    return mid;
  }

  private int snapRoundaboutConnection(OsmTrack tt, OsmTrack t, int indexStart, int indexEnd, int indexMeeting, MatchedWaypoint startWp) {

    int indexMeetingBack = (indexMeeting == -1 ? tt.nodes.size() - 1 : indexMeeting);
    int indexMeetingFore = 0;
    int indexStartBack = indexStart;
    int indexStartFore = 0;

    OsmPathElement ptStart = tt.nodes.get(indexStartBack);
    OsmPathElement ptMeeting = tt.nodes.get(indexMeetingBack);
    OsmPathElement ptEnd = t.nodes.get(indexEnd);

    boolean bMeetingIsOnRoundabout = ptMeeting.message.isRoundabout();
    boolean bMeetsRoundaboutStart = false;
    int wayDistance = 0;

    int i;
    OsmPathElement last_n = null;

    for (i = 0; i < indexEnd; i++) {
      OsmPathElement n = t.nodes.get(i);
      if (last_n != null) wayDistance += n.calcDistance(last_n);
      last_n = n;
      if (n.positionEquals(ptStart)) {
        indexStartFore = i;
        bMeetsRoundaboutStart = true;
      }
      if (n.positionEquals(ptMeeting)) {
        indexMeetingFore = i;
      }

    }

    if (routingContext.correctMisplacedViaPointsDistance > 0 &&
      wayDistance > routingContext.correctMisplacedViaPointsDistance) {
      return 0;
    }

    if (!bMeetsRoundaboutStart && bMeetingIsOnRoundabout) {
      indexEnd = indexMeetingFore;
    }
    if (bMeetsRoundaboutStart && bMeetingIsOnRoundabout) {
      indexEnd = indexStartFore;
    }

    List<OsmPathElement> removeList = new ArrayList<>();
    if (!bMeetsRoundaboutStart) {
      indexStartBack = indexMeetingBack;
      while (!tt.nodes.get(indexStartBack).message.isRoundabout()) {
        indexStartBack--;
        if (indexStartBack == 2) break;
      }
    }

    for (i = indexStartBack + 1; i < tt.nodes.size(); i++) {
      OsmPathElement n = tt.nodes.get(i);
      OsmTrack.OsmPathElementHolder detours = tt.getFromDetourMap(n.getIdFromPos());
      if (detours != null) {
        OsmTrack.OsmPathElementHolder h = detours;
        while (h != null) {
          h = h.nextHolder;
        }
      }
      removeList.add(n);
    }

    OsmPathElement ttend = null;
    if (!bMeetingIsOnRoundabout && !bMeetsRoundaboutStart) {
      ttend = tt.nodes.get(indexStartBack);
      OsmTrack.OsmPathElementHolder ttend_detours = tt.getFromDetourMap(ttend.getIdFromPos());
      if (ttend_detours != null) {
        tt.registerDetourForId(ttend.getIdFromPos(), null);
      }
    }

    for (OsmPathElement e : removeList) {
      tt.nodes.remove(e);
    }
    removeList.clear();


    for (i = 0; i < indexEnd; i++) {
      OsmPathElement n = t.nodes.get(i);
      if (n.positionEquals(bMeetsRoundaboutStart ? ptStart : ptEnd)) break;
      if (!bMeetingIsOnRoundabout && !bMeetsRoundaboutStart && n.message.isRoundabout()) break;

      OsmTrack.OsmPathElementHolder detours = t.getFromDetourMap(n.getIdFromPos());
      if (detours != null) {
        OsmTrack.OsmPathElementHolder h = detours;
        while (h != null) {
          h = h.nextHolder;
        }
      }
      removeList.add(n);
    }

    // time hold
    float atime = 0;
    float aenergy = 0;
    int acost = 0;
    if (i > 1) {
      atime = t.nodes.get(i).getTime();
      aenergy = t.nodes.get(i).getEnergy();
      acost = t.nodes.get(i).cost;
    }

    for (OsmPathElement e : removeList) {
      t.nodes.remove(e);
    }
    removeList.clear();

    if (atime > 0f) {
      for (OsmPathElement e : t.nodes) {
        e.setTime(e.getTime() - atime);
        e.setEnergy(e.getEnergy() - aenergy);
        e.cost = e.cost - acost;
      }
    }

    if (!bMeetingIsOnRoundabout && !bMeetsRoundaboutStart) {

      OsmTrack.OsmPathElementHolder ttend_detours = tt.getFromDetourMap(ttend.getIdFromPos());

      OsmTrack mid = null;
      if (ttend_detours != null && ttend_detours.node != null) {
        mid = getExtraSegment(ttend, ttend_detours.node);
      }
      OsmPathElement tt_end = tt.nodes.get(tt.nodes.size() - 1);

      int last_cost = tt_end.cost;
      float last_time = tt_end.getTime();
      float last_energy = tt_end.getEnergy();
      int tmp_cost = 0;
      float tmp_time = 0f;
      float tmp_energy = 0f;

      if (mid != null) {
        boolean start = false;
        for (OsmPathElement e : mid.nodes) {
          if (start) {
            if (e.positionEquals(ttend_detours.node)) {
              tmp_cost = e.cost;
              tmp_time = e.getTime();
              tmp_energy = e.getEnergy();
              break;
            }
            e.cost = last_cost + e.cost;
            e.setTime(last_time + e.getTime());
            e.setEnergy(last_energy + e.getEnergy());
            tt.nodes.add(e);
          }
          if (e.positionEquals(tt_end)) start = true;
        }

        ttend_detours.node.cost = last_cost + tmp_cost;
        ttend_detours.node.setTime(last_time + tmp_time);
        ttend_detours.node.setEnergy(last_energy + tmp_energy);
        tt.nodes.add(ttend_detours.node);
        t.nodes.add(0, ttend_detours.node);
      }

    }

    tt.cost = tt.nodes.get(tt.nodes.size()-1).cost;
    t.cost = t.nodes.get(t.nodes.size()-1).cost;

    startWp.correctedpoint = new OsmNode(ptStart.getILon(), ptStart.getILat());

    return (t.nodes.size());
  }

  // check for way back on way point
  private boolean snapPathConnection(OsmTrack tt, OsmTrack t, MatchedWaypoint startWp) {
    if (!startWp.name.startsWith("via") && !startWp.name.startsWith("rt"))
      return false;

    int ourSize = tt.nodes.size();
    if (ourSize > 0) {
      OsmPathElement testPoint = tt.nodes.get(ourSize - 1);
      if (routingContext.poipoints != null) {
        for (OsmNodeNamed node : routingContext.poipoints) {

          int lon0 = tt.nodes.get(ourSize - 2).getILon();
          int lat0 = tt.nodes.get(ourSize - 2).getILat();
          int lon1 = startWp.crosspoint.ilon;
          int lat1 = startWp.crosspoint.ilat;
          int lon2 = node.ilon;
          int lat2 = node.ilat;
          double angle3 = routingContext.anglemeter.calcAngle(lon0, lat0, lon1, lat1, lon2, lat2);
          int dist = node.calcDistance(startWp.crosspoint);
          if (dist < routingContext.waypointCatchingRange)
            return false;
        }
      }
      List<OsmPathElement> removeBackList = new ArrayList<>();
      List<OsmPathElement> removeForeList = new ArrayList<>();
      List<Integer> removeVoiceHintList = new ArrayList<>();
      OsmPathElement last = null;
      OsmPathElement lastJunction = null;
      CompactLongMap<OsmTrack.OsmPathElementHolder> lastJunctions = new CompactLongMap<>();
      OsmPathElement newJunction = null;
      OsmPathElement newTarget = null;
      OsmPathElement tmpback = null;
      OsmPathElement tmpfore = null;
      OsmPathElement tmpStart = null;
      int indexback = ourSize - 1;
      int indexfore = 0;
      int stop = (indexback - MAX_STEPS_CHECK > 1 ? indexback - MAX_STEPS_CHECK : 1);
      double wayDistance = 0;
      double nextDist = 0;
      boolean bCheckRoundAbout = false;
      boolean bBackRoundAbout = false;
      boolean bForeRoundAbout = false;
      int indexBackFound = 0;
      int indexForeFound = 0;
      int differentLanePoints = 0;
      int indexMeeting = -1;
      while (indexback >= 1 && indexback >= stop && indexfore < t.nodes.size()) {
        tmpback = tt.nodes.get(indexback);
        tmpfore = t.nodes.get(indexfore);
        if (!bBackRoundAbout && tmpback.message != null && tmpback.message.isRoundabout()) {
          bBackRoundAbout = true;
          indexBackFound = indexfore;
        }
        if (!bForeRoundAbout &&
           tmpfore.message != null && tmpfore.message.isRoundabout() ||
          (tmpback.positionEquals(tmpfore) && tmpback.message.isRoundabout())) {
          bForeRoundAbout = true;
          indexForeFound = indexfore;
        }
        if (indexfore == 0) {
          tmpStart = t.nodes.get(0);
        } else {
          double dirback = CheapAngleMeter.getDirection(tmpStart.getILon(), tmpStart.getILat(), tmpback.getILon(), tmpback.getILat());
          double dirfore = CheapAngleMeter.getDirection(tmpStart.getILon(), tmpStart.getILat(), tmpfore.getILon(), tmpfore.getILat());
          double dirdiff = CheapAngleMeter.getDifferenceFromDirection(dirback, dirfore);
          // walking wrong direction
          if (dirdiff > 60 && !bBackRoundAbout && !bForeRoundAbout) break;
        }
        // seems no roundabout, only on one end
        if (bBackRoundAbout != bForeRoundAbout && indexfore - Math.abs(indexForeFound - indexBackFound) > 8) break;
        if (!tmpback.positionEquals(tmpfore)) differentLanePoints++;
        if (tmpback.positionEquals(tmpfore)) indexMeeting = indexback;
        bCheckRoundAbout = bBackRoundAbout && bForeRoundAbout;
        if (bCheckRoundAbout) break;
        indexback--;
        indexfore++;
      }
      //System.out.println("snap round result " + indexback + ": " + bBackRoundAbout + " - " + indexfore + "; " + bForeRoundAbout + " pts " + differentLanePoints);
      if (bCheckRoundAbout) {

        tmpback = tt.nodes.get(--indexback);
        while (tmpback.message != null && tmpback.message.isRoundabout()) {
          tmpback = tt.nodes.get(--indexback);
        }

        int ifore = ++indexfore;
        OsmPathElement testfore = t.nodes.get(ifore);
        while (ifore < t.nodes.size() && testfore.message != null && testfore.message.isRoundabout()) {
          testfore = t.nodes.get(ifore);
          ifore++;
        }

        snapRoundaboutConnection(tt, t, indexback, --ifore, indexMeeting, startWp);

        // remove filled arrays
        removeVoiceHintList.clear();
        removeBackList.clear();
        removeForeList.clear();
        return true;
      }
      indexback = ourSize - 1;
      indexfore = 0;
      while (indexback >= 1 && indexback >= stop && indexfore < t.nodes.size()) {
        int junctions = 0;
        tmpback = tt.nodes.get(indexback);
        tmpfore = t.nodes.get(indexfore);
        if (tmpback.message != null && tmpback.message.isRoundabout()) {
          bCheckRoundAbout = true;
        }
        if (tmpfore.message != null && tmpfore.message.isRoundabout()) {
          bCheckRoundAbout = true;
        }
        {

          int dist = tmpback.calcDistance(tmpfore);
          OsmTrack.OsmPathElementHolder detours = tt.getFromDetourMap(tmpback.getIdFromPos());
          OsmTrack.OsmPathElementHolder h = detours;
          while (h != null) {
            junctions++;
            lastJunctions.put(h.node.getIdFromPos(), h);
            h = h.nextHolder;
          }

          if (dist == 1 && indexfore > 0) {
            if (indexfore == 1) {
              removeBackList.add(tt.nodes.get(tt.nodes.size() - 1)); // last and first should be equal, so drop only on second also equal
              removeForeList.add(t.nodes.get(0));
              removeBackList.add(tmpback);
              removeForeList.add(tmpfore);
              removeVoiceHintList.add(tt.nodes.size() - 1);
              removeVoiceHintList.add(indexback);
            } else {
              removeBackList.add(tmpback);
              removeForeList.add(tmpfore);
              removeVoiceHintList.add(indexback);
            }
            nextDist = t.nodes.get(indexfore - 1).calcDistance(tmpfore);
            wayDistance += nextDist;

          }
          if (dist > 1 || indexback == 1) {
            if (removeBackList.size() != 0) {
              // recover last - should be the cross point
              removeBackList.remove(removeBackList.get(removeBackList.size() - 1));
              removeForeList.remove(removeForeList.get(removeForeList.size() - 1));
              break;
            } else {
              return false;
            }
          }
          indexback--;
          indexfore++;

          if (routingContext.correctMisplacedViaPointsDistance > 0 &&
            wayDistance > routingContext.correctMisplacedViaPointsDistance) {
            removeVoiceHintList.clear();
            removeBackList.clear();
            removeForeList.clear();
            return false;
          }
        }
      }


      // time hold
      float atime = 0;
      float aenergy = 0;
      int acost = 0;
      if (removeForeList.size() > 1) {
        atime = t.nodes.get(indexfore -1).getTime();
        aenergy = t.nodes.get(indexfore -1).getEnergy();
        acost = t.nodes.get(indexfore -1).cost;
      }

      for (OsmPathElement e : removeBackList) {
        tt.nodes.remove(e);
      }
      for (OsmPathElement e : removeForeList) {
        t.nodes.remove(e);
      }
      for (Integer e : removeVoiceHintList) {
        tt.removeVoiceHint(e);
      }
      removeVoiceHintList.clear();
      removeBackList.clear();
      removeForeList.clear();

      if (atime > 0f) {
        for (OsmPathElement e : t.nodes) {
          e.setTime(e.getTime() - atime);
          e.setEnergy(e.getEnergy() - aenergy);
          e.cost = e.cost - acost;
        }
      }

      if (t.nodes.size() < 2)
        return true;
      if (tt.nodes.size() < 1)
        return true;
      if (tt.nodes.size() == 1) {
        last = tt.nodes.get(0);
      } else {
        last = tt.nodes.get(tt.nodes.size() - 2);
      }
      newJunction = t.nodes.get(0);
      newTarget = t.nodes.get(1);

      tt.cost = tt.nodes.get(tt.nodes.size()-1).cost;
      t.cost = t.nodes.get(t.nodes.size()-1).cost;

      // fill to correctedpoint
      startWp.correctedpoint = new OsmNode(newJunction.getILon(), newJunction.getILat());

      return true;
    }
    return false;
  }

  private void recalcTrack(OsmTrack t) {
    int totaldist = 0;
    int totaltime = 0;
    float lasttime = 0;
    float lastenergy = 0;
    float speed_min = 9999;
    Map<Integer, Integer> directMap = new HashMap<>();
    float tmptime = 1;
    float speed = 1;
    int dist;
    double angle;

    double ascend = 0;
    double ehb = 0.;
    int ourSize = t.nodes.size();

    short ele_start = Short.MIN_VALUE;
    short ele_end = Short.MIN_VALUE;
    double eleFactor = routingContext.inverseRouting ? 0.25 : -0.25;

    for (int i = 0; i < ourSize; i++) {
      OsmPathElement n = t.nodes.get(i);
      if (n.message == null) n.message = new MessageData();
      OsmPathElement nLast = null;
      if (i == 0) {
        angle = 0;
        dist = 0;
      } else if (i == 1) {
        angle = 0;
        nLast = t.nodes.get(0);
        dist = nLast.calcDistance(n);
      } else {
        int lon0 = t.nodes.get(i - 2).getILon();
        int lat0 = t.nodes.get(i - 2).getILat();
        int lon1 = t.nodes.get(i - 1).getILon();
        int lat1 = t.nodes.get(i - 1).getILat();
        int lon2 = t.nodes.get(i).getILon();
        int lat2 = t.nodes.get(i).getILat();
        angle = routingContext.anglemeter.calcAngle(lon0, lat0, lon1, lat1, lon2, lat2);
        nLast = t.nodes.get(i - 1);
        dist = nLast.calcDistance(n);
      }
      n.message.linkdist = dist;
      n.message.turnangle = (float) angle;
      totaldist += dist;
      totaltime += n.getTime();
      tmptime = (n.getTime() - lasttime);
      if (dist > 0) {
        speed = dist / tmptime * 3.6f;
        speed_min = Math.min(speed_min, speed);
      }
      if (tmptime == 1.f) { // no time used here
        directMap.put(i, dist);
      }

      lastenergy = n.getEnergy();
      lasttime = n.getTime();

      short ele = n.getSElev();
      if (ele != Short.MIN_VALUE)
        ele_end = ele;
      if (ele_start == Short.MIN_VALUE)
        ele_start = ele;

      if (nLast != null) {
        short ele_last = nLast.getSElev();
        if (ele_last != Short.MIN_VALUE) {
          ehb = ehb + (ele_last - ele) * eleFactor;
        }
        double filter = elevationFilter(n);
        if (ehb > 0) {
          ascend += ehb;
          ehb = 0;
        } else if (ehb < filter) {
          ehb = filter;
        }
      }

    }

    t.ascend = (int) ascend;
    t.plainAscend = (int) ((ele_start - ele_end) * eleFactor + 0.5);

    t.distance = totaldist;
    //t.energy = totalenergy;

    SortedSet<Integer> keys = new TreeSet<>(directMap.keySet());
    for (Integer key : keys) {
      int value = directMap.get(key);
      float addTime = (value / (speed_min / 3.6f));

      double addEnergy = 0;
      if (key > 0) {
        double GRAVITY = 9.81;  // in meters per second^(-2)
        double incline = (t.nodes.get(key - 1).getSElev() == Short.MIN_VALUE || t.nodes.get(key).getSElev() == Short.MIN_VALUE ? 0 : (t.nodes.get(key - 1).getElev() - t.nodes.get(key).getElev()) / value);
        double f_roll = routingContext.totalMass * GRAVITY * (routingContext.defaultC_r + incline);
        double spd = speed_min / 3.6;
        addEnergy = value * (routingContext.S_C_x * spd * spd + f_roll);
      }
      for (int j = key; j < ourSize; j++) {
        OsmPathElement n = t.nodes.get(j);
        n.setTime(n.getTime() + addTime);
        n.setEnergy(n.getEnergy() + (float) addEnergy);
      }
    }
    t.energy = (int) t.nodes.get(t.nodes.size() - 1).getEnergy();

    logInfo("track-length total = " + t.distance);
    logInfo("filtered ascend = " + t.ascend);
  }

  /**
   * find the elevation type for position
   * to determine the filter value
   *
   * @param n  the point
   * @return  the filter value for 1sec / 3sec elevation source
   */
  double elevationFilter(OsmPos n) {
    if (nodesCache != null) {
      int r = nodesCache.getElevationType(n.getILon(), n.getILat());
      if (r == 1) return -5.;
    }
    return -10.;
  }

  // geometric position matching finding the nearest routable way-section
  void matchWaypointsToNodes(List<MatchedWaypoint> unmatchedWaypoints) {
    resetCache(false);
    boolean useDynamicDistance = routingContext.useDynamicDistance;
    boolean bAddBeeline = routingContext.buildBeelineOnRange;
    double range = routingContext.waypointCatchingRange;
    boolean ok = nodesCache.matchWaypointsToNodes(unmatchedWaypoints, range, islandNodePairs);
    if (!ok && useDynamicDistance) {
      logInfo("second check for way points");
      resetCache(false);
      range = -MAX_DYNAMIC_RANGE;
      List<MatchedWaypoint> tmp = new ArrayList<>();
      for (MatchedWaypoint mwp : unmatchedWaypoints) {
        if (mwp.crosspoint == null || mwp.radius >= routingContext.waypointCatchingRange)
          tmp.add(mwp);
      }
      ok = nodesCache.matchWaypointsToNodes(tmp, range, islandNodePairs);
    }
    if (!ok) {
      for (MatchedWaypoint mwp : unmatchedWaypoints) {
        if (mwp.crosspoint == null)
          throw new IllegalArgumentException(mwp.name + "-position not mapped in existing datafile");
      }
    }
    // add beeline points when not already done
    if (useDynamicDistance && !useNodePoints && bAddBeeline) {
      List<MatchedWaypoint> waypoints = new ArrayList<>();
      for (int i = 0; i < unmatchedWaypoints.size(); i++) {
        MatchedWaypoint wp = unmatchedWaypoints.get(i);
        if (wp.waypoint.calcDistance(wp.crosspoint) > routingContext.waypointCatchingRange) {

          MatchedWaypoint nmw = new MatchedWaypoint();
          if (i == 0) {
            OsmNodeNamed onn = new OsmNodeNamed(wp.waypoint);
            onn.name = "from";
            nmw.waypoint = onn;
            nmw.name = onn.name;
            nmw.crosspoint = new OsmNode(wp.waypoint.ilon, wp.waypoint.ilat);
            nmw.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
            onn = new OsmNodeNamed(wp.crosspoint);
            onn.name = wp.name + "_add";
            wp.waypoint = onn;
            waypoints.add(nmw);
            wp.name = wp.name + "_add";
            waypoints.add(wp);
          } else {
            OsmNodeNamed onn = new OsmNodeNamed(wp.crosspoint);
            onn.name = wp.name + "_add";
            nmw.waypoint = onn;
            nmw.crosspoint = new OsmNode(wp.crosspoint.ilon, wp.crosspoint.ilat);
            nmw.node1 = new OsmNode(wp.node1.ilon, wp.node1.ilat);
            nmw.node2 = new OsmNode(wp.node2.ilon, wp.node2.ilat);
            nmw.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;

            if (wp.name != null) nmw.name = wp.name;
            waypoints.add(nmw);
            wp.name = wp.name + "_add";
            waypoints.add(wp);
            if (wp.name.startsWith("via")) {
              wp.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
              MatchedWaypoint emw = new MatchedWaypoint();
              OsmNodeNamed onn2 = new OsmNodeNamed(wp.crosspoint);
              onn2.name = wp.name + "_2";
              emw.name = onn2.name;
              emw.waypoint = onn2;
              emw.crosspoint = new OsmNode(nmw.crosspoint.ilon, nmw.crosspoint.ilat);
              emw.node1 = new OsmNode(nmw.node1.ilon, nmw.node1.ilat);
              emw.node2 = new OsmNode(nmw.node2.ilon, nmw.node2.ilat);
              emw.wpttype = MatchedWaypoint.WAYPOINT_TYPE_SHAPING;
              waypoints.add(emw);
            }
            wp.crosspoint = new OsmNode(wp.waypoint.ilon, wp.waypoint.ilat);
          }
        } else {
          waypoints.add(wp);
        }
      }
      unmatchedWaypoints.clear();
      unmatchedWaypoints.addAll(waypoints);
    }

  }

  private OsmTrack searchTrack(MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack nearbyTrack, OsmTrack refTrack) {
    // remove nogos with waypoints inside
    try {
      boolean calcBeeline = startWp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT;

      if (!calcBeeline)
        return searchRoutedTrack(startWp, endWp, nearbyTrack, refTrack);

      // we want a beeline-segment
      OsmPath path = routingContext.createPath(new OsmLink(null, startWp.crosspoint));
      path = routingContext.createPath(path, new OsmLink(startWp.crosspoint, endWp.crosspoint), null, false);
      return compileTrack(path, false);
    } finally {
      routingContext.restoreNogoList();
    }
  }

  private OsmTrack searchRoutedTrack(MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack nearbyTrack, OsmTrack refTrack) {
    OsmTrack track = null;
    double[] airDistanceCostFactors = new double[]{
      routingContext.pass1coefficient,
      routingContext.pass2coefficient
    };
    boolean isDirty = false;
    IllegalArgumentException dirtyMessage = null;

    if (nearbyTrack != null) {
      airDistanceCostFactor = 0.;
      try {
        track = findTrack("re-routing", startWp, endWp, nearbyTrack, refTrack, true);
      } catch (IllegalArgumentException iae) {
        if (terminated) throw iae;

        // fast partial recalcs: if that timed out, but we had a match,
        // build the concatenation from the partial and the nearby track
        if (matchPath != null) {
          track = mergeTrack(matchPath, nearbyTrack);
          isDirty = true;
          dirtyMessage = iae;
          logInfo("using fast partial recalc");
        }
        if (maxRunningTime > 0) {
          maxRunningTime += System.currentTimeMillis() - startTime; // reset timeout...
        }
      }
    }

    if (track == null) {
      for (int cfi = 0; cfi < airDistanceCostFactors.length; cfi++) {
        if (cfi > 0) lastAirDistanceCostFactor = airDistanceCostFactors[cfi - 1];
        airDistanceCostFactor = airDistanceCostFactors[cfi];

        if (airDistanceCostFactor < 0.) {
          continue;
        }

        OsmTrack t;
        try {
          t = findTrack(cfi == 0 ? "pass0" : "pass1", startWp, endWp, track, refTrack, false);
          if (routingContext.ai != null) return t;
        } catch (IllegalArgumentException iae) {
          if (!terminated && matchPath != null) { // timeout, but eventually prepare a dirty ref track
            logInfo("supplying dirty reference track after timeout");
            foundRawTrack = mergeTrack(matchPath, track);
            foundRawTrack.endPoint = endWp;
            foundRawTrack.nogoChecksums = routingContext.getNogoChecksums();
            foundRawTrack.profileTimestamp = routingContext.profileTimestamp;
            foundRawTrack.isDirty = true;
          }
          throw iae;
        }

        if (t == null && track != null && matchPath != null) {
          // ups, didn't find it, use a merge
          t = mergeTrack(matchPath, track);
          logInfo("using sloppy merge cause pass1 didn't reach destination");
        }
        if (t != null) {
          track = t;
        } else {
          throw new IllegalArgumentException("no track found at pass=" + cfi);
        }
      }
    }
    if (track == null) throw new IllegalArgumentException("no track found");

    OsmPathElement lastElement = null;

    boolean wasClean = nearbyTrack != null && !nearbyTrack.isDirty;
    if (refTrack == null && !(wasClean && isDirty)) { // do not overwrite a clean with a dirty track
      logInfo("supplying new reference track, dirty=" + isDirty);
      track.endPoint = endWp;
      track.nogoChecksums = routingContext.getNogoChecksums();
      track.profileTimestamp = routingContext.profileTimestamp;
      track.isDirty = isDirty;
      foundRawTrack = track;
    }

    if (!wasClean && isDirty) {
      throw dirtyMessage;
    }

    // final run for verbose log info and detail nodes
    airDistanceCostFactor = 0.;
    lastAirDistanceCostFactor = 0.;
    guideTrack = track;
    startTime = System.currentTimeMillis(); // reset timeout...
    try {
      OsmTrack tt = findTrack("re-tracking", startWp, endWp, null, refTrack, false);
      if (tt == null) throw new IllegalArgumentException("error re-tracking track");
      return tt;
    } finally {
      guideTrack = null;
    }
  }


  /**
   * Re-tracking pass that takes a raw track produced by single-pass
   * {@link #findTrack} and produces a detailed copy with per-edge
   * {@code MessageData} populated. The same pass that
   * {@link #searchRoutedTrack} runs internally as its final step,
   * exposed for the round-trip planner which needs detailed tracks
   * for its committed legs without paying the 2-pass routing cost
   * for every candidate it evaluates.
   *
   * <p>The single-pass tracks have correct geometry and cost but lack
   * the {@code wayKeyValues} fields required by the quality gate's
   * paved-profile hostility check. Re-tracking walks the existing path
   * via {@code guideTrack} so the resulting track follows exactly the
   * same nodes, just with full per-edge metadata.
   *
   * <p>{@code refTrack} is accepted for call-site compatibility but is not
   * applied during this pass. Reuse penalties belong to route choice; this
   * method only annotates an already-chosen route.
   */
  /**
   * Fallback time budget for the guided detail-retrack when the caller imposed
   * none ({@code maxRunningTime <= 0} — e.g. the quality tests' {@code doRun(0)}
   * or an untimed CLI run). The guided pass normally visits few nodes, but if it
   * exceeds the guide-track cost cap it can fall back to a free search; this caps
   * that so a pathological retrack cannot run unbounded (it then times out and
   * gracefully returns the raw track via the catch below). Production
   * ({@code maxRunningTime > 0}) is already bounded by the request budget and is
   * left unchanged.
   */
  private static final long RETRACK_DETAIL_FALLBACK_BUDGET_MS = 60_000;

  OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack refTrack) {
    if (rawTrack == null || rawTrack.nodes == null || rawTrack.nodes.size() < 2) return rawTrack;
    double savedAirDistFactor = airDistanceCostFactor;
    double savedLastFactor = lastAirDistanceCostFactor;
    OsmTrack savedGuide = guideTrack;
    long savedStartTime = startTime;
    long savedMaxRunningTime = maxRunningTime;
    boolean savedSuppressIslandGuard = suppressRoutingIslandGuard;
    airDistanceCostFactor = 0.;
    lastAirDistanceCostFactor = 0.;
    guideTrack = rawTrack;
    startTime = System.currentTimeMillis();
    // Bound the retrack when the caller set no time budget (see constant above);
    // production paths pass a positive maxRunningTime and are unaffected.
    if (maxRunningTime <= 0) {
      maxRunningTime = RETRACK_DETAIL_FALLBACK_BUDGET_MS;
    }
    // Guided retracking visits few nodes (the route is already known), so
    // the island-check guard `nodesVisited < MAXNODES_ISLAND_CHECK` would
    // false-positive every call. Suppress it only for this scoped retrack;
    // do not mutate islandNodePairs.freezeCount, because the rest of the
    // planner still needs normal island detection.
    suppressRoutingIslandGuard = true;
    try {
      // The guide track already fixes the exact node sequence. Reuse
      // poisoning is useful while choosing a route, but it can make this
      // metadata-only pass exceed the guide-track cost cap and fall back to
      // the raw no-message track. Keep retracking purely descriptive.
      OsmTrack detailed = findTrack("re-tracking", startWp, endWp, null, null, false);
      return detailed != null ? detailed : rawTrack;
    } catch (IllegalArgumentException | RoutingIslandException e) {
      logInfo("retrackForDetail failed: " + e.getClass().getSimpleName() + " "
        + (e.getMessage() == null ? "" : e.getMessage()) + " — using raw track");
      return rawTrack;
    } finally {
      guideTrack = savedGuide;
      airDistanceCostFactor = savedAirDistFactor;
      lastAirDistanceCostFactor = savedLastFactor;
      startTime = savedStartTime;
      maxRunningTime = savedMaxRunningTime;
      suppressRoutingIslandGuard = savedSuppressIslandGuard;
    }
  }

  void resetCache(boolean detailed) {
    if (hasInfo() && nodesCache != null) {
      logInfo("NodesCache status before reset=" + nodesCache.formatStatus());
    }
    long maxmem = routingContext.memoryclass * 1024L * 1024L; // in MB

    nodesCache = new NodesCache(segmentDir, routingContext.expctxWay, routingContext.forceSecondaryData, maxmem, nodesCache, detailed);
    islandNodePairs.clearTempPairs();
  }

  OsmPath getStartPath(OsmNode n1, OsmNode n2, MatchedWaypoint mwp, OsmNodeNamed endPos, boolean sameSegmentSearch) {
    if (endPos != null) {
      endPos.radius = 1.5;
    }
    OsmPath p = getStartPath(n1, n2, new OsmNodeNamed(mwp.crosspoint), endPos, sameSegmentSearch);

    // special case: start+end on same segment
    if (p != null && p.cost >= 0 && sameSegmentSearch && endPos != null && endPos.radius < 1.5) {
      p.treedepth = 0; // hack: mark for the final-check
    }
    return p;
  }


  OsmPath getStartPath(OsmNode n1, OsmNode n2, OsmNodeNamed wp, OsmNodeNamed endPos, boolean sameSegmentSearch) {
    try {
      routingContext.setWaypoint(wp, sameSegmentSearch ? endPos : null, false);
      OsmPath bestPath = null;
      OsmLink bestLink = null;
      OsmLink startLink = new OsmLink(null, n1);
      OsmPath startPath = routingContext.createPath(startLink);
      startLink.addLinkHolder(startPath, null);
      double minradius = 1e10;
      for (OsmLink link = n1.firstlink; link != null; link = link.getNext(n1)) {
        OsmNode nextNode = link.getTarget(n1);
        if (nextNode.isHollow())
          continue; // border node?
        if (nextNode.firstlink == null)
          continue; // don't care about dead ends
        if (nextNode == n1)
          continue; // ?
        if (nextNode != n2)
          continue; // just that link

        wp.radius = 1.5;
        OsmPath testPath = routingContext.createPath(startPath, link, null, guideTrack != null);
        testPath.airdistance = endPos == null ? 0 : nextNode.calcDistance(endPos);
        if (wp.radius < minradius) {
          bestPath = testPath;
          minradius = wp.radius;
          bestLink = link;
        }
      }
      if (bestLink != null) {
        bestLink.addLinkHolder(bestPath, n1);
      }
      if (bestPath != null) bestPath.treedepth = 1;

      return bestPath;
    } finally {
      routingContext.unsetWaypoint();
    }
  }

  OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc) {
    try {
      List<OsmNode> wpts2 = new ArrayList<>();
      if (startWp != null) wpts2.add(startWp.waypoint);
      if (endWp != null) wpts2.add(endWp.waypoint);
      routingContext.cleanNogoList(wpts2);

      boolean detailed = guideTrack != null;
      resetCache(detailed);
      nodesCache.nodesMap.cleanupMode = detailed ? 0 : (routingContext.considerTurnRestrictions ? 2 : 1);
      return _findTrack(operationName, startWp, endWp, costCuttingTrack, refTrack, fastPartialRecalc);
    } finally {
      routingContext.restoreNogoList();
      nodesCache.clean(false); // clean only non-virgin caches
    }
  }


  private OsmTrack _findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc) {
    boolean verbose = guideTrack != null;

    int maxTotalCost = guideTrack != null ? guideTrack.cost + 5000 : 1000000000;
    int firstMatchCost = 1000000000;

    logInfo("findtrack with airDistanceCostFactor=" + airDistanceCostFactor);
    if (costCuttingTrack != null) logInfo("costCuttingTrack.cost=" + costCuttingTrack.cost);

    matchPath = null;
    int nodesVisited = 0;

    long startNodeId1 = startWp.node1.getIdFromPos();
    long startNodeId2 = startWp.node2.getIdFromPos();
    long endNodeId1 = endWp == null ? -1L : endWp.node1.getIdFromPos();
    long endNodeId2 = endWp == null ? -1L : endWp.node2.getIdFromPos();
    OsmNode end1 = null;
    OsmNode end2 = null;
    OsmNodeNamed endPos = null;

    boolean sameSegmentSearch = false;
    OsmNode start1 = nodesCache.getGraphNode(startWp.node1);
    OsmNode start2 = nodesCache.getGraphNode(startWp.node2);
    if (endWp != null) {
      end1 = nodesCache.getGraphNode(endWp.node1);
      end2 = nodesCache.getGraphNode(endWp.node2);
      nodesCache.nodesMap.endNode1 = end1;
      nodesCache.nodesMap.endNode2 = end2;
      endPos = new OsmNodeNamed(endWp.crosspoint);
      sameSegmentSearch = (start1 == end1 && start2 == end2) || (start1 == end2 && start2 == end1);
    }
    if (!nodesCache.obtainNonHollowNode(start1)) {
      return null;
    }
    nodesCache.expandHollowLinkTargets(start1);
    if (!nodesCache.obtainNonHollowNode(start2)) {
      return null;
    }
    nodesCache.expandHollowLinkTargets(start2);


    routingContext.startDirectionValid = routingContext.forceUseStartDirection || fastPartialRecalc;
    routingContext.startDirectionValid &= routingContext.startDirection != null && !routingContext.inverseDirection;
    if (routingContext.startDirectionValid) {
      logInfo("using start direction " + routingContext.startDirection);
    }

    OsmPath startPath1 = getStartPath(start1, start2, startWp, endPos, sameSegmentSearch);
    OsmPath startPath2 = getStartPath(start2, start1, startWp, endPos, sameSegmentSearch);

    // check for an INITIAL match with the cost-cutting-track
    if (costCuttingTrack != null) {
      OsmPathElement pe1 = costCuttingTrack.getLink(startNodeId1, startNodeId2);
      if (pe1 != null) {
        logInfo("initialMatch pe1.cost=" + pe1.cost);
        int c = startPath1.cost - pe1.cost;
        if (c < 0) c = 0;
        if (c < firstMatchCost) firstMatchCost = c;
      }

      OsmPathElement pe2 = costCuttingTrack.getLink(startNodeId2, startNodeId1);
      if (pe2 != null) {
        logInfo("initialMatch pe2.cost=" + pe2.cost);
        int c = startPath2.cost - pe2.cost;
        if (c < 0) c = 0;
        if (c < firstMatchCost) firstMatchCost = c;
      }

      if (firstMatchCost < 1000000000)
        logInfo("firstMatchCost from initial match=" + firstMatchCost);
    }

    if (startPath1 == null) return null;
    if (startPath2 == null) return null;

    synchronized (openSet) {
      openSet.clear();
      addToOpenset(startPath1);
      addToOpenset(startPath2);
    }
    List<OsmPath> openBorderList = new ArrayList<>(4096);
    boolean memoryPanicMode = false;
    boolean needNonPanicProcessing = false;

    for (; ; ) {
      if (terminated) {
        throw new IllegalArgumentException("operation killed by thread-priority-watchdog after " + (System.currentTimeMillis() - startTime) / 1000 + " seconds");
      }

      if (maxRunningTime > 0) {
        long timeout = (matchPath == null && fastPartialRecalc) ? maxRunningTime / 3 : maxRunningTime;
        if (System.currentTimeMillis() - startTime > timeout) {
          throw new IllegalArgumentException(operationName + " timeout after " + (timeout / 1000) + " seconds");
        }
      }

      synchronized (openSet) {

        OsmPath path = openSet.popLowestKeyValue();
        if (path == null) {
          if (openBorderList.isEmpty()) {
            break;
          }
          for (OsmPath p : openBorderList) {
            openSet.add(p.cost + (int) (p.airdistance * airDistanceCostFactor), p);
          }
          openBorderList.clear();
          memoryPanicMode = false;
          needNonPanicProcessing = true;
          continue;
        }

        if (path.airdistance == -1) {
          continue;
        }

        if (directWeaving && nodesCache.hasHollowLinkTargets(path.getTargetNode())) {
          if (!memoryPanicMode) {
            if (!nodesCache.nodesMap.isInMemoryBounds(openSet.getSize(), false)) {
              int nodesBefore = nodesCache.nodesMap.nodesCreated;
              int pathsBefore = openSet.getSize();

              nodesCache.nodesMap.collectOutreachers();
              for (; ; ) {
                OsmPath p3 = openSet.popLowestKeyValue();
                if (p3 == null) break;
                if (p3.airdistance != -1 && nodesCache.nodesMap.canEscape(p3.getTargetNode())) {
                  openBorderList.add(p3);
                }
              }
              nodesCache.nodesMap.clearTemp();
              for (OsmPath p : openBorderList) {
                openSet.add(p.cost + (int) (p.airdistance * airDistanceCostFactor), p);
              }
              openBorderList.clear();
              logInfo("collected, nodes/paths before=" + nodesBefore + "/" + pathsBefore + " after=" + nodesCache.nodesMap.nodesCreated + "/" + openSet.getSize() + " maxTotalCost=" + maxTotalCost);
              if (!nodesCache.nodesMap.isInMemoryBounds(openSet.getSize(), true)) {
                if (maxTotalCost < 1000000000 || needNonPanicProcessing || fastPartialRecalc) {
                  throw new IllegalArgumentException("memory limit reached");
                }
                memoryPanicMode = true;
                logInfo("************************ memory limit reached, enabled memory panic mode *************************");
              }
            }
          }
          if (memoryPanicMode) {
            openBorderList.add(path);
            continue;
          }
        }
        needNonPanicProcessing = false;


        if (fastPartialRecalc && matchPath != null && path.cost > 30L * firstMatchCost && !costCuttingTrack.isDirty) {
          logInfo("early exit: firstMatchCost=" + firstMatchCost + " path.cost=" + path.cost);

          // use an early exit, unless there's a realistc chance to complete within the timeout
          if (path.cost > maxTotalCost / 2 && System.currentTimeMillis() - startTime < maxRunningTime / 3) {
            logInfo("early exit supressed, running for completion, resetting timeout");
            startTime = System.currentTimeMillis();
            fastPartialRecalc = false;
          } else {
            throw new IllegalArgumentException("early exit for a close recalc");
          }
        }

        if (nodeLimit > 0) { // check node-limit for target island search
          if (--nodeLimit == 0) {
            return null;
          }
        }

        nodesVisited++;
        linksProcessed++;

        OsmLink currentLink = path.getLink();
        OsmNode sourceNode = path.getSourceNode();
        OsmNode currentNode = path.getTargetNode();

        if (currentLink.isLinkUnused()) {
          continue;
        }

        long currentNodeId = currentNode.getIdFromPos();
        long sourceNodeId = sourceNode.getIdFromPos();

        if (!path.didEnterDestinationArea()) {
          islandNodePairs.addTempPair(sourceNodeId, currentNodeId);
        }

        if (path.treedepth != 1) {
          if (path.treedepth == 0) { // hack: sameSegment Paths marked treedepth=0 to pass above check
            path.treedepth = 1;
          }

          if ((sourceNodeId == endNodeId1 && currentNodeId == endNodeId2)
            || (sourceNodeId == endNodeId2 && currentNodeId == endNodeId1)) {
            // track found, compile
            logInfo("found track at cost " + path.cost + " nodesVisited = " + nodesVisited);
            OsmTrack t = compileTrack(path, verbose);
            t.showspeed = routingContext.showspeed;
            t.showSpeedProfile = routingContext.showSpeedProfile;
            return t;
          }

          // check for a match with the cost-cutting-track
          if (costCuttingTrack != null) {
            OsmPathElement pe = costCuttingTrack.getLink(sourceNodeId, currentNodeId);
            if (pe != null) {
              // remember first match cost for fast termination of partial recalcs
              int parentcost = path.originElement == null ? 0 : path.originElement.cost;

              // hitting start-element of costCuttingTrack?
              int c = path.cost - parentcost - pe.cost;
              if (c > 0) parentcost += c;

              if (parentcost < firstMatchCost) firstMatchCost = parentcost;

              int costEstimate = path.cost
                + path.elevationCorrection()
                + (costCuttingTrack.cost - pe.cost);
              if (costEstimate <= maxTotalCost) {
                matchPath = OsmPathElement.create(path);
              }
              if (costEstimate < maxTotalCost) {
                logInfo("maxcost " + maxTotalCost + " -> " + costEstimate);
                maxTotalCost = costEstimate;
              }
            }
          }
        }

        OsmLinkHolder firstLinkHolder = currentLink.getFirstLinkHolder(sourceNode);
        for (OsmLinkHolder linkHolder = firstLinkHolder; linkHolder != null; linkHolder = linkHolder.getNextForLink()) {
          ((OsmPath) linkHolder).airdistance = -1; // invalidate the entry in the open set;
        }

        if (path.treedepth > 1) {
          boolean isBidir = currentLink.isBidirectional();
          sourceNode.unlinkLink(currentLink);

          // if the counterlink is alive and does not yet have a path, remove it
          if (isBidir && currentLink.getFirstLinkHolder(currentNode) == null && !routingContext.considerTurnRestrictions) {
            currentNode.unlinkLink(currentLink);
          }
        }

        // recheck cutoff before doing expensive stuff
        int addDiff = 100;
        if (path.cost + path.airdistance > maxTotalCost + addDiff) {
          continue;
        }

        nodesCache.nodesMap.currentMaxCost = maxTotalCost;
        nodesCache.nodesMap.currentPathCost = path.cost;
        nodesCache.nodesMap.destination = endPos;

        routingContext.firstPrePath = null;

        for (OsmLink link = currentNode.firstlink; link != null; link = link.getNext(currentNode)) {
          OsmNode nextNode = link.getTarget(currentNode);

          if (!nodesCache.obtainNonHollowNode(nextNode)) {
            continue; // border node?
          }
          if (nextNode.firstlink == null) {
            continue; // don't care about dead ends
          }
          if (nextNode == sourceNode) {
            continue; // border node?
          }

          OsmPrePath prePath = routingContext.createPrePath(path, link);
          if (prePath != null) {
            prePath.next = routingContext.firstPrePath;
            routingContext.firstPrePath = prePath;
          }
        }

        for (OsmLink link = currentNode.firstlink; link != null; link = link.getNext(currentNode)) {
          OsmNode nextNode = link.getTarget(currentNode);

          if (!nodesCache.obtainNonHollowNode(nextNode)) {
            continue; // border node?
          }
          if (nextNode.firstlink == null) {
            continue; // don't care about dead ends
          }
          if (nextNode == sourceNode) {
            continue; // border node?
          }

          if (guideTrack != null) {
            int gidx = path.treedepth + 1;
            if (gidx >= guideTrack.nodes.size()) {
              continue;
            }
            OsmPathElement guideNode = guideTrack.nodes.get(routingContext.inverseRouting ? guideTrack.nodes.size() - 1 - gidx : gidx);
            long nextId = nextNode.getIdFromPos();
            if (nextId != guideNode.getIdFromPos()) {
              // not along the guide-track, discard, but register for voice-hint processing
              if (routingContext.turnInstructionMode > 0) {
                OsmPath detour = routingContext.createPath(path, link, refTrack, true);
                if (detour.cost >= 0. && nextId != startNodeId1 && nextId != startNodeId2) {
                  guideTrack.registerDetourForId(currentNode.getIdFromPos(), OsmPathElement.create(detour));
                }
              }
              continue;
            }
          }

          OsmPath bestPath = null;

          boolean isFinalLink = false;
          long targetNodeId = nextNode.getIdFromPos();
          if (currentNodeId == endNodeId1 || currentNodeId == endNodeId2) {
            if (targetNodeId == endNodeId1 || targetNodeId == endNodeId2) {
              isFinalLink = true;
            }
          }

          for (OsmLinkHolder linkHolder = firstLinkHolder; linkHolder != null; linkHolder = linkHolder.getNextForLink()) {
            OsmPath otherPath = (OsmPath) linkHolder;
            try {
              if (isFinalLink) {
                endPos.radius = 1.5; // 1.5 meters is the upper limit that will not change the unit-test result..
                routingContext.setWaypoint(endPos, true);
              }
              OsmPath testPath = routingContext.createPath(otherPath, link, refTrack, guideTrack != null);
              if (testPath.cost >= 0 && (bestPath == null || testPath.cost < bestPath.cost) &&
                (testPath.sourceNode.getIdFromPos() != testPath.targetNode.getIdFromPos())) {
                bestPath = testPath;
              }
            } finally {
              if (isFinalLink) {
                routingContext.unsetWaypoint();
              }
            }
          }
          if (bestPath != null) {
            bestPath.airdistance = isFinalLink ? 0 : nextNode.calcDistance(endPos);

            boolean inRadius = boundary == null || boundary.isInBoundary(nextNode, bestPath.cost);

            if (inRadius && (isFinalLink || bestPath.cost + bestPath.airdistance <= (lastAirDistanceCostFactor != 0. ? maxTotalCost * lastAirDistanceCostFactor : maxTotalCost) + addDiff)) {
              // add only if this may beat an existing path for that link
              OsmLinkHolder dominator = link.getFirstLinkHolder(currentNode);
              while (dominator != null) {
                OsmPath dp = (OsmPath) dominator;
                if (dp.airdistance != -1 && bestPath.definitlyWorseThan(dp)) {
                  break;
                }
                dominator = dominator.getNextForLink();
              }

              if (dominator == null) {
                bestPath.treedepth = path.treedepth + 1;
                link.addLinkHolder(bestPath, currentNode);
                addToOpenset(bestPath);
              }
            }
          }
        }
      }
    }

    if (!suppressRoutingIslandGuard
        && nodesVisited < MAXNODES_ISLAND_CHECK && islandNodePairs.getFreezeCount() < 5) {
      throw new RoutingIslandException();
    }

    return null;
  }

  private void addToOpenset(OsmPath path) {
    if (path.cost >= 0) {
      openSet.add(path.cost + (int) (path.airdistance * airDistanceCostFactor), path);
    }
  }

  private OsmTrack compileTrack(OsmPath path, boolean verbose) {
    OsmPathElement element = OsmPathElement.create(path);

    // for final track, cut endnode
    if (guideTrack != null && element.origin != null) {
      element = element.origin;
    }

    float totalTime = element.getTime();
    float totalEnergy = element.getEnergy();

    OsmTrack track = new OsmTrack();
    track.cost = path.cost;
    track.energy = (int) path.getTotalEnergy();

    int distance = 0;

    double eleFactor = routingContext.inverseRouting ? -0.25 : 0.25;
    while (element != null) {
      if (guideTrack != null && element.message == null) {
        element.message = new MessageData();
      }
      OsmPathElement nextElement = element.origin;
      // ignore double element
      if (nextElement != null && nextElement.positionEquals(element)) {
        element = nextElement;
        continue;
      }
      if (routingContext.inverseRouting) {
        element.setTime(totalTime - element.getTime());
        element.setEnergy(totalEnergy - element.getEnergy());
        track.nodes.add(element);
      } else {
        track.nodes.add(0, element);
      }

      if (nextElement != null) {
        distance += element.calcDistance(nextElement);
      }
      element = nextElement;
    }
    track.distance = distance;
    logInfo("track-length = " + track.distance);
    track.buildMap();

    // for final track..
    if (guideTrack != null) {
      track.copyDetours(guideTrack);
    }
    return track;
  }

  private OsmTrack mergeTrack(OsmPathElement match, OsmTrack oldTrack) {
    logInfo("**************** merging match=" + match.cost + " with oldTrack=" + oldTrack.cost);
    OsmPathElement element = match;
    OsmTrack track = new OsmTrack();
    track.cost = oldTrack.cost;

    while (element != null) {
      track.addNode(element);
      element = element.origin;
    }
    long lastId = 0;
    long id1 = match.getIdFromPos();
    long id0 = match.origin == null ? 0 : match.origin.getIdFromPos();
    boolean appending = false;
    for (OsmPathElement n : oldTrack.nodes) {
      if (appending) {
        track.nodes.add(n);
      }

      long id = n.getIdFromPos();
      if (id == id1 && lastId == id0) {
        appending = true;
      }
      lastId = id;
    }


    track.buildMap();
    return track;
  }

  public int getPathPeak() {
    synchronized (openSet) {
      return openSet.getPeakSize();
    }
  }

  public int[] getOpenSet() {
    if (extract == null) {
      extract = new Object[500];
    }

    synchronized (openSet) {
      if (guideTrack != null) {
        List<OsmPathElement> nodes = guideTrack.nodes;
        int[] res = new int[nodes.size() * 2];
        int i = 0;
        for (OsmPathElement n : nodes) {
          res[i++] = n.getILon();
          res[i++] = n.getILat();
        }
        return res;
      }

      int size = openSet.getExtract(extract);
      int[] res = new int[size * 2];
      for (int i = 0, j = 0; i < size; i++) {
        OsmPath p = (OsmPath) extract[i];
        extract[i] = null;
        OsmNode n = p.getTargetNode();
        res[j++] = n.ilon;
        res[j++] = n.ilat;
      }
      return res;
    }
  }

  public boolean isFinished() {
    return finished;
  }

  public synchronized int getLinksProcessed() {
    return linksProcessed;
  }

  /**
   * Aggregates a child engine's work count into this (parent) engine.
   * Synchronized: in speculative AUTO mode the GREEDY child runs
   * runChildCandidate on its own thread concurrently with the request
   * thread's ISO_GREEDY aggregation — an unsynchronized += on the shared
   * parent field loses one child's entire count. The engine's own hot-loop
   * {@code linksProcessed++} stays unsynchronized (single-threaded per
   * engine); only the cross-engine aggregation contends.
   */
  private synchronized void addLinksProcessed(int childLinks) {
    linksProcessed += childLinks;
  }

  public int getDistance() {
    return foundTrack.distance;
  }

  public int getAscend() {
    return foundTrack.ascend;
  }

  public int getPlainAscend() {
    return foundTrack.plainAscend;
  }

  public String getTime() {
    return Formatter.getFormattedTime2(foundTrack.getTotalSeconds());
  }

  public OsmTrack getFoundTrack() {
    return foundTrack;
  }

  /** The last round-trip planning result (carries the planned loop waypoints), or null. */
  public RoundTripResult getLastRoundTripResult() {
    return lastRoundTripResult;
  }

  /**
   * The last round-trip track that was rejected by the quality gate, if
   * any. {@link #getFoundTrack()} returns null on rejection; this method
   * returns the geometry that tripped the gate so post-mortem analysis
   * tools can inspect WHY each rejection occurred. Returns null if no
   * round-trip request was made or no track ever reached the gate.
   */
  public OsmTrack getLastRejectedTrack() {
    return lastRejectedTrack;
  }

  public String getFoundInfo() {
    return outputMessage;
  }

  public int getAlternativeIndex() {
    return alternativeIndex;
  }

  public OsmTrack getFoundRawTrack() {
    return foundRawTrack;
  }

  public String getErrorMessage() {
    return errorMessage;
  }

  public void terminate() {
    terminated = true;
  }

  public boolean isTerminated() {
    return terminated;
  }

  public String getOutfile() {
    return outfile;
  }
}
