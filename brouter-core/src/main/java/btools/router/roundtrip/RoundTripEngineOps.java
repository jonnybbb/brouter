package btools.router.roundtrip;

import java.io.File;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoutingContext;

/**
 * The round-trip planners' seam to the routing engine: coarse delegates to the
 * engine's leg router, matcher, isochrone expansion, timers, and logging, so
 * {@link GreedyRoundTripPlanner} and {@link GraphNativeCandidateProvider} never
 * touch engine internals directly and the engine's members stay
 * package-private. The production adapter is {@code RoutingEngine#roundTripOps()}
 * — the engine's single public hand-out for this package (same pattern as
 * {@link FastPlacementOps}). Method names mirror the engine members they
 * delegate to.
 */
public interface RoundTripEngineOps {

  /** The request context (read-only use; the planner reads profile knobs). */
  RoutingContext routingContext();

  /** Engine log line; a no-op in {@code quite} child engines (AUTO candidates). */
  void logInfo(String msg);

  /** True when the engine was terminated (watchdog/cancel). */
  boolean isTerminated();

  /** Engine start wall-clock millis (deadline arithmetic). */
  long startTime();

  /** Engine run budget in millis, 0 = untimed. */
  long maxRunningTime();

  /** The active round-trip search radius (0 outside round-trip requests). */
  double roundTripSearchRadius();

  /** True when the engine runs in round-trip mode. */
  boolean isRoundTripMode();

  /** True while an explicit-via round trip is being generated. */
  boolean explicitViaRoundTrip();

  /** Recalculate a track's totals (engine primitive shared with normal routing). */
  void recalcTrack(OsmTrack track);

  /** v1.7.8 geometric circle placement (upstream engine primitive). */
  void buildPointsFromCircle(List<OsmNodeNamed> waypoints, double startAngle,
                             double searchRadius, int points);

  /** Merge duplicate round-trip voice hints (needs VoiceHint package internals). */
  void consolidateRoundTripVoiceHints(OsmTrack track);

  /** Publish the matched waypoints of the final track on the engine. */
  void setMatchedWaypoints(List<MatchedWaypoint> waypoints);

  /** The engine's published matched waypoints. */
  List<MatchedWaypoint> matchedWaypoints();

  /** The live request waypoint list (the orchestrator appends generated vias). */
  List<OsmNodeNamed> waypoints();

  /** The engine's current result track (null while unset / after a reset). */
  OsmTrack foundTrack();

  /** Set the engine's result track. */
  void setFoundTrack(OsmTrack track);

  /** The engine's current error message (track-XOR-error contract). */
  String errorMessage();

  /** Set the engine's error message. */
  void setErrorMessage(String message);

  /** Run the engine's routing pipeline over the current waypoint list. */
  void doRouting(long budgetMs);

  /** The active effort policy preset (null outside bounded dispatch). */
  RoundTripEffortPolicy roundTripEffortPolicy();

  /** Set the active effort policy preset. */
  void setRoundTripEffortPolicy(RoundTripEffortPolicy policy);

  /** Absolute wall-clock deadline for the whole round-trip request; 0 = untimed. */
  long roundTripRequestDeadline();

  /** Set the request deadline. */
  void setRoundTripRequestDeadline(long deadlineMillis);

  /** Per-leg routing budget for the round-trip fallthrough. */
  long roundTripRoutingBudgetMs();

  /** Set the per-leg routing budget. */
  void setRoundTripRoutingBudgetMs(long budgetMs);

  /** Set the active search radius (consulted by engine-side matching hooks). */
  void setRoundTripSearchRadius(double searchRadius);

  /** Publish the planner result for telemetry consumers. */
  void setLastRoundTripResult(RoundTripResult result);

  /** Start-centered isochrone expansion, placement variant (2-arg overload). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius);

  /** Canonical missing-start-tile error, or null when the tile is present. */
  String startTileMissingError(OsmNodeNamed start);

  /** Write the adopted track to the engine's configured output (engine IO). */
  void writeAdoptedTrackOutput(OsmTrack track);

  /** Terminate the engine's current search (volatile kill flag). */
  void terminate();

  /** Release the engine's routing caches. */
  void cleanupRoutingResources();

  /** Engine exception logging. */
  void logException(Throwable t);

  /** The optimized-FAST placement seam of this engine. */
  FastPlacementOps fastPlacementOps();

  /** Area-info based random direction pick (upstream engine primitive). */
  double getRandomDirectionFromData(OsmNodeNamed wp, double searchRadius);

  /** The bounded tier's gate verdict, published for the shared doRun advisory. */
  RoundTripQualityResult boundedGateVerdict();

  /** Set the bounded tier's gate verdict. */
  void setBoundedGateVerdict(RoundTripQualityResult verdict);

  /** Per-leg guide tracks from a greedy adoption, consulted by doRouting. */
  OsmTrack[] greedyLegTracks();

  /** Set the per-leg guide tracks. */
  void setGreedyLegTracks(OsmTrack[] tracks);

  /** The last gate-rejected track (diagnostics surface). */
  OsmTrack lastRejectedTrack();

  /** Set the last gate-rejected track. */
  void setLastRejectedTrack(OsmTrack track);

  /** True when the active profile allows ferries (gate context). */
  boolean roundTripFerriesAllowed();

  /** True when the uniform gate would hard-reject this verdict. */
  boolean roundTripQualityHardReject(RoundTripQualityResult quality);

  /** Milliseconds left of the whole-request budget (Long.MAX_VALUE when untimed). */
  long remainingRequestBudgetMs();

  /** The engine's segment directory (child engine construction). */
  File segmentDir();

  /** Engine throwable logging (compact form). */
  void logThrowable(Throwable t);

  /** Mark/unmark the explicit-via round-trip mode. */
  void setExplicitViaRoundTrip(boolean explicitVia);

  /** The forced-corridor acceptance latch from the greedy adoption. */
  boolean roundTripForcedCorridorAccepted();

  /** Set the forced-corridor acceptance latch. */
  void setRoundTripForcedCorridorAccepted(boolean accepted);

  /** Aggregate a child engine's link expansions into this engine's counter. */
  void addLinksProcessed(long links);

  /** Set the engine start wall-clock (planner saves and restores around a leg). */
  void setStartTime(long startTimeMillis);

  /** Set the engine run budget (planner saves and restores around a leg). */
  void setMaxRunningTime(long maxRunningTimeMillis);

  /** Wall-clock bound for the next isochrone expansion; 0 clears it. */
  void setTransientExpansionDeadline(long deadlineMillis);

  /** Pass-1 air-distance cost factor currently active on the engine. */
  double airDistanceCostFactor();

  /** Set the pass-1 air-distance cost factor (planner saves and restores it). */
  void setAirDistanceCostFactor(double factor);

  /** One leg search — the engine's internal findTrack primitive. */
  OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                     OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc);

  /** Re-run a raw track at full detail between its endpoints. */
  OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                            OsmTrack refTrack);

  /** Profile-aware road snap for a single point. */
  MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist);

  /** Reset the engine's node cache. */
  void resetCache(boolean detailed);

  /**
   * One leg search with the engine's live guide track suspended — a local
   * repair search must not be steered by the track it is repairing.
   */
  OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp);

  /** Batch waypoint matching through the engine's node cache + island pairs. */
  void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance);

  /** Start-centered isochrone expansion (see {@code RoutingEngine#runIsochroneExpansion}). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                 OsmTrack refTrack, boolean includeCandidateTracks);
}
