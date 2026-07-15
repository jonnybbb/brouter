package btools.router.roundtrip;

import java.io.File;
import java.util.List;

import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoutingContext;

/**
 * The read-only slice of the engine seam: the request context, engine
 * environment, and stateless engine primitives. Read-only is meant against
 * the round-trip request state — {@link #recalcTrack} mutates the track it is
 * given and {@link #addLinksProcessed} bumps a telemetry counter, but nothing
 * here mutates the request fields behind {@link RoundTripRequestState}. One of
 * the four role interfaces composed by {@link RoundTripEngineOps}.
 */
public interface EngineContext {

  /** The request context (read-only use; the planner reads profile knobs). */
  RoutingContext routingContext();

  /** The engine's segment directory (child engine construction). */
  File segmentDir();

  /** True when the engine runs in round-trip mode. */
  boolean isRoundTripMode();

  /** True while an explicit-via round trip is being generated. */
  boolean explicitViaRoundTrip();

  /** The active round-trip search radius (0 outside round-trip requests). */
  double roundTripSearchRadius();

  /** True when the active profile allows ferries (gate context). */
  boolean roundTripFerriesAllowed();

  /** True when the uniform gate would hard-reject this verdict. */
  boolean roundTripQualityHardReject(RoundTripQualityResult quality);

  /** Canonical missing-start-tile error, or null when the tile is present. */
  String startTileMissingError(OsmNodeNamed start);

  /** Area-info based random direction pick (upstream engine primitive). */
  double getRandomDirectionFromData(OsmNodeNamed wp, double searchRadius);

  /** v1.7.8 geometric circle placement (upstream engine primitive). */
  void buildPointsFromCircle(List<OsmNodeNamed> waypoints, double startAngle,
                             double searchRadius, int points);

  /** Recalculate a track's totals (engine primitive shared with normal routing). */
  void recalcTrack(OsmTrack track);

  /** The optimized-FAST placement seam of this engine. */
  FastPlacementOps fastPlacementOps();

  /** Aggregate a child engine's link expansions into this engine's counter. */
  void addLinksProcessed(long links);

  /** Milliseconds left of the whole-request budget (Long.MAX_VALUE when untimed). */
  long remainingRequestBudgetMs();
}
