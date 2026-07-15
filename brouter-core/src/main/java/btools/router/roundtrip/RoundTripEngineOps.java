package btools.router.roundtrip;

/**
 * The round-trip planners' seam to the routing engine, composed of four role
 * interfaces so each consumer can depend on just the slice it uses:
 * {@link LegRouter} (leg searches, matching, isochrone expansion),
 * {@link RoundTripRequestState} (shared mutable request state),
 * {@link EngineIO} (logging and track output), and
 * {@link EngineContext} (read-only request context and engine primitives).
 * The production adapter is {@code RoutingEngine#roundTripOps()} — the
 * engine's single public hand-out for this package (same pattern as
 * {@link FastPlacementOps}). Method names mirror the engine members they
 * delegate to.
 *
 * <p>Only the {@link RoundTripOrchestrator} holds the full composite; the
 * two methods declared here directly are orchestrator-only.
 */
public interface RoundTripEngineOps
  extends LegRouter, RoundTripRequestState, EngineIO, EngineContext {

  /** Run the engine's routing pipeline over the current waypoint list. */
  void doRouting(long budgetMs);

  /** Release the engine's routing caches. */
  void cleanupRoutingResources();
}
