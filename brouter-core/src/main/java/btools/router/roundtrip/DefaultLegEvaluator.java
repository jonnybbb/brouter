package btools.router.roundtrip;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;
import btools.router.RoutingIslandException;

/**
 * Standard {@link LegEvaluator} implementation routing raw legs via {@link RoundTripEngineOps#findTrackTimed}.
 */
public final class DefaultLegEvaluator implements LegEvaluator {

  private final RoundTripEngineOps ops;
  private final String operationPrefix;
  private final MatchedWaypoint openingWaypoint;

  public DefaultLegEvaluator(RoundTripEngineOps ops) {
    this(ops, "refine-leg");
  }

  public DefaultLegEvaluator(RoundTripEngineOps ops, String operationPrefix) {
    this.ops = ops;
    this.operationPrefix = operationPrefix;
    this.openingWaypoint = ops.matchedWaypoints() == null || ops.matchedWaypoints().isEmpty()
      ? null : RefineSkeleton.copyWaypoint(ops.matchedWaypoints().get(0));
  }

  @Override
  public OsmTrack route(MatchedWaypoint from, MatchedWaypoint to, long timeoutMs) {
    boolean opening = openingWaypoint != null && LegCache.isSeamValid(openingWaypoint, from);
    try (RefineHeading heading = new RefineHeading(ops.routingContext(), opening)) {
      ops.checkRefinementBudget();
      return ops.findTrackTimed(operationPrefix, from, to, null, timeoutMs);
    } catch (IllegalArgumentException | RoutingIslandException e) {
      if (ops.isTerminated() || (e.getMessage() != null
          && (e.getMessage().contains("timeout") || e.getMessage().contains("thread-priority-watchdog")))) {
        throw e;
      }
      return null;
    }
  }
}
