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

  public DefaultLegEvaluator(RoundTripEngineOps ops) {
    this(ops, "refine-leg");
  }

  public DefaultLegEvaluator(RoundTripEngineOps ops, String operationPrefix) {
    this.ops = ops;
    this.operationPrefix = operationPrefix;
  }

  @Override
  public OsmTrack route(MatchedWaypoint from, MatchedWaypoint to, long timeoutMs) {
    try {
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
