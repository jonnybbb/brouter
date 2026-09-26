package btools.router.roundtrip;

import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;

/**
 * Engine seam role: leg searches, waypoint matching, isochrone expansion,
 * and the kill switch.
 */
public interface LegRouter {

  default long refinementDeadline() {
    return 0;
  }

  default void setRefinementDeadline(long deadline) {
  }

  default void checkRefinementBudget() {
    RefineBudget.check(this, refinementDeadline());
  }

  /** One leg search — the engine's findTrack primitive. */
  OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                     OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc);

  /**
   * Leg search with the engine's live guide track suspended — a local repair
   * search must not be steered by the track it is repairing.
   */
  OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp);

  /**
   * Leg search under its own time budget: the engine saves/restores its clock
   * and runs goal-directed at the profile's pass-1 coefficient.
   */
  OsmTrack findTrackTimed(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                          OsmTrack refTrack, long budgetMs);

  /** Re-run a raw track at full detail between its endpoints. */
  OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                            OsmTrack refTrack);

  /** Re-run a raw track at full detail between its endpoints bounded by budgetMs. */
  default OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                                    OsmTrack refTrack, long budgetMs) {
    return retrackForDetail(rawTrack, startWp, endWp, refTrack);
  }

  /** Profile-aware road snap for a single point. */
  MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist);

  /** Batch waypoint matching through the engine's node cache. */
  void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance);

  /**
   * Island-learning hook for an islanded leg (fast-motor profiles): record
   * both endpoints' match pairs so the engine's matcher avoids the pocket on
   * the next match of the same point. Returns whether anything was learned —
   * {@code false} on bike/foot profiles (bit-identical behavior) and for
   * endpoints without match nodes. Default no-op for test stubs.
   */
  default boolean recordLegIsland(MatchedWaypoint from, MatchedWaypoint to) {
    return false;
  }

  void resetCache(boolean detailed);

  /** Wall-clock bound for the next isochrone expansion; 0 clears it. */
  void setTransientExpansionDeadline(long deadlineMillis);

  /** Start-centered isochrone expansion (placement variant). */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius);

  /** Start-centered isochrone expansion. */
  IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                 OsmTrack refTrack, boolean includeCandidateTracks);

  /** Terminate the engine's current search (volatile kill flag). */
  void terminate();

  boolean isTerminated();

  /**
   * Register a hook run when THIS engine is terminated — the cascade that
   * forwards a server pre-emption to child engines (typically
   * {@code child::terminate}). Registering after termination runs the hook
   * immediately.
   */
  void addTerminationHook(Runnable hook);

  /** Price explicitly raw legs after clipping/detail preparation, with one continuous state. */
  default LoopPrice priceRawLoop(List<OsmTrack> legs, List<MatchedWaypoint> waypoints) {
    return LoopPrice.failure(FinalizationOutcome.FAILURE);
  }

  /** Linear path walker: validates the complete finished geometry, regardless of metadata. */
  default int walkPathCost(OsmTrack track, MatchedWaypoint startWp, MatchedWaypoint endWp) {
    return -1;
  }

  /** Continuous loop walker: prices a closed loop across vias as a single path. */
  default int walkLoopCost(List<OsmTrack> legs, List<MatchedWaypoint> waypoints) {
    return -1;
  }

  /** Method used for the last path/loop cost walk ("continuous", "per_leg", "single_track", or "none"). */
  default String getLastPricingMethod() {
    return "none";
  }
  default String getLastPricingFailure() {
    return "unavailable";
  }

}
