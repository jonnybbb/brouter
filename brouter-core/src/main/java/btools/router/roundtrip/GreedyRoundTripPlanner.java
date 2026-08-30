package btools.router.roundtrip;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;

import java.util.ArrayList;
import java.util.List;

/**
 * The forward greedy loop planner (CEUR-WS Vol-3885): legs are committed from
 * the start outward, the tail head is the via planned last, and the closing
 * leg is routed from it back to the start. All planning machinery lives in
 * {@link AbstractGreedyPlanner}; this class only fixes the orientation. Its
 * output is pinned bit-for-bit by {@code GreedyPlannerParityTest}.
 */
public class GreedyRoundTripPlanner extends AbstractGreedyPlanner {

  public GreedyRoundTripPlanner(RoundTripEngineOps engine, RoundTripCandidateProvider provider) {
    super(engine, provider);
  }

  /** Convenience wiring form: fan the composite engine seam out to the roles. */
  public GreedyRoundTripPlanner(RoundTripEngineOps engine, RoundTripCandidateProvider provider,
                                CandidateScorer scorer, int subRouteCount, double tolerance,
                                int maxAttempts) {
    super(engine, provider, scorer, subRouteCount, tolerance, maxAttempts);
  }

  public GreedyRoundTripPlanner(LegRouter router, EngineIO io,
                                EngineContext ctx, RoundTripCandidateProvider provider,
                                CandidateScorer scorer, int subRouteCount, double tolerance,
                                int maxAttempts) {
    super(router, io, ctx, provider, scorer, subRouteCount, tolerance, maxAttempts);
  }

  @Override
  protected boolean initSession(GreedyPlanSession s, MatchedWaypoint startMwp) {
    s.currentMwp = startMwp;
    s.waypointStack.add(startMwp);
    return true;
  }

  @Override
  protected OsmTrack routeLeg(String name, MatchedWaypoint tailHead, MatchedWaypoint candidate,
                              OsmTrack refTrack, long deadline) {
    return timedFindTrack(name, tailHead, candidate, refTrack, deadline);
  }

  @Override
  protected OsmTrack detailLeg(String name, OsmTrack leg, MatchedWaypoint tailHead,
                               MatchedWaypoint candidate, OsmTrack refTrack, long deadline) {
    return detailWithFallback(name, leg, tailHead, candidate, refTrack, deadline);
  }

  @Override
  protected OsmTrack routeClosure(GreedyPlanSession s, List<OsmTrack> segments, OsmTrack ref,
                                  MatchedWaypoint tailHead, long deadline, int step) {
    return routeReturnWithVariants(segments, ref, tailHead, s.startMwp, deadline, s.result,
      s.totalDistance, s.desiredDistance, step);
  }

  @Override
  protected OsmTrack detailClosure(String name, OsmTrack closure, GreedyPlanSession s,
                                   MatchedWaypoint tailHead, OsmTrack ref, long deadline) {
    return detailWithFallback(name, closure, tailHead, s.startMwp, ref, deadline);
  }

  @Override
  protected OsmTrack routeForceClose(GreedyPlanSession s, MatchedWaypoint tailHead,
                                     OsmTrack ref, long deadline) {
    return timedFindTrack("greedy-force-close", tailHead, s.startMwp, ref, deadline);
  }

  @Override
  protected OsmTrack retrackClosure(GreedyPlanSession s, OsmTrack closure, MatchedWaypoint tailHead) {
    return router.retrackForDetail(closure, tailHead, s.startMwp, null);
  }

  @Override
  protected void commitLeg(List<OsmTrack> segments, OsmTrack leg) {
    segments.add(leg);
  }

  @Override
  protected void uncommitLeg(List<OsmTrack> segments) {
    segments.remove(segments.size() - 1);
  }

  @Override
  protected void replaceCommittedLeg(List<OsmTrack> segments, OsmTrack leg) {
    segments.set(segments.size() - 1, leg);
  }

  @Override
  protected void commitClosure(List<OsmTrack> segments, OsmTrack closure) {
    segments.add(closure);
  }

  @Override
  protected List<OsmTrack> loopLegs(List<OsmTrack> segments, OsmTrack closure) {
    List<OsmTrack> legs = new ArrayList<>(segments);
    if (closure != null) legs.add(closure);
    return legs;
  }

  @Override
  protected void joinTentative(List<OsmPathElement> target, List<OsmPathElement> prefix,
                               List<OsmPathElement> candidate) {
    target.addAll(prefix);
    appendNodesDeduped(target, candidate);
  }

  @Override
  protected List<MatchedWaypoint> travelOrderedStack(List<MatchedWaypoint> stack) {
    return stack;
  }

  @Override
  protected double directionBearing(double bearingFromTailHead, int cpIlon, int cpIlat,
                                    GreedyPlanSession s) {
    return bearingFromTailHead;
  }

  @Override
  protected int directionStep(int step) {
    return step;
  }

  @Override
  protected OsmPathElement legFarEnd(OsmTrack leg) {
    return leg.nodes.get(leg.nodes.size() - 1);
  }
}
