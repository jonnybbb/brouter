package btools.router.roundtrip;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.FormatGpx;
import btools.router.OsmNodeNamed;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

public class RefineFinalizerTest {

  @Before
  @After
  public void resetHooks() {
    RefineFinalizer.failureInjectionStep = RefineFinalizer.FinalizationStep.NONE;
    RefineFinalizer.failureInjectionTimeout = false;
  }

  private static RoundTripRequest createRequest(RoundTripEngineOps ops, RoutingEngine re, RoundTripResult res) {
    RoundTripRequest req = new RoundTripRequest(ops);
    req.track = re.getFoundTrack();
    req.lastResult = res;
    return req;
  }

  @Test
  public void testOwnershipIsolation() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> baselineRawLegs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      OsmTrack raw = ops.findTrackTimed("own-leg-" + i, mwps.get(i), mwps.get(i + 1), null, 5000L);
      Assert.assertNotNull(raw);
      baselineRawLegs.add(raw);
    }

    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    // Snapshot baseline track
    OsmTrack baseTrack = res.getTrack();
    int baseCost = baseTrack.cost;
    float baseDist = baseTrack.distance;
    int baseNodes = baseTrack.nodes.size();

    // Prepare Candidate A and Candidate B as separate deep copies
    List<OsmTrack> candALegs = new ArrayList<>();
    for (OsmTrack leg : baselineRawLegs) {
      candALegs.add(copyTrack(leg));
    }
    List<OsmTrack> candBLegs = new ArrayList<>();
    for (OsmTrack leg : baselineRawLegs) {
      candBLegs.add(copyTrack(leg));
    }

    // Finalize Candidate A
    FinishedCandidate finA = RefineFinalizer.finalizeCandidate(
      candALegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);
    Assert.assertEquals("Expected success: " + finA.getReason(), FinalizationOutcome.SUCCESS, finA.getOutcome());

    // Verify baseline was NOT mutated by finalizing Candidate A
    Assert.assertEquals("Baseline cost unchanged after finalizing A", baseCost, baseTrack.cost);
    Assert.assertEquals("Baseline distance unchanged after finalizing A", baseDist, baseTrack.distance, 0.001);
    Assert.assertEquals("Baseline nodes unchanged after finalizing A", baseNodes, baseTrack.nodes.size());

    // Verify Candidate B legs were NOT mutated by finalizing Candidate A
    for (int i = 0; i < candBLegs.size(); i++) {
      Assert.assertEquals("Candidate B leg " + i + " nodes unchanged",
        baselineRawLegs.get(i).nodes.size(), candBLegs.get(i).nodes.size());
      Assert.assertEquals("Candidate B leg " + i + " cost unchanged",
        baselineRawLegs.get(i).cost, candBLegs.get(i).cost);
    }

    // Finalize Candidate B
    FinishedCandidate finB = RefineFinalizer.finalizeCandidate(
      candBLegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);
    Assert.assertEquals(FinalizationOutcome.SUCCESS, finB.getOutcome());

    // Verify finA and finB are independent objects with matching values
    Assert.assertNotSame(finA.getTrack(), finB.getTrack());
    Assert.assertEquals(finA.getTrack().cost, finB.getTrack().cost);
  }

  @Test
  public void testFailureInjectionAfterEachStepLeavesRouteByteIdentical() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> baselineRawLegs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      OsmTrack raw = ops.findTrackTimed("fail-leg-" + i, mwps.get(i), mwps.get(i + 1), null, 5000L);
      Assert.assertNotNull(raw);
      baselineRawLegs.add(raw);
    }
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    // Baseline serialized GPX
    byte[] baselineGpx = new FormatGpx(ops.routingContext()).format(re.getFoundTrack()).getBytes(StandardCharsets.UTF_8);
    int baselineCost = re.getFoundTrack().cost;
    float baselineDist = re.getFoundTrack().distance;
    int baselineNodeCount = re.getFoundTrack().nodes.size();

    RefineFinalizer.FinalizationStep[] stepsToFail = {
      RefineFinalizer.FinalizationStep.RETRACK,
      RefineFinalizer.FinalizationStep.MERGE,
      RefineFinalizer.FinalizationStep.CLEANUP,
      RefineFinalizer.FinalizationStep.GATE,
      RefineFinalizer.FinalizationStep.PRICING,
      RefineFinalizer.FinalizationStep.RCS
    };

    for (RefineFinalizer.FinalizationStep step : stepsToFail) {
      RefineFinalizer.failureInjectionStep = step;
      RefineFinalizer.failureInjectionTimeout = false;

      List<OsmTrack> candLegs = new ArrayList<>();
      for (OsmTrack leg : baselineRawLegs) {
        candLegs.add(copyTrack(leg));
      }

      FinishedCandidate fc = RefineFinalizer.finalizeCandidate(
        candLegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);

      Assert.assertEquals("Step " + step + " must result in FAILURE", FinalizationOutcome.FAILURE, fc.getOutcome());

      // Prepare diagnostics
      RefineDiagnostics diag = new RefineDiagnostics();
      diag.refineApplied = false;
      diag.refineReason = "failed_at_" + step;

      // Publish diagnostics only on failure
      RoundTripRequest req = createRequest(ops, re, res);
      RefineStage.publishDiagnostics(req, diag);

      // Verify route state is byte-identical to baseline
      byte[] currentGpx = new FormatGpx(ops.routingContext()).format(re.getFoundTrack()).getBytes(StandardCharsets.UTF_8);
      Assert.assertTrue("Step " + step + " GPX must be byte-identical to baseline",
        Arrays.equals(baselineGpx, currentGpx));
      Assert.assertEquals("Step " + step + " cost unchanged", baselineCost, re.getFoundTrack().cost);
      Assert.assertEquals("Step " + step + " distance unchanged", baselineDist, re.getFoundTrack().distance, 0.001);
      Assert.assertEquals("Step " + step + " node count unchanged", baselineNodeCount, re.getFoundTrack().nodes.size());

      // Verify diagnostics published on lastResult
      Assert.assertNotNull(res.getRefineDiagnostics());
      Assert.assertFalse(res.getRefineDiagnostics().refineApplied);
      Assert.assertEquals("failed_at_" + step, res.getRefineDiagnostics().refineReason);
    }
  }

  @Test
  public void testDeadlineRetrackTimeout() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> baselineRawLegs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      baselineRawLegs.add(copyTrack(res.getTrack()));
    }
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    // Simulate timeout during retrack
    RefineFinalizer.failureInjectionStep = RefineFinalizer.FinalizationStep.RETRACK;
    RefineFinalizer.failureInjectionTimeout = true;

    FinishedCandidate fc = RefineFinalizer.finalizeCandidate(
      baselineRawLegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);

    Assert.assertEquals(FinalizationOutcome.TIMEOUT, fc.getOutcome());
    Assert.assertTrue("Timeout reason set", fc.getReason().contains("timeout"));

    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = false;
    diag.refineTruncated = true;
    diag.timeoutOperation = "retrack";
    diag.refineReason = "timeout_retrack";

    RoundTripRequest req = createRequest(ops, re, res);
    RefineStage.publishDiagnostics(req, diag);

    Assert.assertTrue(res.getRefineDiagnostics().refineTruncated);
    Assert.assertEquals("retrack", res.getRefineDiagnostics().timeoutOperation);
  }

  @Test
  public void testDeadlinePricingTimeout() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> baselineRawLegs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      OsmTrack raw = ops.findTrackTimed("price-time-leg-" + i, mwps.get(i), mwps.get(i + 1), null, 5000L);
      baselineRawLegs.add(raw);
    }
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    RefineFinalizer.failureInjectionStep = RefineFinalizer.FinalizationStep.PRICING;
    RefineFinalizer.failureInjectionTimeout = true;

    FinishedCandidate fc = RefineFinalizer.finalizeCandidate(
      baselineRawLegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);

    Assert.assertEquals("Expected timeout: " + fc.getReason() + " (outcome=" + fc.getOutcome() + ")", FinalizationOutcome.TIMEOUT, fc.getOutcome());
    Assert.assertTrue("Pricing timeout reason set", fc.getReason().contains("pricing"));

    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = false;
    diag.refineTruncated = true;
    diag.timeoutOperation = "pricing";
    diag.refineReason = "timeout_pricing";

    RoundTripRequest req = createRequest(ops, re, res);
    RefineStage.publishDiagnostics(req, diag);

    Assert.assertTrue(res.getRefineDiagnostics().refineTruncated);
    Assert.assertEquals("pricing", res.getRefineDiagnostics().timeoutOperation);
  }

  @Test
  public void testCancellationPropagation() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    // Mock an ops that throws watchdog cancellation during retrack
    RoundTripEngineOps cancellingOps = new DelegatingOps(ops) {
      @Override
      public OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp, OsmTrack refTrack) {
        throw new IllegalArgumentException("operation killed by thread-priority-watchdog after 5 seconds");
      }
    };

    List<MatchedWaypoint> mwps = re.getLastRoundTripResult().getMatchedWaypoints();
    List<OsmTrack> legs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      legs.add(copyTrack(re.getLastRoundTripResult().getTrack()));
    }
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    try {
      RefineFinalizer.finalizeCandidate(legs, skeleton, cancellingOps, cleanup, 1000, "trekking", 90, 2 * Math.PI * 1000,
        System.currentTimeMillis() + 10000);
      Assert.fail("Watchdog cancellation must propagate");
    } catch (IllegalArgumentException e) {
      Assert.assertTrue("Exception must be watchdog cancellation",
        e.getMessage().contains("thread-priority-watchdog"));
    }
  }

  @Test
  public void testSuccessfulFinalizationAndAtomicPublish() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> baselineRawLegs = new ArrayList<>();
    for (int i = 0; i < mwps.size() - 1; i++) {
      OsmTrack raw = ops.findTrackTimed("succ-leg-" + i, mwps.get(i), mwps.get(i + 1), null, 5000L);
      Assert.assertNotNull(raw);
      baselineRawLegs.add(raw);
    }
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    FinishedCandidate winner = RefineFinalizer.finalizeCandidate(
      baselineRawLegs, skeleton, ops, cleanup, ops.roundTripSearchRadius(), "trekking", 90, 2 * Math.PI * 1000, System.currentTimeMillis() + 10000);

    Assert.assertEquals("Expected success: " + winner.getReason(), FinalizationOutcome.SUCCESS, winner.getOutcome());
    Assert.assertNotNull(winner.getTrack());
    Assert.assertTrue("Winner oracle cost positive", winner.getOracleCostPerMeter() > 0);

    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = true;
    diag.refineReason = "accepted";
    diag.oracleCostPerMeterAfter = winner.getOracleCostPerMeter();

    RoundTripRequest req = createRequest(ops, re, res);
    RefineStage.publish(ops, req, winner, diag);

    Assert.assertSame("request.track updated", winner.getTrack(), req.track);
    Assert.assertSame("foundTrack updated", winner.getTrack(), ops.foundTrack());
    Assert.assertSame("qualityVerdict updated", winner.getQualityVerdict(), req.qualityVerdict);
    Assert.assertSame("lastResult track updated", winner.getTrack(), res.getTrack());
    Assert.assertTrue("lastResult refineDiagnostics applied", res.getRefineDiagnostics().refineApplied);
  }

  @Test
  public void publishRefreshesWaypointCountAndClearsObsoleteLegs() {
    RoundTripResult result = new RoundTripResult();
    result.setLegTracks(Arrays.asList(new OsmTrack()));
    result.setLoopWaypoints(Arrays.asList(new OsmNodeNamed()));
    result.setWithinTolerance(true);
    result.setDistanceContract(1000, 0.05);
    OsmTrack track = new OsmTrack();
    track.distance = 1200;
    List<MatchedWaypoint> points = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      MatchedWaypoint point = new MatchedWaypoint();
      point.crosspoint = new btools.mapaccess.OsmNode(180000000 + i * 100, 140000000);
      point.name = "via" + i;
      points.add(point);
    }
    RefineDiagnostics diagnostics = new RefineDiagnostics();
    diagnostics.requestedDistance = 1000;
    RoundTripRequest request = new RoundTripRequest(null);
    request.lastResult = result;
    FinishedCandidate winner = FinishedCandidate.fromBaseline(track, points,
      RoundTripQualityResult.builder().accepted(true).build(), 1, 1, "continuous");
    RefineStage.publish(null, request, winner, diagnostics);
    Assert.assertEquals(4, result.getLoopWaypoints().size());
    Assert.assertEquals(4, result.getMatchedWaypoints().size());
    Assert.assertNull("Original raw legs must not describe a refined, cleaned route", result.getLegTracks());
    Assert.assertFalse("Tolerance must describe the new distance", result.isWithinTolerance());
  }

  private static OsmTrack copyTrack(OsmTrack track) {
    OsmTrack copy = new OsmTrack();
    copy.cost = track.cost;
    copy.distance = track.distance;
    if (track.nodes != null) {
      for (OsmPathElement pe : track.nodes) {
        copy.nodes.add(OsmPathElement.create(pe.getILon(), pe.getILat(), pe.getSElev(), null));
      }
    }
    return copy;
  }

  private static class DelegatingOps implements RoundTripEngineOps {
    private final RoundTripEngineOps delegate;

    DelegatingOps(RoundTripEngineOps delegate) {
      this.delegate = delegate;
    }

    @Override
    public FastPlacementOps fastPlacementOps() {
      return delegate.fastPlacementOps();
    }

    @Override
    public double getRandomDirectionFromData(OsmNodeNamed wp, double searchRadius) {
      return delegate.getRandomDirectionFromData(wp, searchRadius);
    }

    @Override
    public void buildPointsFromCircle(List<OsmNodeNamed> waypoints, double startAngle,
                                      double searchRadius, int points) {
      delegate.buildPointsFromCircle(waypoints, startAngle, searchRadius, points);
    }

    @Override
    public void recalcTrack(OsmTrack track) {
      delegate.recalcTrack(track);
    }

    @Override
    public long remainingRequestBudgetMs() {
      return delegate.remainingRequestBudgetMs();
    }

    @Override
    public int getLinksProcessed() {
      return delegate.getLinksProcessed();
    }

    @Override
    public RoutingOutcome doRouting(long budgetMs) {
      return delegate.doRouting(budgetMs);
    }

    @Override
    public void cleanupRoutingResources() {
      delegate.cleanupRoutingResources();
    }

    @Override
    public OsmTrack findTrackTimed(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                                   OsmTrack refTrack, long budgetMs) {
      return delegate.findTrackTimed(operationName, startWp, endWp, refTrack, budgetMs);
    }

    @Override
    public OsmTrack findTrack(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp,
                              OsmTrack costCuttingTrack, OsmTrack refTrack, boolean fastPartialRecalc) {
      return delegate.findTrack(operationName, startWp, endWp, costCuttingTrack, refTrack, fastPartialRecalc);
    }

    @Override
    public OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp, MatchedWaypoint endWp) {
      return delegate.findTrackUnguided(operationName, startWp, endWp);
    }

    @Override
    public OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp, MatchedWaypoint endWp,
                                     OsmTrack refTrack) {
      return delegate.retrackForDetail(rawTrack, startWp, endWp, refTrack);
    }

    @Override
    public MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name, double maxSnapDist) {
      return delegate.profileAwareMatchPoint(ilon, ilat, name, maxSnapDist);
    }

    @Override
    public void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance) {
      delegate.matchWaypointsToNodes(waypoints, maxDistance);
    }

    @Override
    public void resetCache(boolean detailed) {
      delegate.resetCache(detailed);
    }

    @Override
    public void setTransientExpansionDeadline(long deadlineMillis) {
      delegate.setTransientExpansionDeadline(deadlineMillis);
    }

    @Override
    public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius) {
      return delegate.runIsochroneExpansion(start, searchRadius);
    }

    @Override
    public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                         OsmTrack refTrack, boolean includeCandidateTracks) {
      return delegate.runIsochroneExpansion(start, searchRadius, refTrack, includeCandidateTracks);
    }

    @Override
    public void terminate() {
      delegate.terminate();
    }

    @Override
    public boolean isTerminated() {
      return delegate.isTerminated();
    }

    @Override
    public boolean recordLegIsland(MatchedWaypoint from, MatchedWaypoint to) {
      return delegate.recordLegIsland(from, to);
    }

    @Override
    public void addTerminationHook(Runnable hook) {
      delegate.addTerminationHook(hook);
    }

    @Override
    public List<OsmNodeNamed> waypoints() {
      return delegate.waypoints();
    }

    @Override
    public OsmTrack foundTrack() {
      return delegate.foundTrack();
    }

    @Override
    public void setFoundTrack(OsmTrack track) {
      delegate.setFoundTrack(track);
    }

    @Override
    public String errorMessage() {
      return delegate.errorMessage();
    }

    @Override
    public void setErrorMessage(String message) {
      delegate.setErrorMessage(message);
    }

    @Override
    public List<MatchedWaypoint> matchedWaypoints() {
      return delegate.matchedWaypoints();
    }

    @Override
    public void setMatchedWaypoints(List<MatchedWaypoint> waypoints) {
      delegate.setMatchedWaypoints(waypoints);
    }

    @Override
    public long startTime() {
      return delegate.startTime();
    }

    @Override
    public long maxRunningTime() {
      return delegate.maxRunningTime();
    }

    @Override
    public void setMaxRunningTime(long maxRunningTimeMillis) {
      delegate.setMaxRunningTime(maxRunningTimeMillis);
    }

    @Override
    public long roundTripRequestDeadline() {
      return delegate.roundTripRequestDeadline();
    }

    @Override
    public void setRoundTripRuntimeHints(RoundTripRuntimeHints hints) {
      delegate.setRoundTripRuntimeHints(hints);
    }

    @Override
    public long roundTripRoutingBudgetMs() {
      return delegate.roundTripRoutingBudgetMs();
    }

    @Override
    public RoundTripEffortPolicy roundTripEffortPolicy() {
      return delegate.roundTripEffortPolicy();
    }

    @Override
    public void setRoundTripEffortPolicy(RoundTripEffortPolicy policy) {
      delegate.setRoundTripEffortPolicy(policy);
    }

    @Override
    public void setLastRejectedTrack(OsmTrack track) {
      delegate.setLastRejectedTrack(track);
    }

    @Override
    public void setLastRoundTripResult(RoundTripResult result) {
      delegate.setLastRoundTripResult(result);
    }

    @Override
    public void setLastRoundTripQuality(RoundTripQualityResult quality) {
      delegate.setLastRoundTripQuality(quality);
    }

    @Override
    public void addLinksProcessed(long links) {
      delegate.addLinksProcessed(links);
    }

    @Override
    public void logInfo(String message) {
      delegate.logInfo(message);
    }

    @Override
    public void logException(Throwable t) {
      delegate.logException(t);
    }

    @Override
    public void logThrowable(Throwable t) {
      delegate.logThrowable(t);
    }

    @Override
    public void writeAdoptedTrackOutput(OsmTrack track) {
      delegate.writeAdoptedTrackOutput(track);
    }

    @Override
    public void consolidateRoundTripVoiceHints(OsmTrack track) {
      delegate.consolidateRoundTripVoiceHints(track);
    }

    @Override
    public RoutingContext routingContext() {
      return delegate.routingContext();
    }

    @Override
    public File segmentDir() {
      return delegate.segmentDir();
    }

    @Override
    public boolean isRoundTripMode() {
      return delegate.isRoundTripMode();
    }

    @Override
    public boolean explicitViaRoundTrip() {
      return delegate.explicitViaRoundTrip();
    }

    @Override
    public double roundTripSearchRadius() {
      return delegate.roundTripSearchRadius();
    }

    @Override
    public boolean roundTripFerriesAllowed() {
      return delegate.roundTripFerriesAllowed();
    }

    @Override
    public boolean roundTripQualityHardReject(RoundTripQualityResult quality) {
      return delegate.roundTripQualityHardReject(quality);
    }

    @Override
    public int walkPathCost(OsmTrack track, MatchedWaypoint startWp, MatchedWaypoint endWp) {
      return delegate.walkPathCost(track, startWp, endWp);
    }

    @Override
    public int walkLoopCost(List<OsmTrack> legs, List<MatchedWaypoint> waypoints) {
      return delegate.walkLoopCost(legs, waypoints);
    }

    @Override
    public void setLastRefineDiagnostics(RefineDiagnostics diag) {
      delegate.setLastRefineDiagnostics(diag);
    }

    @Override
    public RefineDiagnostics lastRefineDiagnostics() {
      return delegate.lastRefineDiagnostics();
    }
  }
}
