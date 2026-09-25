package btools.router.roundtrip;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.FormatGpx;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;

public class RefineSingleMoveTest {

  @Test
  public void testSingleMoveEndToEndAcceptPath() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "move-accept");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;

    // 1. Comparator rule (§4.5): baseline is the original tier loop
    double baseOracleCost = LoopCostOracle.price(ops, res.getTrack(), res.getMatchedWaypoints());
    double baseRcs = RouteChoiceScore.score(res.getTrack(), reqDist, "trekking",
      re.getLastRoundTripQuality(), 90).score();
    FinishedCandidate baseline = FinishedCandidate.fromBaseline(
      res.getTrack(), res.getMatchedWaypoints(), re.getLastRoundTripQuality(),
      baseOracleCost, baseRcs);
    Assert.assertTrue("Baseline must be success", baseline.isSuccess());

    // 2. Initialize leg cache
    RefineInitResult initResult = RefineInitializer.initialize(
      ops, evaluator, skeleton, cleanup, res.getTrack(), radius, "trekking", 90,
      reqDist, System.currentTimeMillis() + 10000L);
    Assert.assertTrue("Initialization must succeed", initResult.isSuccess());

    // 3. Generate a MOVE proposal
    RefineConfig config = new RefineConfig();
    MoveProposalOperator mover = new MoveProposalOperator(ops, config);
    SplitmixRandom rng = new SplitmixRandom(42L);

    MoveProposalOperator.MoveProposal proposal = null;
    for (int p = 0; p < 20; p++) {
      proposal = mover.proposeMove(skeleton, skeleton, rng, radius, reqDist);
      if (proposal.isFeasible()) {
        break;
      }
    }
    Assert.assertNotNull("Should find a feasible proposal", proposal);
    Assert.assertTrue("Proposal must be feasible", proposal.isFeasible());

    // 4. Re-route only affected legs
    int movedIdx = proposal.getMovedViaIndex();
    RefineSkeleton mutatedSkeleton = proposal.getMutatedSkeleton();
    List<MatchedWaypoint> mutWps = mutatedSkeleton.getWaypoints();

    List<OsmTrack> candLegs = new ArrayList<>(skeleton.getLegCount());
    LegCache cache = initResult.getLegCache();

    for (int i = 0; i < skeleton.getLegCount(); i++) {
      MatchedWaypoint from = mutWps.get(i);
      MatchedWaypoint to = mutWps.get(i + 1);

      if (i == movedIdx || i == movedIdx + 1) {
        OsmTrack leg = evaluator.route(from, to, 5000L);
        Assert.assertNotNull("Mutated leg must route", leg);
        cache.put(from, to, leg);
        candLegs.add(leg);
      } else {
        OsmTrack cached = cache.get(from, to);
        Assert.assertNotNull("Unaffected leg must be in cache", cached);
        candLegs.add(cached);
      }
    }

    // 5. Finalize candidate
    FinishedCandidate finalist = RefineFinalizer.finalizeCandidate(
      candLegs, mutatedSkeleton, ops, cleanup, radius, "trekking", 90,
      reqDist, System.currentTimeMillis() + 10000L);
    Assert.assertTrue("Finalization must succeed: " + finalist.getReason(), finalist.isSuccess());

    // 6. Test accept path: construct an accepted winner meeting all ship predicate criteria
    FinishedCandidate winningCandidate = new FinishedCandidate(
      FinalizationOutcome.SUCCESS, "winning_candidate",
      finalist.getTrack(),
      finalist.getMatchedWaypoints(),
      finalist.getQualityVerdict(),
      baseline.getOracleCostPerMeter() * 0.98, // 2% lower cost per meter
      baseline.getRcs() + 0.01 // higher RCS
    );

    ShipPredicate.Result predResult = ShipPredicate.evaluate(winningCandidate, baseline, config, reqDist);
    Assert.assertTrue("Predicate must accept winning candidate: " + predResult.getReason(), predResult.isAccepted());

    // 7. Publish winner atomically
    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = true;
    diag.refineReason = predResult.getReason();
    diag.oracleCostPerMeterBefore = baseline.getOracleCostPerMeter();
    diag.oracleCostPerMeterAfter = winningCandidate.getOracleCostPerMeter();
    diag.rcsBefore = baseline.getRcs();
    diag.rcsAfter = winningCandidate.getRcs();
    diag.evaluations = 1;
    diag.proposals = 1;

    RoundTripRequest req = createRequest(ops, re, res);
    RefineStage.publish(ops, req, winningCandidate, diag);

    Assert.assertSame("Request track published", winningCandidate.getTrack(), req.track);
    Assert.assertSame("Found track published", winningCandidate.getTrack(), ops.foundTrack());
    Assert.assertSame("Quality verdict published", winningCandidate.getQualityVerdict(), req.qualityVerdict);
    Assert.assertSame("Result track published", winningCandidate.getTrack(), res.getTrack());
    Assert.assertTrue("Diagnostics published", res.getRefineDiagnostics().refineApplied);
    Assert.assertNotNull("Engine diagnostics published", ops.lastRefineDiagnostics());
  }

  @Test
  public void testSingleMoveRejectPathLeavesBaselineByteIdentical() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "move-reject");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;

    // Snapshot baseline track & GPX
    byte[] baselineGpx = new FormatGpx(ops.routingContext()).format(re.getFoundTrack()).getBytes(StandardCharsets.UTF_8);
    int baselineCost = re.getFoundTrack().cost;
    float baselineDist = re.getFoundTrack().distance;
    int baselineNodes = re.getFoundTrack().nodes.size();

    // Snapshot baseline engine matched waypoints to test isolation (§4.5 ownership rule)
    List<MatchedWaypoint> baselineWps = new ArrayList<>();
    for (MatchedWaypoint wp : ops.matchedWaypoints()) {
      baselineWps.add(RefineSkeleton.copyWaypoint(wp));
    }

    // 1. Comparator rule (§4.5): baseline is original tier loop
    double baseOracleCost = LoopCostOracle.price(ops, res.getTrack(), res.getMatchedWaypoints());
    double baseRcs = RouteChoiceScore.score(res.getTrack(), reqDist, "trekking",
      re.getLastRoundTripQuality(), 90).score();
    FinishedCandidate baseline = FinishedCandidate.fromBaseline(
      res.getTrack(), res.getMatchedWaypoints(), re.getLastRoundTripQuality(),
      baseOracleCost, baseRcs);

    // 2. Initialize
    RefineInitResult initResult = RefineInitializer.initialize(
      ops, evaluator, skeleton, cleanup, res.getTrack(), radius, "trekking", 90,
      reqDist, System.currentTimeMillis() + 10000L);
    Assert.assertTrue("Initialization must succeed", initResult.isSuccess());

    // 3. Create and finalize a speculative candidate
    MoveProposalOperator mover = new MoveProposalOperator(ops, new RefineConfig());
    SplitmixRandom rng = new SplitmixRandom(99L);
    MoveProposalOperator.MoveProposal proposal = mover.proposeMove(skeleton, skeleton, rng, radius, reqDist);
    if (proposal.isFeasible()) {
      RefineSkeleton mutSkel = proposal.getMutatedSkeleton();
      List<OsmTrack> candLegs = new ArrayList<>();
      for (int i = 0; i < mutSkel.getLegCount(); i++) {
        candLegs.add(evaluator.route(mutSkel.getWaypoints().get(i), mutSkel.getWaypoints().get(i + 1), 5000L));
      }
      // Running speculative candidate finalization must NEVER mutate ops.matchedWaypoints()!
      FinishedCandidate speculative = RefineFinalizer.finalizeCandidate(
        candLegs, mutSkel, ops, cleanup, radius, "trekking", 90, reqDist, System.currentTimeMillis() + 10000L);
      Assert.assertNotNull("Speculative candidate outcome exists", speculative.getOutcome());
    }

    // Verify engine matched waypoints remained completely intact and isolated during speculative finalization!
    List<MatchedWaypoint> afterFinalizeWps = ops.matchedWaypoints();
    Assert.assertEquals("Engine matched waypoints count unchanged", baselineWps.size(), afterFinalizeWps.size());
    for (int i = 0; i < baselineWps.size(); i++) {
      Assert.assertEquals("Waypoint " + i + " node1 unchanged",
        baselineWps.get(i).node1.getIdFromPos(), afterFinalizeWps.get(i).node1.getIdFromPos());
      Assert.assertEquals("Waypoint " + i + " node2 unchanged",
        baselineWps.get(i).node2.getIdFromPos(), afterFinalizeWps.get(i).node2.getIdFromPos());
      Assert.assertEquals("Waypoint " + i + " crosspoint unchanged",
        baselineWps.get(i).crosspoint.getIdFromPos(), afterFinalizeWps.get(i).crosspoint.getIdFromPos());
    }

    // 4. Create a deliberately worse candidate (higher cost) to test reject path
    FinishedCandidate worseCandidate = new FinishedCandidate(
      FinalizationOutcome.SUCCESS, "worse_candidate",
      LegCache.copyTrack(baseline.getTrack()),
      baseline.getMatchedWaypoints(),
      baseline.getQualityVerdict(),
      baseline.getOracleCostPerMeter() * 1.5, // 50% higher cost
      baseline.getRcs()
    );

    RefineConfig config = new RefineConfig();
    ShipPredicate.Result predResult = ShipPredicate.evaluate(worseCandidate, baseline, config, reqDist);
    Assert.assertFalse("Predicate must reject worse candidate", predResult.isAccepted());
    Assert.assertTrue("Reason mentions cost", predResult.getReason().contains("cost_not_improved"));

    // On rejection, publish diagnostics only
    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = false;
    diag.refineReason = predResult.getReason();
    diag.oracleCostPerMeterBefore = baseline.getOracleCostPerMeter();
    diag.rcsBefore = baseline.getRcs();
    diag.evaluations = 1;
    diag.proposals = 1;

    RoundTripRequest req = createRequest(ops, re, res);
    RefineStage.publishDiagnostics(req, diag);

    // Verify baseline track is completely byte-identical
    byte[] currentGpx = new FormatGpx(ops.routingContext()).format(re.getFoundTrack()).getBytes(StandardCharsets.UTF_8);
    Assert.assertTrue("GPX output must be byte-identical to baseline on reject",
      Arrays.equals(baselineGpx, currentGpx));
    Assert.assertEquals("Cost unchanged", baselineCost, re.getFoundTrack().cost);
    Assert.assertEquals("Distance unchanged", baselineDist, re.getFoundTrack().distance, 0.001);
    Assert.assertEquals("Node count unchanged", baselineNodes, re.getFoundTrack().nodes.size());

    // Verify diagnostics recorded on lastResult
    Assert.assertNotNull(res.getRefineDiagnostics());
    Assert.assertFalse("refineApplied is false", res.getRefineDiagnostics().refineApplied);
    Assert.assertEquals(predResult.getReason(), res.getRefineDiagnostics().refineReason);
  }

  private static RoundTripRequest createRequest(RoundTripEngineOps ops, RoutingEngine re, RoundTripResult res) {
    RoundTripRequest req = new RoundTripRequest(ops);
    req.track = re.getFoundTrack();
    req.lastResult = res;
    req.qualityVerdict = re.getLastRoundTripQuality();
    return req;
  }
}
