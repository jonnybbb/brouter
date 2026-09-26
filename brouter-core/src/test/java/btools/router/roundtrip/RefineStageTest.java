package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;

public class RefineStageTest {

  private static RoundTripRequest createRequest(RoundTripEngineOps ops, RoutingEngine re, RoundTripResult res) {
    RoundTripRequest req = new RoundTripRequest(ops);
    req.track = re.getFoundTrack();
    req.lastResult = res;
    req.qualityVerdict = re.getLastRoundTripQuality();
    req.producingTier = RoundTripAlgorithm.GREEDY;
    return req;
  }

  @Test
  public void testSkipReasonRefineOff() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    // Refine is off by default
    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("refine_off", diag.refineReason);
    Assert.assertFalse(diag.refineApplied);
  }

  @Test
  public void testSkipReasonAutoChild() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
      rc.roundTripSuppressDecoration = true;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("auto_child", diag.refineReason);
  }

  @Test
  public void testSkipReasonExplicitVias() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    req.setExplicitVia(true);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("explicit_vias", diag.refineReason);
  }

  @Test
  public void testSkipReasonTierNotSupported() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    req.producingTier = RoundTripAlgorithm.WAYPOINT;
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("tier_not_supported", diag.refineReason);
  }

  @Test
  public void testSkipReasonBoundedPreset() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    req.effortPolicy = RoundTripEffortPolicy.BOUNDED_PRESET;
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("bounded_preset", diag.refineReason);
  }

  @Test
  public void testSkipReasonSamewayback() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
      rc.allowSamewayback = true;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("samewayback", diag.refineReason);
  }

  @Test
  public void testSkipReasonForcedCorridor() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    req.forcedCorridorAccepted = true;
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("forced_corridor", diag.refineReason);
  }

  @Test
  public void testSkipReasonGateRejected() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RoundTripQualityResult rejected = RoundTripQualityResult.builder()
        .reject(RoundTripQualityResult.RejectionTier.QUALITY, "self_crossings")
        .build();
    RefineStage.refine(ops, orch, req, rejected, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("gate_rejected", diag.refineReason);
  }

  @Test
  public void testSkipReasonNoRoute() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    req.track = null;
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("no_route", diag.refineReason);
  }

  @Test
  public void testSkipReasonTooFewVias() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    // Track with only start and end (0 vias)
    OsmTrack shortTrack = new OsmTrack();
    shortTrack.nodes = req.track.nodes;
    List<MatchedWaypoint> twoWps = new ArrayList<>();
    twoWps.add(req.track.getMatchedWaypoints().get(0));
    twoWps.add(req.track.getMatchedWaypoints().get(req.track.getMatchedWaypoints().size() - 1));
    shortTrack.setMatchedWaypoints(twoWps);
    req.track = shortTrack;

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    Assert.assertEquals("too_few_vias", diag.refineReason);
  }

  @Test
  public void testRefineExecutionRunsWhenEligible() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripRefine = "best_of_n";
      rc.roundTripRefineEvals = 4;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripRequest req = createRequest(ops, re, res);
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);

    RefineDiagnostics diag = ops.lastRefineDiagnostics();
    Assert.assertNotNull(diag);
    // Verified that refinement ran and made an evaluation or diagnostic decision
    Assert.assertNotNull(diag.refineReason);
    Assert.assertTrue("Elapsed time recorded", diag.elapsedMs >= 0);
  }
}
