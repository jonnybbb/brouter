package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;
import btools.router.RoutingParamCollector;

public class RefineParameterAndSkipReasonTest {

  @Test
  public void testParameterClamping() {
    RoutingParamCollector collector = new RoutingParamCollector();

    // 1. roundTripRefine mode values
    RoutingContext rc1 = new RoutingContext();
    Map<String, String> p1 = new HashMap<>();
    p1.put("roundTripRefine", "LOCAL");
    collector.setParams(rc1, null, p1);
    Assert.assertEquals("local", rc1.roundTripRefine);

    RoutingContext rc2 = new RoutingContext();
    Map<String, String> p2 = new HashMap<>();
    p2.put("roundTripRefine", "ANNEAL");
    collector.setParams(rc2, null, p2);
    Assert.assertEquals("anneal", rc2.roundTripRefine);

    RoutingContext rc3 = new RoutingContext();
    Map<String, String> p3 = new HashMap<>();
    p3.put("roundTripRefine", "invalid_mode");
    collector.setParams(rc3, null, p3);
    Assert.assertEquals("none", rc3.roundTripRefine);

    // 2. roundTripRefineEvals clamping [1, 64]
    RoutingContext rc4 = new RoutingContext();
    Map<String, String> p4 = new HashMap<>();
    p4.put("roundTripRefineEvals", "-10");
    collector.setParams(rc4, null, p4);
    Assert.assertEquals(1, (int) rc4.roundTripRefineEvals);

    RoutingContext rc5 = new RoutingContext();
    Map<String, String> p5 = new HashMap<>();
    p5.put("roundTripRefineEvals", "200");
    collector.setParams(rc5, null, p5);
    Assert.assertEquals(64, (int) rc5.roundTripRefineEvals);

    RoutingContext rc6 = new RoutingContext();
    Map<String, String> p6 = new HashMap<>();
    p6.put("roundTripRefineEvals", "24");
    collector.setParams(rc6, null, p6);
    Assert.assertEquals(24, (int) rc6.roundTripRefineEvals);

    // 3. roundTripRefineMaxMs clamping [500, 8000]
    RoutingContext rc7 = new RoutingContext();
    Map<String, String> p7 = new HashMap<>();
    p7.put("roundTripRefineMaxMs", "100");
    collector.setParams(rc7, null, p7);
    Assert.assertEquals(500L, (long) rc7.roundTripRefineMaxMs);

    RoutingContext rc8 = new RoutingContext();
    Map<String, String> p8 = new HashMap<>();
    p8.put("roundTripRefineMaxMs", "50000");
    collector.setParams(rc8, null, p8);
    Assert.assertEquals(8000L, (long) rc8.roundTripRefineMaxMs);

    RoutingContext rc9 = new RoutingContext();
    Map<String, String> p9 = new HashMap<>();
    p9.put("roundTripRefineMaxMs", "4000");
    collector.setParams(rc9, null, p9);
    Assert.assertEquals(4000L, (long) rc9.roundTripRefineMaxMs);
  }

  @Test
  public void testChildSuppressionAndContextCopy() {
    RoutingContext parent = new RoutingContext();
    parent.roundTripRefine = "local";
    parent.roundTripRefineEvals = 32;
    parent.roundTripRefineMaxMs = 4000L;
    parent.roundTripSuppressDecoration = false;

    RoutingContext child = parent.copyRequestFields();
    // Refine parameters MUST NOT be propagated to child context (§4.6)
    Assert.assertNull("Child context must not inherit roundTripRefine", child.roundTripRefine);
    Assert.assertNull("Child context must not inherit roundTripRefineEvals", child.roundTripRefineEvals);
    Assert.assertNull("Child context must not inherit roundTripRefineMaxMs", child.roundTripRefineMaxMs);
  }

  @Test
  public void testAllTenSkipReasons() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripOrchestrator orch = new RoundTripOrchestrator(ops);

    // 1. refine_off
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "none";
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("refine_off", ops.lastRefineDiagnostics().refineReason);
    }

    // 2. auto_child
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      ops.routingContext().roundTripSuppressDecoration = true;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("auto_child", ops.lastRefineDiagnostics().refineReason);
      ops.routingContext().roundTripSuppressDecoration = false;
    }

    // 3. explicit_vias
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      req.setExplicitVia(true);
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("explicit_vias", ops.lastRefineDiagnostics().refineReason);
    }

    // 4. tier_not_supported
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      req.producingTier = RoundTripAlgorithm.WAYPOINT;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("tier_not_supported", ops.lastRefineDiagnostics().refineReason);
    }

    // 5. bounded_preset
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      req.effortPolicy = RoundTripEffortPolicy.BOUNDED_PRESET;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("bounded_preset", ops.lastRefineDiagnostics().refineReason);
    }

    // 6. samewayback
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      ops.routingContext().allowSamewayback = true;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("samewayback", ops.lastRefineDiagnostics().refineReason);
      ops.routingContext().allowSamewayback = false;
    }

    // 7. forced_corridor
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      req.forcedCorridorAccepted = true;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("forced_corridor", ops.lastRefineDiagnostics().refineReason);
    }

    // 8. gate_rejected
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      RoundTripQualityResult rejected = RoundTripQualityResult.builder()
          .reject(RoundTripQualityResult.RejectionTier.QUALITY, "retrace")
          .build();
      RefineStage.refine(ops, orch, req, rejected, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("gate_rejected", ops.lastRefineDiagnostics().refineReason);
    }

    // 9. no_route
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      req.track = null;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("no_route", ops.lastRefineDiagnostics().refineReason);
    }

    // 10. too_few_vias
    {
      RoundTripRequest req = createRequest(ops, re, res);
      ops.routingContext().roundTripRefine = "local";
      OsmTrack shortTrack = new OsmTrack();
      shortTrack.nodes = req.track.nodes;
      List<MatchedWaypoint> twoWps = new ArrayList<>();
      twoWps.add(req.track.getMatchedWaypoints().get(0));
      twoWps.add(req.track.getMatchedWaypoints().get(req.track.getMatchedWaypoints().size() - 1));
      shortTrack.setMatchedWaypoints(twoWps);
      req.track = shortTrack;
      RefineStage.refine(ops, orch, req, req.qualityVerdict, 1000.0, 90.0, RoundTripAlgorithm.GREEDY);
      Assert.assertEquals("too_few_vias", ops.lastRefineDiagnostics().refineReason);
    }
  }

  private static RoundTripRequest createRequest(RoundTripEngineOps ops, RoutingEngine re, RoundTripResult res) {
    RoundTripRequest req = new RoundTripRequest(ops);
    req.track = re.getFoundTrack();
    req.lastResult = res;
    req.qualityVerdict = re.getLastRoundTripQuality();
    req.producingTier = RoundTripAlgorithm.GREEDY;
    return req;
  }
}
