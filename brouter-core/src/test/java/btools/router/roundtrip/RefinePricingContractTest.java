package btools.router.roundtrip;

import org.junit.Test;

import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class RefinePricingContractTest {
  @Test
  public void pricesOriginalAndCleanedGeometryInsteadOfReconstructedRawLegs() {
    RoutingEngine engine = RoundTripFixture.engine("gravel", 0, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult baseline = engine.getLastRoundTripResult();
    RoundTripEngineOps ops = engine.roundTripOps();
    RefineSkeleton skeleton = RefineInitializer.extractSkeleton(baseline.getTrack(), baseline.getMatchedWaypoints());
    double originalCost = LoopCostOracle.price(ops, baseline.getTrack(), baseline.getMatchedWaypoints());
    assertTrue(originalCost > 0);
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    RefineInitResult init = RefineInitializer.initialize(ops, new DefaultLegEvaluator(ops), skeleton,
      cleanup, baseline.getTrack(), 1000, "gravel", 0, 2 * Math.PI * 1000,
      System.currentTimeMillis() + 10000, true);
    assertTrue(init.isSuccess());
    assertEquals("Baseline must price the original tier track", originalCost,
      init.getBaselineOracleCostPerMeter(), 0.000001);
    FinishedCandidate candidate = init.getRebuiltCandidate();
    assertTrue(candidate.getReason(), candidate.isSuccess());
    double finishedCost = LoopCostOracle.price(ops, candidate.getTrack(), candidate.getMatchedWaypoints());
    assertTrue(finishedCost > 0);
    assertEquals("Candidate must price its cleaned geometry", finishedCost,
      candidate.getOracleCostPerMeter(), 0.000001);
    double baselineRcs = RouteChoiceScore.score(baseline.getTrack(), 2 * Math.PI * 1000,
      "gravel", engine.getLastRoundTripQuality(), 90).score();
    FinishedCandidate original = FinishedCandidate.fromBaseline(baseline.getTrack(), baseline.getMatchedWaypoints(),
      engine.getLastRoundTripQuality(), originalCost, baselineRcs, "continuous");
    assertTrue("This real reconstruction costs more than the original", finishedCost > originalCost);
    assertTrue(ShipPredicate.evaluate(candidate, original, new RefineConfig(), 2 * Math.PI * 1000)
      .getReason().startsWith("cost_not_improved"));
    for (String unsafeMethod : new String[]{"per_leg", "unknown"}) {
      FinishedCandidate unsafe = new FinishedCandidate(FinalizationOutcome.SUCCESS, "test", candidate.getTrack(),
        candidate.getMatchedWaypoints(), candidate.getQualityVerdict(), candidate.getOracleCostPerMeter(),
        candidate.getRcs(), unsafeMethod);
      assertTrue(ShipPredicate.evaluate(unsafe, original, new RefineConfig(), 2 * Math.PI * 1000)
        .getReason().startsWith("pricing_method_mismatch"));
    }
  }
  @Test
  public void rejectsExistingMixedGeometryBaselineWithoutReconstructingIt() {
    RoutingEngine engine = RoundTripFixture.engine("gravel", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripRefine = "local";
    });
    RoundTripResult result = engine.getLastRoundTripResult();
    long signature = LoopCostOracle.geometrySignature(result.getTrack());
    assertTrue(LoopCostOracle.price(engine.roundTripOps(), result.getTrack(), result.getMatchedWaypoints()) < 0);
    assertTrue(engine.getLastPricingFailure().startsWith("geometry_mismatch"));
    org.junit.Assert.assertFalse(engine.getLastRefineDiagnostics().refineApplied);
    assertEquals(signature, LoopCostOracle.geometrySignature(result.getTrack()));
  }

}
