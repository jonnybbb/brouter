package btools.router.roundtrip;

import java.util.Collections;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;

public class RefineSearchTest {

  @Test
  public void testSearchBestOfN() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "search-test");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;
    long deadline = System.currentTimeMillis() + 10000L;

    RefineInitResult init = RefineInitializer.initialize(
      ops, evaluator, skeleton,
      new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops),
      res.getTrack(), radius, "trekking", 90, reqDist, deadline);
    Assert.assertTrue("Initialization should succeed", init.isSuccess());

    RefineConfig config = new RefineConfig();
    config.mode = RefineConfig.Mode.BEST_OF_N;
    config.evaluations = 4;
    config.topKFinalists = 2;

    MoveProposalOperator moveOp = new MoveProposalOperator(ops, config);
    RefineSearch search = new RefineSearch(
      ops, evaluator, init.getLegCache(), config, skeleton,
      init.getRawLegs(), LoopCostOracle.evaluate(ops, init.getRawLegs(), skeleton.getWaypoints(), deadline),
      moveOp, radius, reqDist, 42, deadline);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult searchResult = search.search(diag);

    Assert.assertNotNull(searchResult);
    Assert.assertTrue(diag.evaluations <= 4);
    Assert.assertTrue(diag.proposals >= diag.evaluations);
    Assert.assertTrue(searchResult.getFinalists().size() <= 2);

    // If finalists exist, check energy ordering
    List<RefineSearch.SearchCandidate> finalists = searchResult.getFinalists();
    if (finalists.size() > 1) {
      Assert.assertTrue("Finalists must be ordered by energy ascending",
        finalists.get(0).getEnergy() <= finalists.get(1).getEnergy());
    }
  }

  @Test
  public void testSearchLocal() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "search-local");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;
    long deadline = System.currentTimeMillis() + 10000L;

    RefineInitResult init = RefineInitializer.initialize(
      ops, evaluator, skeleton,
      new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops),
      res.getTrack(), radius, "trekking", 90, reqDist, deadline);
    Assert.assertTrue("Initialization should succeed", init.isSuccess());

    RefineConfig config = new RefineConfig();
    config.mode = RefineConfig.Mode.LOCAL;
    config.evaluations = 4;
    config.topKFinalists = 2;

    MoveProposalOperator moveOp = new MoveProposalOperator(ops, config);
    RefineSearch search = new RefineSearch(
      ops, evaluator, init.getLegCache(), config, skeleton,
      init.getRawLegs(), LoopCostOracle.evaluate(ops, init.getRawLegs(), skeleton.getWaypoints(), deadline),
      moveOp, radius, reqDist, 42, deadline);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult searchResult = search.search(diag);

    Assert.assertNotNull(searchResult);
    Assert.assertTrue(diag.evaluations <= 4);
  }

  @Test
  public void testSearchAnneal() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "search-anneal");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;
    long deadline = System.currentTimeMillis() + 10000L;

    RefineInitResult init = RefineInitializer.initialize(
      ops, evaluator, skeleton,
      new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops),
      res.getTrack(), radius, "trekking", 90, reqDist, deadline);
    Assert.assertTrue("Initialization should succeed", init.isSuccess());

    RefineConfig config = new RefineConfig();
    config.mode = RefineConfig.Mode.ANNEAL;
    config.evaluations = 4;
    config.topKFinalists = 2;

    MoveProposalOperator moveOp = new MoveProposalOperator(ops, config);
    RefineSearch search = new RefineSearch(
      ops, evaluator, init.getLegCache(), config, skeleton,
      init.getRawLegs(), LoopCostOracle.evaluate(ops, init.getRawLegs(), skeleton.getWaypoints(), deadline),
      moveOp, radius, reqDist, 42, deadline);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult searchResult = search.search(diag);

    Assert.assertNotNull(searchResult);
    Assert.assertTrue(diag.evaluations <= 4);
  }

  @Test
  public void testSearchTimeoutEarlyTermination() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "search-timeout");
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;

    // Expired deadline
    long deadline = System.currentTimeMillis() - 50L;

    RefineConfig config = new RefineConfig();
    config.evaluations = 10;
    MoveProposalOperator moveOp = new MoveProposalOperator(ops, config);

    RefineSearch search = new RefineSearch(
      ops, evaluator, new LegCache(), config, skeleton,
      Collections.singletonList(res.getTrack()), LoopPrice.success(1, 1, 0),
      moveOp, radius, reqDist, 42, deadline);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult result = search.search(diag);

    Assert.assertTrue("Should mark truncated", diag.refineTruncated);
    Assert.assertEquals("search_evaluation", diag.timeoutOperation);
    Assert.assertTrue(result.getFinalists().isEmpty());
  }

  @Test
  public void testSearchEmptySkeletonOrLegs() {
    RefineConfig config = new RefineConfig();
    RefineSearch search = new RefineSearch(
      null, null, new LegCache(), config, null,
      Collections.<OsmTrack>emptyList(), LoopPrice.success(1, 1, 0),
      null, 1000.0, 6000.0, 42, 0L);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult result = search.search(diag);

    Assert.assertNotNull(result);
    Assert.assertTrue(result.getFinalists().isEmpty());
  }

  @Test
  public void testCacheHitsIncrementedOnRepeatedLeg() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegCache legCache = new LegCache();
    // Preload legCache with the first leg
    MatchedWaypoint w0 = mwps.get(0);
    MatchedWaypoint w1 = mwps.get(1);
    legCache.put(w0, w1, res.getTrack());

    Assert.assertNotNull(legCache.get(w0, w1));
    Assert.assertEquals(1, legCache.getHits());
  }
}
