package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;
import btools.util.CheapRuler;

public class RefineDirectionAndOperatorTest {

  @Test
  public void testBearingWraparoundAndSymmetry() {
    // 1. 359° vs 1° wraparound: angular delta must be 2.0°, NOT 358.0°
    double baseBearing = 359.0;
    double candBearing = 1.0;
    double diff = (candBearing - baseBearing + 540.0) % 360.0 - 180.0;
    Assert.assertEquals(2.0, Math.abs(diff), 1e-6);

    // 2. Symmetric wraparound: 1° vs 359°
    double reverseDiff = (baseBearing - candBearing + 540.0) % 360.0 - 180.0;
    Assert.assertEquals(2.0, Math.abs(reverseDiff), 1e-6);

    // 3. East (90°) vs West (270°): opposite direction must be 180°
    double eastBearing = 90.0;
    double westBearing = 270.0;
    double oppositeDiff = (westBearing - eastBearing + 540.0) % 360.0 - 180.0;
    Assert.assertEquals(180.0, Math.abs(oppositeDiff), 1e-6);
  }

  @Test
  public void testFarthestPointOnCleanedRoute() {
    // Verify that ShipPredicate checks the cleaned route nodes, not just via crosspoints
    OsmTrack baseTrack = new OsmTrack();
    baseTrack.distance = 10000;
    baseTrack.nodes = new ArrayList<>();
    // Start at (0, 0)
    baseTrack.nodes.add(OsmPathElement.create(0, 0, (short) 0, null));
    // Midpoint at (10000, 10000)
    baseTrack.nodes.add(OsmPathElement.create(10000, 10000, (short) 0, null));
    // Farthest node at (20000, 20000)
    baseTrack.nodes.add(OsmPathElement.create(20000, 20000, (short) 0, null));
    // Return to (0, 0)
    baseTrack.nodes.add(OsmPathElement.create(0, 0, (short) 0, null));

    RoundTripQualityResult accepted = RoundTripQualityResult.builder().accepted(true).build();
    FinishedCandidate base = FinishedCandidate.fromBaseline(baseTrack, new ArrayList<>(), accepted, 1.0, 0.8, "continuous");

    // Candidate with same farthest point bearing (North-East)
    OsmTrack candTrack = new OsmTrack();
    candTrack.distance = 10000;
    candTrack.nodes = new ArrayList<>();
    candTrack.nodes.add(OsmPathElement.create(0, 0, (short) 0, null));
    candTrack.nodes.add(OsmPathElement.create(20000, 20000, (short) 0, null));
    candTrack.nodes.add(OsmPathElement.create(0, 0, (short) 0, null));
    FinishedCandidate cand = FinishedCandidate.fromBaseline(candTrack, new ArrayList<>(), accepted, 0.9, 0.8, "continuous");

    RefineConfig cfg = new RefineConfig();
    ShipPredicate.Result res = ShipPredicate.evaluate(cand, base, cfg, 10000.0);
    Assert.assertTrue("Candidate with aligned bearing must pass: " + res.getReason(), res.isAccepted());
  }

  @Test
  public void testRepeatedMovesDoNotDriftBeyondLimit() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton origSkeleton = new RefineSkeleton(
      mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    RefineConfig config = new RefineConfig();
    MoveProposalOperator mover = new MoveProposalOperator(ops, config);
    SplitmixRandom rng = new SplitmixRandom(12345L);
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;
    double maxDisp = config.maxDisplacementFraction * radius;

    RefineSkeleton currentSkeleton = origSkeleton;
    int acceptedMoves = 0;
    for (int step = 0; step < 50; step++) {
      MoveProposalOperator.MoveProposal prop = mover.proposeMove(
        currentSkeleton, origSkeleton, rng, radius, reqDist);
      if (prop.isFeasible()) {
        currentSkeleton = prop.getMutatedSkeleton();
        acceptedMoves++;
        assertGenerated(prop);
        // Check that every via in currentSkeleton is within maxDisp from origSkeleton
        for (int v = 0; v < currentSkeleton.getVias().size(); v++) {
          MatchedWaypoint cur = currentSkeleton.getVias().get(v);
          MatchedWaypoint orig = origSkeleton.getVias().get(v);
          double d = CheapRuler.distance(orig.crosspoint.ilon, orig.crosspoint.ilat,
            cur.crosspoint.ilon, cur.crosspoint.ilat);
          Assert.assertTrue("Via displacement from original must not exceed maxDisp: " + d + " > " + maxDisp,
            d <= maxDisp + 1.0); // 1m tolerance for integer truncation
        }
      }
    }
    Assert.assertTrue("Should generate at least some feasible moves", acceptedMoves > 0);
  }

  @Test
  public void testProposalOperatorsOnFixture() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton origSkeleton = new RefineSkeleton(
      mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    RefineConfig config = new RefineConfig();
    MoveProposalOperator mover = new MoveProposalOperator(ops, config);
    SplitmixRandom rng = new SplitmixRandom(999L);
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;

    // 1. Test REPLACE
    MoveProposalOperator.MoveProposal replaceProp = null;
    for (int t = 0; t < 30; t++) {
      replaceProp = mover.proposeReplace(origSkeleton, origSkeleton, rng, radius, reqDist);
      if (replaceProp.isFeasible()) {
        break;
      }
    }
    Assert.assertNotNull(replaceProp);
    Assert.assertTrue("REPLACE must produce feasible proposal", replaceProp.isFeasible());
    Assert.assertEquals("REPLACE", replaceProp.getOperator());
    assertGenerated(replaceProp);
    // Frozen final via must be identical
    int lastIdx = origSkeleton.getVias().size() - 1;
    Assert.assertEquals(origSkeleton.getVias().get(lastIdx).crosspoint.ilon,
      replaceProp.getMutatedSkeleton().getVias().get(lastIdx).crosspoint.ilon);

    // 2. Test 2-OPT
    // Ensure skeleton has at least 3 vias (2 non-final vias) for 2-OPT
    if (origSkeleton.getVias().size() >= 3) {
      MoveProposalOperator.MoveProposal optProp = mover.propose2Opt(origSkeleton);
      if (optProp.isFeasible()) {
        Assert.assertEquals("2-OPT", optProp.getOperator());
        // Verify final via is frozen (not reversed)
        Assert.assertEquals(origSkeleton.getVias().get(lastIdx).crosspoint.ilon,
          optProp.getMutatedSkeleton().getVias().get(lastIdx).crosspoint.ilon);
      }
    }

    // 3. Test INSERT
    MoveProposalOperator.MoveProposal insertProp = null;
    for (int t = 0; t < 30; t++) {
      insertProp = mover.proposeInsert(origSkeleton, rng, radius, reqDist);
      if (insertProp.isFeasible()) {
        break;
      }
    }
    if (insertProp != null && insertProp.isFeasible()) {
      Assert.assertEquals("INSERT", insertProp.getOperator());
      assertGenerated(insertProp);
      // Inserted skeleton must have m + 1 vias
      Assert.assertEquals(origSkeleton.getVias().size() + 1, insertProp.getMutatedSkeleton().getVias().size());
      // Final via must still match original final via (closing leg NEVER touched!)
      int newLastIdx = insertProp.getMutatedSkeleton().getVias().size() - 1;
      Assert.assertEquals(origSkeleton.getVias().get(lastIdx).crosspoint.ilon,
        insertProp.getMutatedSkeleton().getVias().get(newLastIdx).crosspoint.ilon);
    }

    // 4. Test unified propose
    boolean foundFeasible = false;
    for (int t = 0; t < 20; t++) {
      MoveProposalOperator.MoveProposal p = mover.propose(origSkeleton, origSkeleton, rng, radius, reqDist);
      if (p.isFeasible()) {
        foundFeasible = true;
        Assert.assertNotNull(p.getOperator());
        Assert.assertNotNull(p.getMutatedSkeleton());
        break;
      }
    }
    Assert.assertTrue("Unified propose must produce feasible candidate", foundFeasible);
  }

  @Test
  public void testChainedMutationsAndAnchorPreservation() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton origSkeleton = new RefineSkeleton(
      mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    RefineConfig config = new RefineConfig();
    MoveProposalOperator mover = new MoveProposalOperator(ops, config);
    SplitmixRandom rng = new SplitmixRandom(42L);
    double radius = ops.roundTripSearchRadius();
    double reqDist = 2 * Math.PI * radius;
    int origViaCount = origSkeleton.getVias().size();

    // 1. Direct unit verification of findOriginalAnchor:
    // a) Original via matches by name even if index shifted
    MatchedWaypoint cur0 = origSkeleton.getVias().get(0);
    MatchedWaypoint anchor0 = MoveProposalOperator.findOriginalAnchor(cur0, origSkeleton.getVias());
    Assert.assertNotNull(anchor0);
    Assert.assertEquals(cur0.crosspoint.ilon, anchor0.crosspoint.ilon);
    Assert.assertEquals(cur0.crosspoint.ilat, anchor0.crosspoint.ilat);

    // b) Inserted via encodes birth coordinates in name
    MatchedWaypoint insertedWp = new MatchedWaypoint();
    insertedWp.name = "refine_insert_1_188709501_140007339";
    insertedWp.crosspoint = new OsmNode(188709500, 140007340);
    MatchedWaypoint insertAnchor = MoveProposalOperator.findOriginalAnchor(insertedWp, origSkeleton.getVias());
    Assert.assertNotNull(insertAnchor);
    Assert.assertEquals(188709501, insertAnchor.crosspoint.ilon);
    Assert.assertEquals(140007339, insertAnchor.crosspoint.ilat);

    // c) Unmatched via falls back to curVia without exception
    MatchedWaypoint unknownWp = new MatchedWaypoint();
    unknownWp.name = "custom_unknown";
    unknownWp.crosspoint = new OsmNode(188700000, 140000000);
    MatchedWaypoint fallbackAnchor = MoveProposalOperator.findOriginalAnchor(unknownWp, origSkeleton.getVias());
    Assert.assertEquals(unknownWp, fallbackAnchor);

    // 2. Chained operators: INSERT -> 2-OPT
    MoveProposalOperator.MoveProposal insertProp = null;
    for (int t = 0; t < 30; t++) {
      insertProp = mover.proposeInsert(origSkeleton, rng, radius, reqDist);
      if (insertProp.isFeasible()) {
        break;
      }
    }
    Assert.assertNotNull("INSERT should succeed", insertProp);
    Assert.assertTrue("INSERT must be feasible", insertProp.isFeasible());
    RefineSkeleton skelAfterInsert = insertProp.getMutatedSkeleton();
    Assert.assertEquals(origViaCount + 1, skelAfterInsert.getVias().size());

    MoveProposalOperator.MoveProposal optProp = null;
    for (int t = 0; t < 30; t++) {
      optProp = mover.propose2Opt(skelAfterInsert, rng);
      if (optProp.isFeasible()) {
        break;
      }
    }
    Assert.assertNotNull("2-OPT should succeed", optProp);
    Assert.assertTrue("2-OPT must be feasible", optProp.isFeasible());
    RefineSkeleton skelAfter2Opt = optProp.getMutatedSkeleton();
    Assert.assertEquals(origViaCount + 1, skelAfter2Opt.getVias().size());

    // Verify all vias in skelAfter2Opt resolve anchors and displacement is bounded
    double maxDisp = config.maxDisplacementFraction * radius;
    for (MatchedWaypoint v : skelAfter2Opt.getVias()) {
      MatchedWaypoint anc = MoveProposalOperator.findOriginalAnchor(v, origSkeleton.getVias());
      Assert.assertNotNull(anc);
      double d = CheapRuler.distance(anc.crosspoint.ilon, anc.crosspoint.ilat,
        v.crosspoint.ilon, v.crosspoint.ilat);
      Assert.assertTrue("Anchor displacement must be within maxDisp: " + d + " <= " + maxDisp,
        d <= maxDisp + 1.0);
    }

    // 3. Chained mutations under LOCAL search
    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "test-chained-local");
    long deadline = System.currentTimeMillis() + 10000L;
    RefineInitResult init = RefineInitializer.initialize(
      ops, evaluator, origSkeleton,
      new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops),
      res.getTrack(), radius, "trekking", 90, reqDist, deadline);
    Assert.assertTrue("Initialization should succeed", init.isSuccess());

    RefineConfig searchCfg = new RefineConfig();
    searchCfg.mode = RefineConfig.Mode.LOCAL;
    searchCfg.evaluations = 8;
    searchCfg.topKFinalists = 2;

    RefineSearch search = new RefineSearch(
      ops, evaluator, init.getLegCache(), searchCfg, origSkeleton,
      init.getRawLegs(), LoopCostOracle.evaluate(ops, init.getRawLegs(), origSkeleton.getWaypoints(), deadline),
      mover, radius, reqDist, 42, deadline);

    RefineDiagnostics diag = new RefineDiagnostics();
    RefineSearch.SearchResult searchResult = search.search(diag);
    Assert.assertNotNull(searchResult);

    // Verify ADR-0002 invariant on all finalists: frozen final via preserved
    MatchedWaypoint origFinalVia = origSkeleton.getVias().get(origViaCount - 1);
    for (RefineSearch.SearchCandidate finalist : searchResult.getFinalists()) {
      List<MatchedWaypoint> fVias = finalist.getSkeleton().getVias();
      MatchedWaypoint finVia = fVias.get(fVias.size() - 1);
      Assert.assertEquals("Frozen final via ilon preserved in finalist",
        origFinalVia.crosspoint.ilon, finVia.crosspoint.ilon);
      Assert.assertEquals("Frozen final via ilat preserved in finalist",
        origFinalVia.crosspoint.ilat, finVia.crosspoint.ilat);
    }
  }
  private static void assertGenerated(MoveProposalOperator.MoveProposal proposal) {
    MatchedWaypoint wp = proposal.getSnappedWaypoint();
    Assert.assertTrue("Optimizer vias must remain generated", wp.generated);
    Assert.assertTrue(WaypointSnapper.isGeneratedRoundTripWaypoint(wp));
    Assert.assertEquals(wp.crosspoint.getIdFromPos(), wp.waypoint.getIdFromPos());
    Assert.assertEquals(MatchedWaypoint.WAYPOINT_TYPE_SHAPING, wp.wpttype);
  }

}
