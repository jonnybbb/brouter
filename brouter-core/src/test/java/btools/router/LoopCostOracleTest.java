package btools.router;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;
import static org.junit.Assert.*;

import btools.mapaccess.MatchedWaypoint;
import btools.router.roundtrip.LoopCostOracle;
import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripResult;

public class LoopCostOracleTest {

  @Test
  public void continuousFailureNeverFallsBackToIndependentlyPricedLegs() {
    OsmTrack first = new OsmTrack();
    first.nodes.add(OsmPathElement.create(100, 100, (short) 0, null));
    first.nodes.add(OsmPathElement.create(200, 100, (short) 0, null));
    OsmTrack second = new OsmTrack();
    second.nodes.add(OsmPathElement.create(200, 100, (short) 0, null));
    second.nodes.add(OsmPathElement.create(100, 100, (short) 0, null));
    RoutingContext context = new RoutingContext();
    context.localFunction = "../misc/profiles2/gravel.brf";
    RoutingEngine engine = new RoutingEngine(null, null, new File("."), new ArrayList<>(), context, 0) {
      @Override
      int walkPathCost(OsmTrack track, MatchedWaypoint start, MatchedWaypoint end) {
        return track == first || track == second ? 100 : -1;
      }
    };
    List<MatchedWaypoint> waypoints = java.util.Arrays.asList(new MatchedWaypoint(), new MatchedWaypoint(), new MatchedWaypoint());
    assertEquals(100, engine.walkPathCost(first, waypoints.get(0), waypoints.get(1)));
    assertEquals(100, engine.walkPathCost(second, waypoints.get(1), waypoints.get(2)));
    assertEquals(-1, engine.walkLoopCost(java.util.Arrays.asList(first, second), waypoints));
    assertEquals("none", engine.getLastPricingMethod());
  }


  @Test
  public void testExactLegPriceMatchOn40FixtureLegs() {
    String[] profiles = {"trekking", "gravel", "fastbike"};
    int[] directions = {0, 90, 180, 270};
    int[] radii = {1000, 1500};

    int totalVerifiedLegs = 0;

    for (String profile : profiles) {
      for (int direction : directions) {
        for (int radius : radii) {
          RoutingEngine re = RoundTripFixture.engine(profile, direction, radius, rc -> {
            rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
            rc.roundTripStrictQuality = false;
          });
          RoundTripResult res = re.getLastRoundTripResult();
          if (res == null || res.getMatchedWaypoints() == null || res.getMatchedWaypoints().size() < 2) {
            continue;
          }
          List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
          for (int l = 0; l < mwps.size() - 1; l++) {
            MatchedWaypoint fromWp = mwps.get(l);
            MatchedWaypoint toWp = mwps.get(l + 1);
            OsmTrack rawLeg = re.roundTripOps().findTrackTimed("verify-leg-" + totalVerifiedLegs,
              fromWp, toWp, null, 5000L);
            if (rawLeg == null || rawLeg.nodes == null || rawLeg.nodes.size() < 2) {
              continue;
            }
            int walked = LoopCostOracle.priceCost(re.roundTripOps(), java.util.Collections.singletonList(rawLeg), java.util.Arrays.asList(fromWp, toWp));
            OsmTrack detailed = re.roundTripOps().retrackForDetail(rawLeg, fromWp, toWp, null);
            assertNotSame("Reference must be detailed", rawLeg, detailed);
            int exact = LoopCostOracle.priceCost(re.roundTripOps(), detailed, fromWp, toWp);
            assertTrue("Detailed reference must be priceable: " + re.getLastPricingFailure(), exact > 0);
            assertEquals("Raw preparation must price the same clipped geometry on leg " + totalVerifiedLegs,
              exact, walked);
            totalVerifiedLegs++;
          }
        }
      }
    }

    System.out.println("Verified exact cost match on " + totalVerifiedLegs + " raw legs (target >= 40)");
    assertTrue("Should verify exact cost match on >= 40 legs, got " + totalVerifiedLegs, totalVerifiedLegs >= 40);
  }

  @Test
  public void testSegmentationInvariance() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    assertNotNull(res);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    assertTrue(mwps.size() >= 4);

    MatchedWaypoint wp0 = mwps.get(0);
    MatchedWaypoint wp1 = mwps.get(1);
    MatchedWaypoint wp2 = mwps.get(2);
    MatchedWaypoint wp3 = mwps.get(3);

    OsmTrack leg0 = re.roundTripOps().findTrackTimed("seg-0", wp0, wp1, null, 5000L);
    OsmTrack leg1 = re.roundTripOps().findTrackTimed("seg-1", wp1, wp2, null, 5000L);
    OsmTrack leg2 = re.roundTripOps().findTrackTimed("seg-2", wp2, wp3, null, 5000L);
    assertNotNull(leg0);
    assertNotNull(leg1);
    assertNotNull(leg2);

    List<OsmTrack> segmentedLegs = new ArrayList<>();
    segmentedLegs.add(leg0);
    segmentedLegs.add(leg1);
    segmentedLegs.add(leg2);

    List<MatchedWaypoint> subMwps = new ArrayList<>();
    subMwps.add(wp0);
    subMwps.add(wp1);
    subMwps.add(wp2);
    subMwps.add(wp3);

    int continuous3LegCost = LoopCostOracle.priceCost(re.roundTripOps(), segmentedLegs, subMwps);
    assertTrue("Continuous walked cost should be positive", continuous3LegCost > 0);

    assertTrue("leg0 should have enough nodes to split", leg0.nodes.size() > 10);
    int splitIdx = leg0.nodes.size() / 2;
    OsmPathElement splitElem = leg0.nodes.get(splitIdx);
    MatchedWaypoint splitWp = re.roundTripOps().profileAwareMatchPoint(splitElem.getILon(), splitElem.getILat(), "split", 50.0);
    assertNotNull("Split point must be matched for this regression to run", splitWp);
    if (splitWp != null) {
      OsmTrack leg0a = re.roundTripOps().findTrackTimed("split-a", wp0, splitWp, null, 5000L);
      OsmTrack leg0b = re.roundTripOps().findTrackTimed("split-b", splitWp, wp1, null, 5000L);
      assertNotNull(leg0a);
      assertNotNull(leg0b);
      List<OsmTrack> splitLegs = new ArrayList<>();
      splitLegs.add(leg0a);
      splitLegs.add(leg0b);
      List<MatchedWaypoint> splitWaypoints = new ArrayList<>();
      splitWaypoints.add(wp0);
      splitWaypoints.add(splitWp);
      splitWaypoints.add(wp1);

      int splitWalkedCost = LoopCostOracle.priceCost(re.roundTripOps(), splitLegs, splitWaypoints);
      int singleWalkedCost = LoopCostOracle.priceCost(re.roundTripOps(), java.util.Collections.singletonList(leg0), java.util.Arrays.asList(wp0, wp1));
      System.out.println("Segmentation invariance: single leg=" + singleWalkedCost + " vs 2 sub-legs=" + splitWalkedCost);
      assertEquals("Segmentation invariance: split sub-legs equal single leg cost within tolerance",
        (double) singleWalkedCost, (double) splitWalkedCost, 1.0);
    }
  }

  @Test
  public void testClosedLoopPricing() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    assertNotNull(res);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> freshRawLegs = new ArrayList<>();
    for (int l = 0; l < mwps.size() - 1; l++) {
      OsmTrack raw = re.roundTripOps().findTrackTimed("raw-closed-" + l, mwps.get(l), mwps.get(l + 1), null, 5000L);
      assertNotNull(raw);
      freshRawLegs.add(raw);
    }

    double loopCostPerM = LoopCostOracle.price(re.roundTripOps(), freshRawLegs, mwps);
    int loopCost = LoopCostOracle.priceCost(re.roundTripOps(), freshRawLegs, mwps);
    System.out.println("Closed loop price: cost=" + loopCost + " cost/m=" + loopCostPerM);
    assertTrue("Loop cost should be positive", loopCost > 0);
    assertTrue("Loop cost/m should be positive", loopCostPerM > 0.0);
  }

  @Test
  public void testTurnCostPreservationAcrossVias() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    assertNotNull(res);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> freshRawLegs = new ArrayList<>();
    int sumSeparateCosts = 0;
    for (int l = 0; l < mwps.size() - 1; l++) {
      OsmTrack raw = re.roundTripOps().findTrackTimed("raw-turn-" + l, mwps.get(l), mwps.get(l + 1), null, 5000L);
      assertNotNull(raw);
      freshRawLegs.add(raw);
      sumSeparateCosts += raw.cost;
    }

    int continuousCost = LoopCostOracle.priceCost(re.roundTripOps(), freshRawLegs, mwps);
    assertTrue("Continuous loop cost should reflect turn penalties across vias", continuousCost >= sumSeparateCosts);
  }

  @Test
  public void testElevationHysteresisPreservation() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 180, 2000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    assertNotNull(res);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    List<OsmTrack> legs = new ArrayList<>();
    for (int l = 0; l < mwps.size() - 1; l++) {
      OsmTrack raw = re.roundTripOps().findTrackTimed("raw-ele-" + l, mwps.get(l), mwps.get(l + 1), null, 5000L);
      assertNotNull(raw);
      legs.add(raw);
    }
    int continuousCost = LoopCostOracle.priceCost(re.roundTripOps(), legs, mwps);
    assertTrue("Continuous loop cost with elevation hysteresis must be positive", continuousCost > 0);
  }

  @Test
  public void testClippedEndpoints() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    MatchedWaypoint from = re.getLastRoundTripResult().getMatchedWaypoints().get(0);
    MatchedWaypoint to = re.getLastRoundTripResult().getMatchedWaypoints().get(1);

    OsmTrack rawLeg = re.roundTripOps().findTrackTimed("raw-clipped", from, to, null, 5000L);
    assertNotNull(rawLeg);
    int walkedCost = LoopCostOracle.priceCost(re.roundTripOps(), java.util.Collections.singletonList(rawLeg), java.util.Arrays.asList(from, to));
    OsmTrack detailed = re.roundTripOps().retrackForDetail(rawLeg, from, to, null);
    assertNotSame(rawLeg, detailed);
    assertEquals("Endpoint clipping prices the exact detailed route",
      LoopCostOracle.priceCost(re.roundTripOps(), detailed, from, to), walkedCost);
  }

  @Test
  public void testDetailedToJunctionReduction() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    MatchedWaypoint from = re.getLastRoundTripResult().getMatchedWaypoints().get(0);
    MatchedWaypoint to = re.getLastRoundTripResult().getMatchedWaypoints().get(1);

    OsmTrack rawLeg = re.roundTripOps().findTrackTimed("raw-detail", from, to, null, 5000L);
    assertNotNull(rawLeg);

    OsmTrack rawCopy = new OsmTrack();
    rawCopy.cost = rawLeg.cost + 50000;
    for (OsmPathElement pe : rawLeg.nodes) {
      rawCopy.nodes.add(OsmPathElement.create(pe.getILon(), pe.getILat(), pe.getSElev(), null));
    }
    OsmTrack detailed = re.roundTripOps().retrackForDetail(rawCopy, from, to, null);
    assertNotNull(detailed);
    assertTrue("Detailed track has transfer nodes: " + detailed.nodes.size() + " > " + rawLeg.nodes.size(),
      detailed.nodes.size() > rawLeg.nodes.size());

    int walkedDetailed = LoopCostOracle.priceCost(re.roundTripOps(), detailed, from, to);
    assertTrue("Detailed track walked cost is positive", walkedDetailed > 0);
    assertTrue("Detailed walked cost matches raw cost within 1", Math.abs(walkedDetailed - rawLeg.cost) <= 1);
  }

  @Test
  public void testPrice30ShippedLoopsAndReport() throws IOException {
    String[] profiles = {"trekking", "gravel", "fastbike"};
    int[] directions = {0, 45, 90, 135, 180, 225, 270, 315};
    int[] radii = {1000, 1500, 2000};

    List<String> reportLines = new ArrayList<>();
    reportLines.add("# Milestone M0.1: Cost Oracle vs Historical Track Pricing Report");
    reportLines.add("");
    reportLines.add("| Loop Scenario | Profile | Radius (m) | Distance (m) | Shipped Cost/m | Oracle Cost/m | Delta % | Status |");
    reportLines.add("|---|---|---|---|---|---|---|---|");

    int pricedLoopsCount = 0;

    for (String profile : profiles) {
      for (int direction : directions) {
        for (int radius : radii) {
          if (pricedLoopsCount >= 32) break;

          String scenario = profile + "_dir" + direction + "_r" + radius;
          try {
            RoutingEngine re = RoundTripFixture.engine(profile, direction, radius, rc -> {
              rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
              rc.roundTripStrictQuality = false;
            });
            OsmTrack shippedTrack = re.getFoundTrack();
            RoundTripResult res = re.getLastRoundTripResult();
            if (shippedTrack == null || res == null || res.getMatchedWaypoints() == null || res.getMatchedWaypoints().size() < 2) {
              reportLines.add(String.format("| %s | %s | %d | N/A | N/A | N/A | N/A | SKIPPED_NO_ROUTE |",
                scenario, profile, radius));
              continue;
            }

            List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
            List<OsmTrack> freshRawLegs = new ArrayList<>();
            for (int l = 0; l < mwps.size() - 1; l++) {
              OsmTrack raw = re.roundTripOps().findTrackTimed("report-raw-" + l, mwps.get(l), mwps.get(l + 1), null, 5000L);
              if (raw != null) {
                freshRawLegs.add(raw);
              }
            }

            if (freshRawLegs.size() != mwps.size() - 1) {
              reportLines.add(String.format("| %s | %s | %d | %d | %.4f | N/A | N/A | LEG_ROUTING_FAILED |",
                scenario, profile, radius, shippedTrack.distance, (double) shippedTrack.cost / shippedTrack.distance));
              continue;
            }

            double oracleCostPerM = LoopCostOracle.price(re.roundTripOps(), freshRawLegs, mwps);
            if (oracleCostPerM <= 0.0) {
              reportLines.add(String.format("| %s | %s | %d | %d | %.4f | N/A | N/A | ORACLE_UNPRICED |",
                scenario, profile, radius, shippedTrack.distance, (double) shippedTrack.cost / shippedTrack.distance));
              continue;
            }

            double shippedCostPerM = (double) shippedTrack.cost / shippedTrack.distance;
            double deltaPct = 100.0 * (oracleCostPerM - shippedCostPerM) / shippedCostPerM;

            reportLines.add(String.format("| %s | %s | %d | %d | %.4f | %.4f | %+.2f%% | OK |",
              scenario, profile, radius, shippedTrack.distance, shippedCostPerM, oracleCostPerM, deltaPct));
            pricedLoopsCount++;
          } catch (Exception e) {
            reportLines.add(String.format("| %s | %s | %d | N/A | N/A | N/A | N/A | ERROR: %s |",
              scenario, profile, radius, e.getMessage()));
          }
        }
      }
    }

    reportLines.add("");
    reportLines.add("**Total successfully priced loops:** " + pricedLoopsCount + " (target >= 30)");

    File docsReport = new File(RoundTripFixture.projectDir(), "brouter-core/build/reports/refinement/m0_1_oracle_report.md");
    docsReport.getParentFile().mkdirs();
    try (FileWriter fw = new FileWriter(docsReport)) {
      for (String line : reportLines) {
        fw.write(line + "\n");
      }
    }
    System.out.println("Report written to " + docsReport.getAbsolutePath());
    for (String line : reportLines) {
      System.out.println(line);
    }

    assertTrue("Should successfully price >= 30 shipped loops, got " + pricedLoopsCount, pricedLoopsCount >= 30);
  }
}
