package btools.router.roundtrip;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
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

public class GeometryFidelityTest {

  @Test
  public void testMidEdgeVias() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull("Result should not be null", res);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    Assert.assertTrue("Should have at least 3 waypoints", mwps.size() >= 3);

    MatchedWaypoint startWp = mwps.get(0);
    MatchedWaypoint viaWp = mwps.get(1);
    MatchedWaypoint nextWp = mwps.get(2);

    RoundTripEngineOps ops = re.roundTripOps();

    OsmTrack leg1 = ops.findTrackTimed("midedge-leg1", startWp, viaWp, null, 5000L);
    Assert.assertNotNull("Leg 1 must route", leg1);
    int cost1 = LoopCostOracle.priceCost(ops, leg1, startWp, viaWp);
    Assert.assertEquals("Leg 1 cost matches", leg1.cost, cost1);

    OsmTrack leg2 = ops.findTrackTimed("midedge-leg2", viaWp, nextWp, null, 5000L);
    Assert.assertNotNull("Leg 2 must route", leg2);
    int cost2 = LoopCostOracle.priceCost(ops, leg2, viaWp, nextWp);
    Assert.assertEquals("Leg 2 cost matches", leg2.cost, cost2);

    List<OsmTrack> twoLegs = new ArrayList<>();
    twoLegs.add(leg1);
    twoLegs.add(leg2);
    List<MatchedWaypoint> twoWps = new ArrayList<>();
    twoWps.add(startWp);
    twoWps.add(viaWp);
    twoWps.add(nextWp);

    int multiCost = LoopCostOracle.priceCost(ops, twoLegs, twoWps);
    Assert.assertTrue("Continuous price across mid-edge via must be positive", multiCost > 0);

    // Retrack for detail
    OsmTrack leg1Copy = copyTrack(leg1);
    OsmTrack detLeg1 = ops.retrackForDetail(leg1Copy, startWp, viaWp, null);
    Assert.assertNotNull("Detailed leg 1 must not be null", detLeg1);
    Assert.assertEquals("Detailed leg start pos matches crosspoint",
      startWp.crosspoint.getIdFromPos(), detLeg1.nodes.get(0).getIdFromPos());
    Assert.assertEquals("Detailed leg end pos matches crosspoint",
      viaWp.crosspoint.getIdFromPos(), detLeg1.nodes.get(detLeg1.nodes.size() - 1).getIdFromPos());
  }

  @Test
  public void testSameEdgeEndpoints() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    MatchedWaypoint baseWp = res.getMatchedWaypoints().get(1);

    OsmNode n1 = baseWp.node1;
    OsmNode n2 = baseWp.node2;

    MatchedWaypoint fromWp = new MatchedWaypoint();
    fromWp.node1 = n1;
    fromWp.node2 = n2;
    fromWp.crosspoint = new OsmNode();
    fromWp.crosspoint.ilon = (int) (0.75 * n1.ilon + 0.25 * n2.ilon);
    fromWp.crosspoint.ilat = (int) (0.75 * n1.ilat + 0.25 * n2.ilat);
    fromWp.waypoint = fromWp.crosspoint;

    MatchedWaypoint toWp = new MatchedWaypoint();
    toWp.node1 = n1;
    toWp.node2 = n2;
    toWp.crosspoint = new OsmNode();
    toWp.crosspoint.ilon = (int) (0.25 * n1.ilon + 0.75 * n2.ilon);
    toWp.crosspoint.ilat = (int) (0.25 * n1.ilat + 0.75 * n2.ilat);
    toWp.waypoint = toWp.crosspoint;

    RoundTripEngineOps ops = re.roundTripOps();
    OsmTrack sameEdgeTrack = ops.findTrackTimed("same-edge-leg", fromWp, toWp, null, 5000L);
    Assert.assertNotNull("Same-edge leg routing should find track", sameEdgeTrack);
    Assert.assertTrue("Same-edge leg cost should be positive", sameEdgeTrack.cost > 0);

    int walkedCost = LoopCostOracle.priceCost(ops, sameEdgeTrack, fromWp, toWp);
    Assert.assertEquals("Single same-edge segment price should equal routed cost",
      sameEdgeTrack.cost, walkedCost);
  }

  @Test
  public void testOppositeArrivalDeparture() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);
    MatchedWaypoint startWp = res.getMatchedWaypoints().get(0);
    MatchedWaypoint spurWp = res.getMatchedWaypoints().get(1);

    RoundTripEngineOps ops = re.roundTripOps();

    OsmTrack outwardLeg = ops.findTrackTimed("outward", startWp, spurWp, null, 5000L);
    Assert.assertNotNull(outwardLeg);
    OsmTrack returnLeg = ops.findTrackTimed("return", spurWp, startWp, null, 5000L);
    Assert.assertNotNull(returnLeg);

    List<OsmTrack> legs = new ArrayList<>();
    legs.add(outwardLeg);
    legs.add(returnLeg);
    List<MatchedWaypoint> mwps = new ArrayList<>();
    mwps.add(startWp);
    mwps.add(spurWp);
    mwps.add(startWp);

    int hairpinCost = LoopCostOracle.priceCost(ops, legs, mwps);
    Assert.assertTrue("180 degree turnaround at via should produce valid cost", hairpinCost > 0);
  }

  @Test
  public void testCurvedEdges() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 2000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();

    boolean foundCurvedLeg = false;
    for (int i = 0; i < mwps.size() - 1; i++) {
      MatchedWaypoint from = mwps.get(i);
      MatchedWaypoint to = mwps.get(i + 1);
      OsmTrack raw = ops.findTrackTimed("raw-" + i, from, to, null, 5000L);
      if (raw == null) {
        continue;
      }
      OsmTrack rawCopy = copyTrack(raw);
      OsmTrack det = ops.retrackForDetail(rawCopy, from, to, null);
      if (det != null && det.nodes.size() > raw.nodes.size()) {
        foundCurvedLeg = true;
        int rawPrice = LoopCostOracle.priceCost(ops, raw, from, to);
        int detPrice = LoopCostOracle.priceCost(ops, det, from, to);
        Assert.assertEquals("Raw leg price equals routed cost", raw.cost, rawPrice);
        Assert.assertEquals("Detailed curved leg price matches raw within 1", raw.cost, detPrice, 1.0);
        break;
      }
    }
    Assert.assertTrue("At least one leg should contain curved edge with transfer nodes", foundCurvedLeg);
  }

  @Test
  public void testRemovedViaSpurs() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1500, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);

    OsmTrack track = new OsmTrack();
    OsmPathElement nodeA = OsmPathElement.create(188720000, 140000000, (short) 0, null);
    OsmPathElement nodeB = OsmPathElement.create(188730000, 140010000, (short) 0, null);
    OsmPathElement nodeC = OsmPathElement.create(188740000, 140020000, (short) 0, null);
    OsmPathElement nodeV = OsmPathElement.create(188750000, 140030000, (short) 0, null);
    OsmPathElement nodeC2 = OsmPathElement.create(188740000, 140020000, (short) 0, null);
    OsmPathElement nodeB2 = OsmPathElement.create(188730000, 140010000, (short) 0, null);
    OsmPathElement nodeD = OsmPathElement.create(188760000, 140040000, (short) 0, null);
    OsmPathElement nodeE = OsmPathElement.create(188720000, 140000000, (short) 0, null);

    track.nodes.add(nodeA);
    track.nodes.add(nodeB);
    track.nodes.add(nodeC);
    track.nodes.add(nodeV);
    track.nodes.add(nodeC2);
    track.nodes.add(nodeB2);
    track.nodes.add(nodeD);
    track.nodes.add(nodeE);

    List<MatchedWaypoint> wpts = new ArrayList<>();
    MatchedWaypoint startWp = createMatchedWaypoint("start", 188720000, 140000000);
    startWp.indexInTrack = 0;
    wpts.add(startWp);

    MatchedWaypoint spurWp = createMatchedWaypoint("spurVia", 188750000, 140030000);
    spurWp.indexInTrack = 3;
    wpts.add(spurWp);

    MatchedWaypoint endWp = createMatchedWaypoint("end", 188720000, 140000000);
    endWp.indexInTrack = 7;
    wpts.add(endWp);

    cleanup.removeBackAndForthSegments(track, wpts);

    Assert.assertEquals("Spur B-C-V-C-B should be removed leaving 4 nodes", 4, track.nodes.size());
    Assert.assertSame(nodeA, track.nodes.get(0));
    Assert.assertSame(nodeB, track.nodes.get(1));
    Assert.assertSame(nodeD, track.nodes.get(2));
    Assert.assertSame(nodeE, track.nodes.get(3));

    Assert.assertEquals("Via indexInTrack should be adjusted to surviving branch node", 1, spurWp.indexInTrack);
    Assert.assertEquals("End indexInTrack should be adjusted", 3, endWp.indexInTrack);
  }

  @Test
  public void testSkeletonRecoveryOnGreedyAndAutoWinners() {
    String[] profiles = {"trekking", "gravel", "fastbike"};
    RoundTripAlgorithm[] algs = {RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.AUTO};
    int[] directions = {0, 90, 180, 270};

    int testedLoops = 0;

    for (String profile : profiles) {
      for (RoundTripAlgorithm alg : algs) {
        for (int direction : directions) {
          RoutingEngine re = RoundTripFixture.engine(profile, direction, 1500, rc -> {
            rc.roundTripAlgorithm = alg;
            rc.roundTripStrictQuality = false;
          });
          RoundTripResult res = re.getLastRoundTripResult();
          if (res == null || res.getTrack() == null) {
            continue;
          }
          OsmTrack track = res.getTrack();
          List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
          if (mwps == null || mwps.size() < 2) {
            continue;
          }

          testedLoops++;

          Assert.assertEquals("First waypoint must map to index 0", 0, mwps.get(0).indexInTrack);
          Assert.assertEquals("Last waypoint must map to last node",
            track.nodes.size() - 1, mwps.get(mwps.size() - 1).indexInTrack);

          int prevIdx = -1;
          for (int i = 0; i < mwps.size(); i++) {
            MatchedWaypoint mwp = mwps.get(i);
            Assert.assertTrue("Waypoint index must be >= 0", mwp.indexInTrack >= 0);
            Assert.assertTrue("Waypoint index must be < track.nodes.size()", mwp.indexInTrack < track.nodes.size());
            Assert.assertTrue("Waypoint indices must be monotonically non-decreasing", mwp.indexInTrack >= prevIdx);
            prevIdx = mwp.indexInTrack;

            // Nearest node check
            OsmNode target = mwp.crosspoint != null ? mwp.crosspoint : mwp.waypoint;
            if (target != null && i > 0 && i < mwps.size() - 1) {
              int dist = track.nodes.get(mwp.indexInTrack).calcDistance(target);
              Assert.assertTrue("Waypoint " + i + " distance to its mapped node should be <= search radius (was " + dist + "m)",
                dist <= 1500);
            }
          }
        }
      }
    }
    Assert.assertTrue("Must have tested multiple GREEDY and AUTO loops", testedLoops >= 8);
  }

  @Test
  public void testGenerateGeometryFidelityReport() throws IOException {
    String[] profiles = {"trekking", "gravel", "fastbike"};
    int[] directions = {0, 45, 90, 135, 180, 225, 270, 315};
    int[] radii = {1000, 1500, 2000};

    List<ReportRow> rows = new ArrayList<>();

    for (String profile : profiles) {
      for (int radius : radii) {
        for (int direction : directions) {
          RoutingEngine re = RoundTripFixture.engine(profile, direction, radius, rc -> {
            rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
            rc.roundTripStrictQuality = false;
          });
          RoundTripResult res = re.getLastRoundTripResult();
          if (res == null || res.getTrack() == null) {
            continue;
          }
          OsmTrack detailedTrack = res.getTrack();
          List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
          if (mwps == null || mwps.size() < 2) {
            continue;
          }

          RoundTripEngineOps ops = re.roundTripOps();
          List<OsmTrack> rawLegs = new ArrayList<>();
          boolean routingFailed = false;
          for (int l = 0; l < mwps.size() - 1; l++) {
            OsmTrack leg = ops.findTrackTimed("report-leg-" + l, mwps.get(l), mwps.get(l + 1), null, 5000L);
            if (leg == null) {
              routingFailed = true;
              break;
            }
            rawLegs.add(leg);
          }
          if (routingFailed || rawLegs.isEmpty()) {
            continue;
          }

          OsmTrack rawConcat = new OsmTrack();
          for (int l = 0; l < rawLegs.size(); l++) {
            OsmTrack leg = rawLegs.get(l);
            int startIdx = (l == 0) ? 0 : 1;
            for (int n = startIdx; n < leg.nodes.size(); n++) {
              rawConcat.nodes.add(leg.nodes.get(n));
            }
          }

          int rawDist = 0;
          for (int n = 0; n < rawConcat.nodes.size() - 1; n++) {
            rawDist += rawConcat.nodes.get(n).calcDistance(rawConcat.nodes.get(n + 1));
          }
          int detDist = (int) detailedTrack.distance;

          int rawCrossings = LoopQualityMetrics.detectCrossings(rawConcat.nodes)[0];
          int detCrossings = LoopQualityMetrics.detectCrossings(detailedTrack.nodes)[0];

          int[] rawReuse = LoopQualityMetrics.reuseStemSplit(rawConcat.nodes);
          int[] detReuse = LoopQualityMetrics.reuseStemSplit(detailedTrack.nodes);

          int rawScatter = rawReuse[1];
          int detScatter = detReuse[1];

          ReportRow row = new ReportRow();
          row.profile = profile;
          row.direction = direction;
          row.radius = radius;
          row.rawNodes = rawConcat.nodes.size();
          row.detNodes = detailedTrack.nodes.size();
          row.rawDistance = rawDist;
          row.detDistance = detDist;
          row.rawCrossings = rawCrossings;
          row.detCrossings = detCrossings;
          row.rawScatter = rawScatter;
          row.detScatter = detScatter;
          rows.add(row);
        }
      }
    }

    Assert.assertTrue("Should collect >= 30 loops for fidelity report", rows.size() >= 30);

    File reportFile = new File(RoundTripFixture.projectDir(), "docs/m0_2_geometry_fidelity_report.md");
    StringBuilder sb = new StringBuilder();
    sb.append("# M0.2 Geometry Fidelity Report\n\n");
    sb.append("This report compares the raw search representation (concatenated junction-level legs) ");
    sb.append("against the detailed shipped track (post-cleanup, detailed transfer nodes) across ")
      .append(rows.size()).append(" Dreieich fixture loops.\n\n");

    sb.append("## Raw vs Detailed Comparison Table\n\n");
    sb.append("| Profile | Dir (°) | Radius (m) | Raw Nodes | Det Nodes | Raw Dist (m) | Det Dist (m) | Δ Dist (%) | Raw Cross | Det Cross | Raw Scatter (m) | Det Scatter (m) |\n");
    sb.append("|---|---|---|---|---|---|---|---|---|---|---|---|\n");

    double sumDeltaDistPct = 0;
    int crossingMatches = 0;
    int scatterMatches = 0;

    for (ReportRow r : rows) {
      double deltaDistPct = 100.0 * (r.detDistance - r.rawDistance) / (double) r.rawDistance;
      sumDeltaDistPct += deltaDistPct;
      if (r.rawCrossings == r.detCrossings) {
        crossingMatches++;
      }
      if (r.rawScatter == r.detScatter) {
        scatterMatches++;
      }

      sb.append(String.format("| %s | %d | %d | %d | %d | %d | %d | %+.2f%% | %d | %d | %d | %d |\n",
        r.profile, r.direction, r.radius, r.rawNodes, r.detNodes,
        r.rawDistance, r.detDistance, deltaDistPct,
        r.rawCrossings, r.detCrossings, r.rawScatter, r.detScatter));
    }

    double avgDeltaDistPct = sumDeltaDistPct / rows.size();
    sb.append("\n## Key Observations\n\n");
    sb.append(String.format("1. **Distance Fidelity:** Average distance delta between raw and detailed is **%+.2f%%**. Transfer nodes along curved ways provide accurate arc distances compared to junction chords, while micro-detour and spur removal trim duplicate distance.\n", avgDeltaDistPct));
    sb.append(String.format("2. **Self-Crossing Agreement:** Raw and detailed self-crossings matched in **%d / %d (%.1f%%)** of loops. Differences occur primarily when cleanup removes micro-detours or loops around vias.\n",
      crossingMatches, rows.size(), 100.0 * crossingMatches / rows.size()));
    sb.append(String.format("3. **Scatter Reuse Agreement:** Raw and detailed scatter reuse matched in **%d / %d (%.1f%%)** of loops. Post-routing cleanup (`removeBackAndForthSegments`, `repairViaPinnedBulges`) cleans spurs at vias which eliminates spur-induced reuse.\n",
      scatterMatches, rows.size(), 100.0 * scatterMatches / rows.size()));
    sb.append("4. **Implication for Search vs Finalization:** Because raw legs have higher spur/micro-detour artifacts than cleaned final routes, evaluating candidates on raw legs acts as a conservative filter. Finalization cleans these artifacts before applying the strict ship predicate.\n");

    try (FileWriter fw = new FileWriter(reportFile)) {
      fw.write(sb.toString());
    }

    System.out.println("Generated M0.2 Geometry Fidelity Report at " + reportFile.getAbsolutePath());
  }

  private static final class ReportRow {
    String profile;
    int direction;
    int radius;
    int rawNodes;
    int detNodes;
    int rawDistance;
    int detDistance;
    int rawCrossings;
    int detCrossings;
    int rawScatter;
    int detScatter;
  }

  private static OsmTrack copyTrack(OsmTrack track) {
    OsmTrack copy = new OsmTrack();
    copy.cost = track.cost + 50000;
    copy.distance = track.distance;
    for (OsmPathElement pe : track.nodes) {
      copy.nodes.add(OsmPathElement.create(pe.getILon(), pe.getILat(), pe.getSElev(), null));
    }
    return copy;
  }

  private static MatchedWaypoint createMatchedWaypoint(String name, int ilon, int ilat) {
    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.name = name;
    mwp.waypoint = new OsmNode();
    mwp.waypoint.ilon = ilon;
    mwp.waypoint.ilat = ilat;
    mwp.crosspoint = mwp.waypoint;
    mwp.node1 = mwp.waypoint;
    mwp.node2 = mwp.waypoint;
    return mwp;
  }
}
