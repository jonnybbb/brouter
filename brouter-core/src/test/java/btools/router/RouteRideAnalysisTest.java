package btools.router;

import java.util.Map;

import org.junit.Test;

import btools.router.roundtrip.RoundTripAlgorithm;

import static org.junit.Assert.*;

public class RouteRideAnalysisTest {
  private static OsmTrack track(String... descriptions) {
    OsmTrack track = new OsmTrack();
    OsmPathElement previous = OsmPathElement.create(188000000, 140000000, (short) 400, null);
    track.nodes.add(previous);
    for (int i = 0; i < descriptions.length; i++) {
      OsmPathElement point = OsmPathElement.create(188000000 + (i + 1) * 1000, 140000000, (short) 400, previous);
      point.message = new MessageData();
      point.message.wayKeyValues = descriptions[i];
      point.message.recordSegment(previous.getIdFromPos(), point.getIdFromPos(), 10);
      track.nodes.add(point);
      previous = point;
    }
    return track;
  }

  @Test
  public void totalsUseSurvivingGeometryAndUnknownBreaksContinuousRuns() {
    OsmTrack track = track("surface=compacted estimated_traffic_class=1", "surface=compacted estimated_traffic_class=1",
      null, "surface=compacted estimated_traffic_class=5", "surface=asphalt smoothness=bad");
    track.distance = 99999;
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 400);
    double length = a.value("distance_m") / 5;
    assertEquals(3 * length, a.value("surface_firm_unpaved_m"), 0.1);
    assertEquals(length, a.value("surface_unknown_m"), 0.1);
    assertEquals(2 * length, a.value("longest_firm_unpaved_m"), 0.1);
    assertEquals(60, a.value("traffic_estimate_known_pct"), 0.1);
    assertEquals(length, a.value("high_traffic_estimate_m"), 0.1);
    assertEquals(length, a.value("poor_smoothness_m"), 0.1);
    assertEquals(50, a.value("model_moving_seconds"), 0);
    assertNotEquals(track.distance, a.value("distance_m"), 1);
  }

  @Test
  public void cleanupJoinCannotInheritRemovedEdgesTagsOrTime() {
    OsmTrack track = track("surface=asphalt", "surface=compacted estimated_traffic_class=5", "surface=asphalt");
    track.nodes.remove(1);
    track.nodes.get(1).origin = track.nodes.get(0); // cleanup repairs origins, not measurement provenance
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 0);
    assertTrue(a.value("metadata_unknown_m") > 100);
    assertEquals(0, a.value("surface_firm_unpaved_m"), 0);
    assertTrue(Double.isNaN(a.value("high_traffic_estimate_m")));
    assertTrue(Double.isNaN(a.value("model_moving_seconds")));
    assertEquals(10, a.value("partial_model_moving_seconds"), 0);
  }

  @Test
  public void tagsAreExactAndUnsupportedValuesRemainUnknown() {
    OsmTrack track = track("cycleway:surface=asphalt surface=mystery estimated_traffic_class=9 estimated_forest_class=0",
      "surface=fine_gravel estimated_traffic_class=3 estimated_forest_class=6 estimated_river_class=4");
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 0);
    assertEquals(0, a.value("surface_paved_m"), 0);
    assertEquals(50, a.value("surface_known_pct"), 0.1);
    assertEquals(50, a.value("forest_estimate_known_pct"), 0.1);
    assertEquals(0, a.value("high_traffic_estimate_m"), 0);
    assertEquals(a.value("surface_firm_unpaved_m"), a.value("high_forest_estimate_m"), 0.1);
  }

  @Test
  public void observationsAreImmutableAndNodeEventsAreEncounters() {
    OsmTrack track = track("highway=steps bicycle=dismount", "surface=asphalt");
    track.nodes.get(1).message.nodeKeyValues = "highway=traffic_signals barrier=gate";
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 0);
    assertEquals(1, a.value("tagged_signal_encounters"), 0);
    assertEquals(1, a.value("tagged_barrier_encounters"), 0);
    assertTrue(a.value("steps_m") > 0);
    track.nodes.clear();
    assertTrue(a.value("distance_m") > 0);
    try {
      a.values().put("distance_m", 0.0);
      fail("Measurements must be immutable");
    } catch (UnsupportedOperationException expected) {
      // immutable result
    }
  }

  @Test
  public void elevationWindowsDoNotInventClimbsAcrossMissingData() {
    OsmTrack track = track("surface=asphalt", "surface=asphalt", "surface=asphalt", "surface=asphalt");
    track.nodes.get(1).setSElev(Short.MIN_VALUE);
    track.nodes.get(2).setSElev((short) 800);
    track.nodes.get(3).setSElev((short) 800);
    track.nodes.get(4).setSElev((short) 800);
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 0);
    assertEquals(50, a.value("elevation_known_pct"), 0.1);
    assertEquals(0, a.value("ascent_100m_m"), 0);
    assertEquals(100, a.value("grade_window_covered_m"), 0);
    for (OsmPathElement node : track.nodes) node.setSElev(Short.MIN_VALUE);
    a = RouteRideAnalysis.measure(track, 0);
    assertTrue(Double.isNaN(a.value("ascent_100m_m")));
    assertEquals(0, a.value("elevation_known_pct"), 0);
  }

  @Test
  public void sustainedGradesUseDistanceWindowsRatherThanNodeSpacing() {
    OsmTrack dense = track("surface=asphalt", "surface=asphalt", "surface=asphalt", "surface=asphalt");
    OsmPathElement first = dense.nodes.get(0);
    for (OsmPathElement node : dense.nodes) {
      double distance = btools.util.CheapRuler.distance(first.getILon(), first.getILat(), node.getILon(), node.getILat());
      node.setSElev((short) Math.round(400 + distance * 0.4));
    }
    OsmTrack sparse = new OsmTrack();
    sparse.nodes.add(first);
    sparse.nodes.add(dense.nodes.get(dense.nodes.size() - 1));
    RouteRideAnalysis a = RouteRideAnalysis.measure(dense, 0);
    RouteRideAnalysis b = RouteRideAnalysis.measure(sparse, 0);
    assertEquals(200, a.value("steep_up_100m_m"), 0);
    assertEquals(a.value("ascent_100m_m"), b.value("ascent_100m_m"), 0.3);
    assertEquals(10, a.value("max_up_grade_100m_pct"), 0.3);
    assertTrue(a.value("grade_window_covered_pct") < 100);
    assertTrue(a.value("final_quarter_ascent_100m_m") >= 0);
  }

  @Test
  public void duplicatePointsDoNotAddMeters() {
    OsmTrack track = track("surface=asphalt");
    double before = RouteRideAnalysis.measure(track, 0).value("distance_m");
    track.nodes.add(track.nodes.get(1));
    assertEquals(before, RouteRideAnalysis.measure(track, 0).value("distance_m"), 0);
  }

  @Test
  public void collectingUnusedTagsDoesNotChangeRouteOrLeakIntoNextRequest() {
    RoutingEngine before = RoundTripFixture.engine("gravel", 90, 1000, rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY);
    RoutingEngine measured = RoundTripFixture.engine("gravel", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.keyValues = new java.util.HashMap<>();
      rc.keyValues.put("processUnusedTags", "1");
    });
    RoutingEngine after = RoundTripFixture.engine("gravel", 90, 1000, rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY);
    assertTrue(measured.roundTripOps().routingContext().processUnusedTags);
    assertFalse(after.roundTripOps().routingContext().processUnusedTags);
    long signature = btools.router.roundtrip.LoopCostOracle.geometrySignature(before.getFoundTrack());
    assertEquals(signature, btools.router.roundtrip.LoopCostOracle.geometrySignature(measured.getFoundTrack()));
    assertEquals(signature, btools.router.roundtrip.LoopCostOracle.geometrySignature(after.getFoundTrack()));
    assertEquals(before.getFoundTrack().cost, measured.getFoundTrack().cost);
  }

  @Test
  public void realFinishedRouteHasVerifiedMetadataWithoutMutation() {
    RoutingEngine engine = RoundTripFixture.engine("gravel", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    OsmTrack track = engine.getFoundTrack();
    String before = new FormatGpx(engine.roundTripOps().routingContext()).format(track);
    RouteRideAnalysis a = RouteRideAnalysis.measure(track, 2 * Math.PI * 1000);
    assertTrue("Real detail messages must carry segment provenance", a.value("metadata_verified_pct") > 80);
    assertTrue(a.value("surface_known_pct") > 0);
    assertTrue(a.value("distance_m") > 1000);
    assertEquals(before, new FormatGpx(engine.roundTripOps().routingContext()).format(track));
    for (Map.Entry<String, Double> entry : a.values().entrySet()) {
      if (entry.getKey().endsWith("_pct") && (entry.getKey().contains("known")
          || entry.getKey().contains("verified") || entry.getKey().contains("covered"))) {
        assertTrue(entry.getKey(), entry.getValue() >= 0 && entry.getValue() <= 100.001);
      }
    }
  }
}
