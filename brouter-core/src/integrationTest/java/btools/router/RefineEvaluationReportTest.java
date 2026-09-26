package btools.router;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import btools.router.roundtrip.RefineDiagnostics;
import btools.router.roundtrip.RefineRouteSnapshot;
import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripQualityResult;
import btools.router.roundtrip.RoundTripQualityResult.RejectionTier;

import static org.junit.Assert.*;

public class RefineEvaluationReportTest {
  @Test
  public void evenMedianAndNearestRankPercentileAreComputed() {
    assertEquals(2.5, RefineEvaluationReport.median(new ArrayList<>(Arrays.asList(4.0, 1.0, 3.0, 2.0))), 0);
    assertEquals(9, RefineEvaluationReport.percentile90(new ArrayList<>(
      Arrays.asList(10.0, 1.0, 8.0, 2.0, 7.0, 3.0, 6.0, 4.0, 5.0, 9.0))), 0);
  }

  @Test
  public void skipsAndTruncationsStayInQualityAndLatencyDenominators() {
    List<RefineEvaluationReport.Cell> records = new ArrayList<>();
    RefineDiagnostics accepted = new RefineDiagnostics();
    accepted.refineApplied = true;
    accepted.eligible = true;
    accepted.oracleCostPerMeterBefore = 10;
    accepted.oracleCostPerMeterAfter = 9;
    records.add(new RefineEvaluationReport.Cell("win", "gravel.brf", accepted, 100, null, true));
    RefineDiagnostics timeout = new RefineDiagnostics();
    timeout.refineApplied = true;
    timeout.refineTruncated = true;
    timeout.oracleCostPerMeterBefore = 10;
    timeout.oracleCostPerMeterAfter = 1;
    timeout.elapsedMs = 4000;
    records.add(new RefineEvaluationReport.Cell("timeout", "gravel.brf", timeout, 6000, null, true));
    records.add(new RefineEvaluationReport.Cell("skip", "gravel.brf", new RefineDiagnostics(), 50, null, false));
    RefineEvaluationReport.Statistics stats = new RefineEvaluationReport.Statistics(records, "gravel.brf", 3000);
    assertEquals(3, stats.cells);
    assertEquals(1, stats.eligible);
    assertEquals(1, stats.wins);
    assertEquals(0, stats.medianGain, 0);
    assertEquals(10.0 / 3, stats.meanGain, 0.00001);
    assertEquals(4000, stats.addedP90);
    assertEquals(6000, stats.requestP90);
    assertEquals(1, stats.truncated);
    assertFalse(stats.passes());
  }

  @Test
  public void missingMeasurementsCannotPassCorrectness() {
    RefineEvaluationReport.Cell cell = new RefineEvaluationReport.Cell("missing", "fastbike.brf", null, 12, null, true);
    RefineEvaluationReport.Statistics stats = new RefineEvaluationReport.Statistics(Arrays.asList(cell), "fastbike.brf", 3000);
    assertEquals(1, stats.regressions);
    assertFalse(stats.passes());
  }

  @Test
  public void reportComputesLengthGateAndCostRegressionsFromSnapshots() {
    RoutingEngine engine = RoundTripFixture.engine("gravel", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
    OsmTrack track = engine.getFoundTrack();
    RefineDiagnostics d = new RefineDiagnostics();
    d.requestedDistance = track.distance;
    d.baseline = RefineRouteSnapshot.capture(track, d.requestedDistance, "gravel", 90,
      engine.getLastRoundTripQuality());
    assertTrue(d.baseline.gateAccepted);
    track.distance += 1000;
    RoundTripQualityResult rejected = RoundTripQualityResult.builder().reject(RejectionTier.QUALITY, "test").build();
    d.result = RefineRouteSnapshot.capture(track, d.requestedDistance, "gravel", 90, rejected);
    assertEquals(d.requestedDistance, d.baseline.distance, 0);
    d.refineApplied = true;
    d.oracleCostPerMeterBefore = 10;
    d.oracleCostPerMeterAfter = 11;
    RefineEvaluationReport.Cell cell = new RefineEvaluationReport.Cell("worse", "gravel.brf", d, 50, null, true);
    RefineEvaluationReport.Statistics stats = new RefineEvaluationReport.Statistics(Arrays.asList(cell), "gravel.brf", 3000);
    assertEquals(1, stats.regressions);
    assertEquals(1, stats.lengthRegressions);
    assertEquals(1, stats.gateRegressions);
    assertEquals(1, stats.costRegressions);
    assertEquals(-10, stats.meanGain, 0);
    assertFalse(stats.passes());
  }

  @Test
  public void rideReportsPreserveUnknownMeasurementsAndEscapeLabels() throws Exception {
    OsmTrack track = new OsmTrack();
    track.nodes.add(OsmPathElement.create(188000000, 140000000, Short.MIN_VALUE, null));
    track.nodes.add(OsmPathElement.create(188001000, 140000000, Short.MIN_VALUE, track.nodes.get(0)));
    RefineDiagnostics d = new RefineDiagnostics();
    RoundTripQualityResult quality = RoundTripQualityResult.builder().accepted(true).build();
    d.baseline = RefineRouteSnapshot.capture(track, 100, "gravel", 0, quality);
    d.result = d.baseline;
    d.refineReason = "<unpriceable>";
    RefineEvaluationReport.Cell cell = new RefineEvaluationReport.Cell("<script>test</script>", "gravel.brf", d, 1, null, true);
    Path output = Files.createTempDirectory("ride-report-test-");
    try {
      RefineEvaluationReport.appendRecord(output, cell);
      RefineEvaluationReport.writeSummary(output, Arrays.asList(cell), false, 3000);
      List<String> lines = Files.readAllLines(output.resolve("ride-analysis.csv"));
      assertEquals(1 + 2 * d.baseline.ride.values().size(), lines.size());
      assertTrue(lines.stream().anyMatch(line -> line.endsWith("\"model_moving_seconds\",\"\"")));
      String html = Files.readString(output.resolve("ride-analysis.html"));
      assertTrue(html.contains("&lt;script&gt;test&lt;/script&gt;"));
      assertFalse(html.contains("<script>"));
      assertTrue(html.contains("unavailable"));
      assertTrue(Files.exists(output.resolve("ride-review.html")));
      String data = Files.readString(output.resolve("ride-review-data.js"));
      assertFalse(data.contains("<script>"));
      assertTrue(data.contains("\"sample\":true"));
      String gpx = Files.readString(output.resolve("gpx/pair-001-A.gpx"));
      assertTrue(gpx.contains("<name>Route A</name>"));
      assertTrue(gpx.contains("lat=\"50.000000\" lon=\"8.000000\""));
      assertFalse(gpx.contains("<ele>"));
      assertFalse(gpx.contains("original"));
      javax.xml.parsers.DocumentBuilderFactory.newInstance().newDocumentBuilder()
        .parse(output.resolve("gpx/pair-001-A.gpx").toFile());
      track.nodes.clear();
      assertEquals("Captured geometry remains immutable", 2, d.baseline.geometry.size());
    } finally {
      try (java.util.stream.Stream<Path> files = Files.walk(output)) {
        for (Path file : (Iterable<Path>) files.sorted(java.util.Comparator.reverseOrder())::iterator) Files.delete(file);
      }
    }
  }

  @Test
  public void matrixMatchesRegionalSuites() {
    List<FullABEvaluationMatrixTest.CellSpec> cells = FullABEvaluationMatrixTest.buildCellMatrix();
    assertEquals(460, cells.size());
    assertEquals(230, cells.stream().filter(FullABEvaluationMatrixTest.CellSpec::isGravel).count());
    assertEquals(230, cells.stream().filter(FullABEvaluationMatrixTest.CellSpec::isFastbike).count());
  }
}
