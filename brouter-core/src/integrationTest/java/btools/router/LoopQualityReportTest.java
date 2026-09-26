package btools.router;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import static org.junit.Assert.*;

import java.io.File;
import java.io.FileWriter;
import btools.router.roundtrip.LoopQualityMetrics;
import btools.router.roundtrip.RefineDiagnostics;

public class LoopQualityReportTest {

  @Rule
  public TemporaryFolder temp = new TemporaryFolder();

  @Test
  public void testRoundTripSerializationWithRefineTelemetry() throws Exception {
    LoopQualityMetrics metrics = LoopQualityMetrics.fromFields(
      12.5, 0.98, 5.0, 50200, 50000, 0.85, 10, 20, 0.45, 2.50, 15, 2, 30, 0, 0);
    double[][] coords = new double[][]{{8.6, 50.1}, {8.7, 50.2}};

    LoopQualityResult r = new LoopQualityResult(
      "dreieich_50km_gravel_N", LoopTestRegion.DREIEICH, 50000,
      "gravel", 0.0, metrics, null, coords, "auto");
    r.disclosed = false;
    r.character = new double[]{0.15, 0.60, 0.20, 0.25};
    r.requestMs = 1245L;
    r.gateVerdict = "ACCEPTED";
    r.rcs = 0.8234;
    r.gateCostPerM = 2.4123;
    r.oracleCostPerM = 2.3891;

    RefineDiagnostics diag = new RefineDiagnostics();
    diag.refineApplied = true;
    diag.refineReason = "accepted";
    diag.oracleCostPerMeterBefore = 2.4500;
    diag.oracleCostPerMeterAfter = 2.3891;
    diag.rcsBefore = 0.8100;
    diag.rcsAfter = 0.8234;
    diag.proposals = 16;
    diag.invalidProposals = 2;
    diag.evaluations = 4;
    diag.legsRouted = 8;
    diag.cacheHits = 2;
    diag.finalizations = 2;
    diag.chains = 1;
    diag.refineTruncated = false;
    diag.timeoutOperation = null;
    diag.elapsedMs = 185L;
    r.refineDiagnostics = diag;

    String json = LoopQualityReport.serializeResult(r);
    assertNotNull(json);
    assertTrue(json.contains("\"requestMs\":1245"));
    assertTrue(json.contains("\"gateVerdict\":\"ACCEPTED\""));
    assertTrue(json.contains("\"rcs\":0.8234"));
    assertTrue(json.contains("\"gateCostPerM\":2.4123"));
    assertTrue(json.contains("\"oracleCostPerM\":2.3891"));
    assertTrue(json.contains("\"refineDiagnostics\":{"));
    assertTrue(json.contains("\"refineApplied\":true"));
    assertTrue(json.contains("\"evaluations\":4"));

    File file = temp.newFile("test_result.json");
    try (FileWriter fw = new FileWriter(file)) {
      fw.write(json);
    }

    LoopQualityResult deserialized = LoopQualityReport.deserializeResult(file);
    assertNotNull(deserialized);
    assertEquals(r.label, deserialized.label);
    assertEquals(r.region, deserialized.region);
    assertEquals(r.distanceMeters, deserialized.distanceMeters);
    assertEquals(r.profileName, deserialized.profileName);
    assertEquals(r.direction, deserialized.direction, 1e-6);
    assertEquals(r.variant, deserialized.variant);
    assertEquals(r.requestMs, deserialized.requestMs);
    assertEquals(r.gateVerdict, deserialized.gateVerdict);
    assertEquals(r.rcs, deserialized.rcs, 1e-4);
    assertEquals(r.gateCostPerM, deserialized.gateCostPerM, 1e-4);
    assertEquals(r.oracleCostPerM, deserialized.oracleCostPerM, 1e-4);

    assertNotNull(deserialized.refineDiagnostics);
    RefineDiagnostics d2 = deserialized.refineDiagnostics;
    assertTrue(d2.refineApplied);
    assertEquals("accepted", d2.refineReason);
    assertEquals(2.4500, d2.oracleCostPerMeterBefore, 1e-4);
    assertEquals(2.3891, d2.oracleCostPerMeterAfter, 1e-4);
    assertEquals(0.8100, d2.rcsBefore, 1e-4);
    assertEquals(0.8234, d2.rcsAfter, 1e-4);
    assertEquals(16, d2.proposals);
    assertEquals(2, d2.invalidProposals);
    assertEquals(4, d2.evaluations);
    assertEquals(8, d2.legsRouted);
    assertEquals(2, d2.cacheHits);
    assertEquals(2, d2.finalizations);
    assertEquals(1, d2.chains);
    assertFalse(d2.refineTruncated);
    assertEquals(185L, d2.elapsedMs);
  }
}
