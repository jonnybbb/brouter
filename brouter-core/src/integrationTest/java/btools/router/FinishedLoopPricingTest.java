package btools.router;

import java.io.File;
import java.util.Arrays;
import java.util.Collection;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import static org.junit.Assert.*;

/** Production geometries that previously stopped refinement before its first proposal. */
@RunWith(Parameterized.class)
public class FinishedLoopPricingTest {
  @Parameterized.Parameters(name = "{0}")
  public static Collection<Object[]> cases() {
    String selection = System.getProperty("loop.refine.pricingCells", "");
    if (!selection.isEmpty()) {
      java.util.List<Object[]> cases = new java.util.ArrayList<>();
      for (String label : selection.split(",")) cases.add(new Object[]{label});
      return cases;
    }
    return Arrays.asList(new Object[][] {
      {"dreieich_30km_gravel_E"}, {"dreieich_30km_fastbike_E"},
      {"mallorca_30km_gravel_N"}, {"mallorca_30km_fastbike_N"},
      {"basel_30km_gravel_N"}, {"basel_30km_fastbike_N"},
      // Sparse bulge connector and relocated continuation endpoint regressions.
      {"rural_lozere_80km_gravel_E"}, {"basel_30km_fastbike_S"},
      {"mallorca_100km_fastbike_E"}
    });
  }

  private final String label;

  public FinishedLoopPricingTest(String label) {
    this.label = label;
  }

  @Test
  public void exactFinishedBaselineIsContinuouslyPriceable() throws Exception {
    File root = new File("..").getCanonicalFile();
    FullABEvaluationMatrixTest.CellSpec spec = FullABEvaluationMatrixTest.buildCellMatrix().stream()
      .filter(cell -> cell.label.equals(label)).findFirst().orElseThrow(AssertionError::new);
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "start";
    start.ilon = spec.region.ilon;
    start.ilat = spec.region.ilat;
    RoutingContext context = new RoutingContext();
    context.localFunction = new File(root, "misc/profiles2/" + spec.profileName).getAbsolutePath();
    context.roundTripDistance = spec.searchRadius;
    context.startDirection = (int) spec.direction;
    context.roundTripAlgorithm = spec.algorithm;
    context.roundTripStrictQuality = false;
    RoutingEngine engine = new RoutingEngine(null, null, new File(root, "segments4"),
      Arrays.asList(start), context, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    engine.quite = true;
    engine.doRun(60000L);
    assertNull(engine.getErrorMessage());
    OsmTrack track = engine.getFoundTrack();
    assertNotNull(track);
    long signature = btools.router.roundtrip.LoopCostOracle.geometrySignature(track);
    int cost = btools.router.roundtrip.LoopCostOracle.priceCost(engine.roundTripOps(), track, track.getMatchedWaypoints());
    assertTrue(label + ": continuous price=" + cost + "; " + engine.getLastPricingFailure(), cost > 0);
    assertEquals("continuous", engine.getLastPricingMethod());
    assertEquals(signature, btools.router.roundtrip.LoopCostOracle.geometrySignature(track));
    assertEquals("Pricing must not retain prior walk state", cost,
      btools.router.roundtrip.LoopCostOracle.priceCost(engine.roundTripOps(), track, track.getMatchedWaypoints()));
  }
}
