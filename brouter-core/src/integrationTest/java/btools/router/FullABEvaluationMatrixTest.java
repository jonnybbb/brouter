package btools.router;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;

import btools.router.roundtrip.RefineDiagnostics;
import btools.router.roundtrip.RoundTripAlgorithm;

/** One original/refined pair per real request, captured at the post-tier hook. */
public class FullABEvaluationMatrixTest {

  /** {@link #runEvaluation} switches loop.refine.measure on; restored here. */
  @Rule
  public final LoopPropertiesRule loopProperties = new LoopPropertiesRule();
  public static final class CellSpec {
    final String label;
    final LoopTestRegion region;
    final int targetDistanceMeters;
    final int searchRadius;
    final double direction;
    final RoundTripAlgorithm algorithm;
    final String profileName;

    CellSpec(String label, LoopTestRegion region, int targetDistanceMeters,
             int searchRadius, double direction, RoundTripAlgorithm algorithm, String profileName) {
      this.label = label;
      this.region = region;
      this.targetDistanceMeters = targetDistanceMeters;
      this.searchRadius = searchRadius;
      this.direction = direction;
      this.algorithm = algorithm;
      this.profileName = profileName;
    }

    boolean isGravel() {
      return "gravel.brf".equals(profileName);
    }

    boolean isFastbike() {
      return "fastbike.brf".equals(profileName);
    }
  }

  public static List<CellSpec> buildCellMatrix() {
    List<CellSpec> cells = new ArrayList<>();
    String[] profiles = new String[]{"gravel.brf", "fastbike.brf"};
    int[] distances = new int[]{30000, 50000, 75000, 80000, 100000};
    int[] radii = new int[]{4800, 8000, 11937, 12700, 15900};
    double[] directions = new double[]{0.0, 90.0, 180.0, 270.0};
    String[] dirLabels = new String[]{"N", "E", "S", "W"};

    for (LoopTestRegion region : new LoopTestRegion[]{
      LoopTestRegion.DREIEICH, LoopTestRegion.URBAN_BERLIN, LoopTestRegion.COASTAL_NICE,
      LoopTestRegion.RURAL_LOZERE, LoopTestRegion.MALLORCA, LoopTestRegion.FREIBURG,
      LoopTestRegion.BASEL, LoopTestRegion.ANNECY, LoopTestRegion.GRENOBLE, LoopTestRegion.GARMISCH,
      LoopTestRegion.GIRONA, LoopTestRegion.CRETE_SENESI, LoopTestRegion.FINALE_LIGURE,
      LoopTestRegion.VOSGES_LA_BRESSE}) {
      for (String prof : profiles) {
        String shortProf = prof.replace(".brf", "");
        if (!region.supportedProfiles.contains(shortProf)) {
          continue;
        }
        for (int i = 0; i < distances.length; i++) {
          int dist = distances[i];
          int rad = radii[i];
          if (dist < region.minLoopMetersForProfile(shortProf)) {
            continue;
          }
          for (int d = 0; d < directions.length; d++) {
            double dir = directions[d];
            if (region.isSeaBlockedDirection(dir)) {
              continue;
            }
            String label = String.format("%s_%dkm_%s_%s",
              region.name().toLowerCase(Locale.ROOT), dist / 1000, shortProf, dirLabels[d]);
            cells.add(new CellSpec(label, region, dist, rad, dir, RoundTripAlgorithm.AUTO, prof));
          }
        }
      }
    }
    return cells;
  }


  @Test
  public void runFullABEvaluationMatrix() throws Exception {
    String selection = System.getProperty("loop.refine.cells", "");
    List<CellSpec> cells = buildCellMatrix();
    Assert.assertEquals("Frozen regional matrix", 460, cells.size());
    if (!selection.isEmpty()) {
      List<String> labels = Arrays.asList(selection.split(","));
      cells.removeIf(cell -> !labels.contains(cell.label));
      Assert.assertEquals("Every selected cell must exist", labels.size(), cells.size());
    }
    String mode = System.getProperty("loop.refine.mode", "local");
    int evals = Integer.getInteger("loop.refine.evaluations", 16);
    int maxMs = Integer.getInteger("loop.refine.maxMs", 3000);
    runEvaluation(cells, mode, evals, maxMs, cells.size() == 460);
  }

  static void runEvaluation(List<CellSpec> cells, String mode, int evals, int maxMs, boolean fullMatrix) throws Exception {
    Assert.assertTrue("Evaluation count must match effective configuration", evals >= 1 && evals <= 64);
    Assert.assertTrue("Budget must match effective configuration", maxMs >= 500 && maxMs <= 8000);
    Assert.assertNotEquals("Refinement mode must be valid", btools.router.roundtrip.RefineConfig.Mode.NONE,
      btools.router.roundtrip.RefineConfig.parseMode(mode));
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segments = new File(projectDir, "segments4");
    Path output = Files.createTempDirectory(new File(projectDir, "brouter-core/build").toPath(), "refine-evaluation-");
    RefineEvaluationReport.writeManifest(output, projectDir.toPath(), segments.toPath(), cells, mode, evals, maxMs);
    // Measurement snapshots on for the run; the calling test's
    // LoopPropertiesRule puts the fork's value back afterwards.
    System.setProperty("loop.refine.measure", "true");
    List<RefineEvaluationReport.Cell> records = new ArrayList<>();
    try {
      for (CellSpec cell : cells) {
        RefineEvaluationReport.Cell record = evaluateCell(cell, segments, projectDir, mode, evals, maxMs);
        records.add(record);
        RefineEvaluationReport.appendRecord(output, record);
        System.out.println(cell.label + ": " + record.reason + ", added " + record.addedMs + " ms");
      }
    } finally {
      RefineEvaluationReport.writeSummary(output, records, fullMatrix, 3000);
      System.out.println("Paired refinement evidence: " + output);
    }
    RefineEvaluationReport.verifyManifest(output, projectDir.toPath(), segments.toPath());
    RefineEvaluationReport.Statistics gravel = new RefineEvaluationReport.Statistics(records, "gravel.brf", 3000);
    RefineEvaluationReport.Statistics fastbike = new RefineEvaluationReport.Statistics(records, "fastbike.brf", 3000);
    Assert.assertEquals("Gravel failed requests", 0, gravel.failed);
    Assert.assertEquals("Fastbike failed requests", 0, fastbike.failed);
    Assert.assertEquals("Gravel correctness regressions", 0, gravel.regressions);
    Assert.assertEquals("Fastbike correctness regressions", 0, fastbike.regressions);
    if (fullMatrix && Boolean.parseBoolean(System.getProperty("loop.refine.requireBars", "true"))) {
      Assert.assertTrue("Refinement has not met the product acceptance bars; see generated report", gravel.passes());
    }
  }

  static RefineEvaluationReport.Cell evaluateCell(CellSpec spec, File segments, File projectDir,
                                                  String mode, int evals, int maxMs) {
    List<OsmNodeNamed> waypoints = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "start";
    start.ilon = spec.region.ilon;
    start.ilat = spec.region.ilat;
    waypoints.add(start);
    RoutingContext context = new RoutingContext();
    context.localFunction = new File(projectDir, "misc/profiles2/" + spec.profileName).getAbsolutePath();
    context.roundTripDistance = spec.searchRadius;
    context.startDirection = (int) spec.direction;
    context.roundTripAlgorithm = spec.algorithm;
    context.roundTripStrictQuality = false;
    context.roundTripRefine = mode;
    context.roundTripRefineEvals = evals;
    context.roundTripRefineMaxMs = (long) maxMs;
    context.alternativeIdx = 0;
    // Preserve observational tags even when the active profile does not use them for cost.
    context.keyValues = new java.util.HashMap<>();
    context.keyValues.put("processUnusedTags", "1");
    RoutingEngine engine = new RoutingEngine(null, null, segments, waypoints, context,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    engine.quite = true;
    long startMs = System.currentTimeMillis();
    String failure = null;
    try {
      engine.doRun(60000L);
      failure = engine.getErrorMessage();
    } catch (RuntimeException e) {
      failure = e.toString();
    }
    long requestMs = System.currentTimeMillis() - startMs;
    RefineDiagnostics diagnostics = engine.getLastRefineDiagnostics();
    return new RefineEvaluationReport.Cell(spec.label, spec.profileName, diagnostics,
      requestMs, failure, engine.getFoundTrack() != null);
  }
}
