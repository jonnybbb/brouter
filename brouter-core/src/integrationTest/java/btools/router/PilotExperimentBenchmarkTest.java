package btools.router;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.router.roundtrip.RoundTripAlgorithm;

/** The M1 corpus uses the same paired observations and report writer as M3. */
public class PilotExperimentBenchmarkTest {
  public static final class CellSpec {
    final String label;
    final String terrainCategory;
    final LoopTestRegion region;
    final int targetDistanceMeters;
    final int searchRadius;
    final double direction;
    final RoundTripAlgorithm algorithm;
    final String profileName;

    CellSpec(String label, String terrainCategory, LoopTestRegion region, int targetDistanceMeters,
             int searchRadius, double direction, RoundTripAlgorithm algorithm, String profileName) {
      this.label = label;
      this.terrainCategory = terrainCategory;
      this.region = region;
      this.targetDistanceMeters = targetDistanceMeters;
      this.searchRadius = searchRadius;
      this.direction = direction;
      this.algorithm = algorithm;
      this.profileName = profileName;
    }

    boolean isFastbike() {
      return "fastbike.brf".equals(profileName);
    }
  }

  private static List<CellSpec> buildCellSpecs() {
    List<CellSpec> cells = new ArrayList<>();

    // 1. Open: CRETE_SENESI (Tuscany)
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{0.0, 180.0}) {
        String base = "crete_" + (dist / 1000) + "km_dir" + (int) dir;
        cells.add(new CellSpec(base + "_greedy", "Open", LoopTestRegion.CRETE_SENESI, dist, rad, dir, RoundTripAlgorithm.GREEDY, "gravel.brf"));
        cells.add(new CellSpec(base + "_auto", "Open", LoopTestRegion.CRETE_SENESI, dist, rad, dir, RoundTripAlgorithm.AUTO, "gravel.brf"));
      }
    }

    // 2. Town: DREIEICH
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{90.0, 270.0}) {
        String base = "dreieich_" + (dist / 1000) + "km_dir" + (int) dir;
        cells.add(new CellSpec(base + "_greedy", "Town", LoopTestRegion.DREIEICH, dist, rad, dir, RoundTripAlgorithm.GREEDY, "gravel.brf"));
        cells.add(new CellSpec(base + "_auto", "Town", LoopTestRegion.DREIEICH, dist, rad, dir, RoundTripAlgorithm.AUTO, "gravel.brf"));
      }
    }

    // 3. Coastal: COASTAL_NICE
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{0.0, 90.0}) {
        String base = "nice_" + (dist / 1000) + "km_dir" + (int) dir;
        cells.add(new CellSpec(base + "_greedy", "Coastal", LoopTestRegion.COASTAL_NICE, dist, rad, dir, RoundTripAlgorithm.GREEDY, "gravel.brf"));
        cells.add(new CellSpec(base + "_auto", "Coastal", LoopTestRegion.COASTAL_NICE, dist, rad, dir, RoundTripAlgorithm.AUTO, "gravel.brf"));
      }
    }

    // 4. Hilly: GIRONA
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{90.0, 180.0}) {
        String base = "girona_" + (dist / 1000) + "km_dir" + (int) dir;
        cells.add(new CellSpec(base + "_greedy", "Hilly", LoopTestRegion.GIRONA, dist, rad, dir, RoundTripAlgorithm.GREEDY, "gravel.brf"));
        cells.add(new CellSpec(base + "_auto", "Hilly", LoopTestRegion.GIRONA, dist, rad, dir, RoundTripAlgorithm.AUTO, "gravel.brf"));
      }
    }

    // Fastbike correctness cases (4 cells)
    cells.add(new CellSpec("basel_30km_fastbike", "Fastbike", LoopTestRegion.BASEL, 30000, 4800, 0.0, RoundTripAlgorithm.AUTO, "fastbike.brf"));
    cells.add(new CellSpec("basel_100km_fastbike", "Fastbike", LoopTestRegion.BASEL, 100000, 15900, 0.0, RoundTripAlgorithm.AUTO, "fastbike.brf"));
    cells.add(new CellSpec("mallorca_30km_fastbike", "Fastbike", LoopTestRegion.MALLORCA, 30000, 4800, 0.0, RoundTripAlgorithm.AUTO, "fastbike.brf"));
    cells.add(new CellSpec("mallorca_100km_fastbike", "Fastbike", LoopTestRegion.MALLORCA, 100000, 15900, 0.0, RoundTripAlgorithm.AUTO, "fastbike.brf"));

    return cells;
  }


  @Test
  public void runPilotExperiment() throws Exception {
    List<FullABEvaluationMatrixTest.CellSpec> cells = new ArrayList<>();
    for (CellSpec c : buildCellSpecs()) {
      cells.add(new FullABEvaluationMatrixTest.CellSpec(c.label, c.region, c.targetDistanceMeters,
        c.searchRadius, c.direction, c.algorithm, c.profileName));
    }
    Assert.assertEquals(36, cells.size());
    for (String mode : new String[]{"best_of_n", "local"}) {
      for (int evaluations : new int[]{4, 8, 16, 32}) {
        FullABEvaluationMatrixTest.runEvaluation(cells, mode, evaluations, 3000, false);
      }
    }
  }
}
