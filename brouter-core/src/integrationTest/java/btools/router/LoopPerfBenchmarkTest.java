package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Wall-clock A/B benchmark for the greedy loop generators, using only public
 * engine APIs so it compiles unchanged against both the optimized tree and the
 * pre-optimization baseline. Opt-in via {@code -Dgolden.tests=true} (reuses the
 * already-forwarded flag). Prints per-scenario and total routing time; compares
 * the MIN-over-repetitions (most stable estimate of achievable time).
 *
 * <pre>
 *   ./gradlew :brouter-core:test --tests '*LoopPerfBenchmarkTest' -Dgolden.tests=true --rerun-tasks
 * </pre>
 */
public class LoopPerfBenchmarkTest {

  private static final int WARMUP_PASSES = 2;
  private static final int TIMED_PASSES = 6;

  private static final Object[][] SCENARIOS = {
    {LoopTestRegion.DREIEICH, 8000, 0, "fastbike", RoundTripAlgorithm.GREEDY},
    {LoopTestRegion.DREIEICH, 8000, 0, "fastbike", RoundTripAlgorithm.ISO_GREEDY},
    {LoopTestRegion.DREIEICH, 8000, 90, "fastbike", RoundTripAlgorithm.GREEDY},
    {LoopTestRegion.DREIEICH, 8000, 90, "fastbike", RoundTripAlgorithm.ISO_GREEDY},
    {LoopTestRegion.URBAN_BERLIN, 8000, 0, "fastbike", RoundTripAlgorithm.GREEDY},
    {LoopTestRegion.URBAN_BERLIN, 8000, 0, "fastbike", RoundTripAlgorithm.ISO_GREEDY},
    {LoopTestRegion.RURAL_LOZERE, 8000, 0, "gravel", RoundTripAlgorithm.GREEDY},
    {LoopTestRegion.RURAL_LOZERE, 8000, 0, "gravel", RoundTripAlgorithm.ISO_GREEDY},
  };

  @Test
  public void benchmark() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4 missing", segDir.isDirectory());

    long[] bestPerScenario = new long[SCENARIOS.length];
    for (int i = 0; i < bestPerScenario.length; i++) bestPerScenario[i] = Long.MAX_VALUE;
    long bestTotal = Long.MAX_VALUE;

    for (int pass = 0; pass < WARMUP_PASSES + TIMED_PASSES; pass++) {
      long passTotal = 0;
      for (int i = 0; i < SCENARIOS.length; i++) {
        Object[] s = SCENARIOS[i];
        File profileFile = new File(projectDir, "misc/profiles2/" + s[3] + ".brf");
        if (!profileFile.exists()) { Assume.assumeTrue("profile missing", false); }
        long t0 = System.nanoTime();
        route((LoopTestRegion) s[0], (int) s[1], (int) s[2], (String) s[3],
          (RoundTripAlgorithm) s[4], segDir, profileFile);
        long dt = System.nanoTime() - t0;
        passTotal += dt;
        if (pass >= WARMUP_PASSES) {
          bestPerScenario[i] = Math.min(bestPerScenario[i], dt);
        }
      }
      if (pass >= WARMUP_PASSES) bestTotal = Math.min(bestTotal, passTotal);
    }

    System.out.println("=== LoopPerfBenchmark (min over " + TIMED_PASSES + " timed passes) ===");
    for (int i = 0; i < SCENARIOS.length; i++) {
      Object[] s = SCENARIOS[i];
      System.out.println(String.format(Locale.US, "  %-11s %-10s dir%-3d %-8s : %6.1f ms",
        ((RoundTripAlgorithm) s[4]).name().toLowerCase(),
        ((LoopTestRegion) s[0]).name().toLowerCase(), (int) s[2], s[3],
        bestPerScenario[i] / 1e6));
    }
    System.out.println(String.format(Locale.US, "  TOTAL (8 scenarios)         : %6.1f ms", bestTotal / 1e6));
  }

  private void route(LoopTestRegion region, int radius, int dir, String profile,
                     RoundTripAlgorithm algo, File segDir, File profileFile) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = region.ilon;
    start.ilat = region.ilat;
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = dir;
    rctx.roundTripDistance = radius;
    rctx.roundTripAlgorithm = algo;

    RoutingEngine re = new RoutingEngine(
      null, null, segDir, wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(0);
  }
}
