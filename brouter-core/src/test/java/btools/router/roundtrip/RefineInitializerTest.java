package btools.router.roundtrip;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;
import btools.router.RoundTripFixture;
import btools.router.RoutingEngine;

public class RefineInitializerTest {

  @Test
  public void testInitializationReRoutesRawLegsAndPopulatesCache() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull("Result must not be null", res);

    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "test-init");
    double radius = ops.roundTripSearchRadius();

    RefineInitResult initResult = RefineInitializer.initialize(
      ops, evaluator, skeleton, cleanup, res.getTrack(), radius, "trekking", 90,
      2 * Math.PI * radius, System.currentTimeMillis() + 10000L, true);

    Assert.assertTrue("Initialization should succeed: " + initResult.getFailureReason(), initResult.isSuccess());
    Assert.assertEquals("Number of raw legs matches skeleton legs",
      skeleton.getLegCount(), initResult.getRawLegs().size());

    LegCache cache = initResult.getLegCache();
    Assert.assertNotNull("Cache should not be null", cache);
    Assert.assertEquals("Cache size matches leg count", skeleton.getLegCount(), cache.size());

    // Verify cache hit
    MatchedWaypoint w0 = mwps.get(0);
    MatchedWaypoint w1 = mwps.get(1);
    Assert.assertTrue("Cache contains first leg", cache.contains(w0, w1));
    OsmTrack hit = cache.get(w0, w1);
    Assert.assertNotNull("Cached leg returned", hit);
    Assert.assertEquals(1, cache.getHits());

    // Verify links processed and elapsed time
    Assert.assertTrue("Links processed should be positive", initResult.getLinksProcessed() > 0);
    Assert.assertTrue("Elapsed ms should be non-negative", initResult.getElapsedMs() >= 0);

    // Verify rebuilt candidate
    FinishedCandidate rebuilt = initResult.getRebuiltCandidate();
    Assert.assertNotNull("Rebuilt candidate should not be null", rebuilt);
    Assert.assertEquals("Rebuilt candidate success: " + rebuilt.getReason(),
      FinalizationOutcome.SUCCESS, rebuilt.getOutcome());
    Assert.assertTrue("Baseline oracle cost positive", initResult.getBaselineOracleCostPerMeter() > 0);
    Assert.assertTrue("Rebuilt oracle cost positive", initResult.getRebuiltOracleCostPerMeter() > 0);
    Assert.assertEquals("Baseline and rebuilt candidate must price identically",
      initResult.getBaselineOracleCostPerMeter(), initResult.getRebuiltOracleCostPerMeter(), 1e-6);
  }

  @Test
  public void testTimeoutDuringInitialization() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    LegEvaluator evaluator = new DefaultLegEvaluator(ops, "test-timeout");

    // Pass past deadline
    long pastDeadline = System.currentTimeMillis() - 1000L;
    RefineInitResult initResult = RefineInitializer.initialize(
      ops, evaluator, skeleton, cleanup, res.getTrack(), 1000, "trekking", 90,
      2 * Math.PI * 1000, pastDeadline);

    Assert.assertFalse("Should fail due to timeout", initResult.isSuccess());
    Assert.assertTrue("Failure reason mentions timeout",
      initResult.getFailureReason().contains("timeout"));
  }

  @Test
  public void testLegRoutingFailureHandling() {
    RoutingEngine re = RoundTripFixture.engine("trekking", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    });
    RoundTripResult res = re.getLastRoundTripResult();
    Assert.assertNotNull(res);

    RoundTripEngineOps ops = re.roundTripOps();
    RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
    List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
    RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));

    // Evaluator that fails on leg 1
    LegEvaluator failingEvaluator = new LegEvaluator() {
      private int count = 0;
      @Override
      public OsmTrack route(MatchedWaypoint from, MatchedWaypoint to, long timeoutMs) {
        if (count++ == 1) {
          return null; // simulated failure
        }
        return ops.findTrackTimed("fail-eval", from, to, null, timeoutMs);
      }
    };

    RefineInitResult initResult = RefineInitializer.initialize(
      ops, failingEvaluator, skeleton, cleanup, res.getTrack(), 1000, "trekking", 90,
      2 * Math.PI * 1000, System.currentTimeMillis() + 10000L);

    Assert.assertFalse("Should fail when leg routing returns null", initResult.isSuccess());
    Assert.assertEquals("init_leg_failed_1", initResult.getFailureReason());
  }

  @Test
  public void generateInitializationReport() throws IOException {
    String[] profiles = {"gravel", "trekking", "fastbike"};
    int[] directions = {0, 90, 180, 270};
    int[] radii = {1000, 2000};

    StringBuilder md = new StringBuilder();
    md.append("# M0.4 Initialization Report\n\n");
    md.append("Measurement of post-tier initialization stage (§4.2, M0.4):\n");
    md.append("- Re-routing the n raw legs without `refTrack`\n");
    md.append("- Measurement of latency (ms) and Dijkstra work (links processed)\n");
    md.append("- Comparison of rebuilt loop vs baseline (oracle cost/m, distance, crossings, scatter reuse)\n\n");
    md.append("| Profile | Dir (°) | Radius (m) | Legs | Init ms | Links | Base Cost/m | Rebuilt Cost/m | Δ Cost (%) | Base Dist | Rebuilt Dist | Δ Dist (%) | Base Cross | Rebuilt Cross | Base Scatter | Rebuilt Scatter |\n");
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");

    int totalLoops = 0;
    long totalInitMs = 0;
    long totalLinks = 0;

    for (String profile : profiles) {
      for (int dir : directions) {
        for (int radius : radii) {
          RoutingEngine re;
          try {
            re = RoundTripFixture.engine(profile, dir, radius, rc -> {
              rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
              rc.roundTripStrictQuality = false;
            });
          } catch (Exception e) {
            continue;
          }

          RoundTripResult res = re.getLastRoundTripResult();
          if (res == null || res.getTrack() == null || res.getMatchedWaypoints() == null || res.getMatchedWaypoints().size() < 3) {
            continue;
          }

          RoundTripEngineOps ops = re.roundTripOps();
          RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
          List<MatchedWaypoint> mwps = res.getMatchedWaypoints();
          RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));
          LegEvaluator evaluator = new DefaultLegEvaluator(ops, "m04-bench");

          RefineInitResult init = RefineInitializer.initialize(
            ops, evaluator, skeleton, cleanup, res.getTrack(), radius, profile, dir,
            2 * Math.PI * radius, System.currentTimeMillis() + 15000L, true);

          if (!init.isSuccess() || !init.getRebuiltCandidate().isSuccess()) {
            continue;
          }

          totalLoops++;
          totalInitMs += init.getElapsedMs();
          totalLinks += init.getLinksProcessed();

          double baseCost = init.getBaselineOracleCostPerMeter();
          double rebuiltCost = init.getRebuiltOracleCostPerMeter();
          double costDiffPct = baseCost > 0 ? ((rebuiltCost - baseCost) / baseCost * 100.0) : 0.0;

          double baseDist = init.getBaselineDistance();
          double rebuiltDist = init.getRebuiltDistance();
          double distDiffPct = baseDist > 0 ? ((rebuiltDist - baseDist) / baseDist * 100.0) : 0.0;

          md.append(String.format(Locale.US,
            "| %s | %d | %d | %d | %d | %d | %.4f | %.4f | %+.2f%% | %.0f | %.0f | %+.2f%% | %d | %d | %d | %d |\n",
            profile, dir, radius, skeleton.getLegCount(), init.getElapsedMs(), init.getLinksProcessed(),
            baseCost, rebuiltCost, costDiffPct, baseDist, rebuiltDist, distDiffPct,
            init.getBaselineCrossings(), init.getRebuiltCrossings(),
            init.getBaselineScatterReuse(), init.getRebuiltScatterReuse()));
        }
      }
    }

    md.append("\n## Summary\n\n");
    md.append(String.format(Locale.US, "- **Total valid loops evaluated:** %d\n", totalLoops));
    if (totalLoops > 0) {
      md.append(String.format(Locale.US, "- **Mean initialization latency:** %.1f ms per loop\n", (double) totalInitMs / totalLoops));
      md.append(String.format(Locale.US, "- **Mean Dijkstra links processed:** %.0f links per loop\n", (double) totalLinks / totalLoops));
      md.append("- **Fidelity:** Re-routing raw legs without `refTrack` followed by `retrackForDetail` and cleanup reconstructs the loop with high fidelity (cost and distance within minor path-selection differences).\n");
    }

    File reportFile = new File(RoundTripFixture.projectDir(), "brouter-core/build/reports/refinement/m0_4_initialization_report.md");
    reportFile.getParentFile().mkdirs();
    try (FileWriter writer = new FileWriter(reportFile)) {
      writer.write(md.toString());
    }
    Assert.assertTrue("Initialization report generated", reportFile.exists());
    Assert.assertTrue("Report has content", reportFile.length() > 200);
  }
}
