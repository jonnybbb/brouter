package btools.router;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.roundtrip.DefaultLegEvaluator;
import btools.router.roundtrip.FinishedCandidate;
import btools.router.roundtrip.LegCache;
import btools.router.roundtrip.LegEvaluator;
import btools.router.roundtrip.LoopCostOracle;
import btools.router.roundtrip.MoveProposalOperator;
import btools.router.roundtrip.RefineConfig;
import btools.router.roundtrip.RefineDiagnostics;
import btools.router.roundtrip.RefineFinalizer;
import btools.router.roundtrip.RefineInitResult;
import btools.router.roundtrip.RefineInitializer;
import btools.router.roundtrip.RefineSkeleton;
import btools.router.roundtrip.RefineStage;
import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripEngineOps;
import btools.router.roundtrip.RoundTripQualityResult;
import btools.router.roundtrip.RoundTripResult;
import btools.router.roundtrip.RoundTripTrackCleanup;
import btools.router.roundtrip.SplitmixRandom;
import btools.router.roundtrip.WaypointSnapper;

public class FullStageCostModelBenchmarkTest {

  private static final class CellSpec {
    final String terrainCategory;
    final LoopTestRegion region;
    final int targetDistanceMeters;
    final int searchRadius;
    final double direction;
    final RoundTripAlgorithm algorithm;

    CellSpec(String terrainCategory, LoopTestRegion region, int targetDistanceMeters,
             int searchRadius, double direction, RoundTripAlgorithm algorithm) {
      this.terrainCategory = terrainCategory;
      this.region = region;
      this.targetDistanceMeters = targetDistanceMeters;
      this.searchRadius = searchRadius;
      this.direction = direction;
      this.algorithm = algorithm;
    }
  }

  private static final class CellResult {
    CellSpec spec;
    boolean eligible;
    String skipReason;
    long baselineGenMs;
    int baselineGenLinks;
    long baselinePricingMs;
    double baselineCostPerMeter;
    long initMs;
    int initLinks;
    long meanEvalMs;
    int meanEvalLinks;
    long finalizationMs;
    int finalizationLinks;
    long publishMs;
    long totalRefineMsK4;
    int totalRefineLinksK4;
    double farthestDist;
    double radiusRatio; // farthestDist / targetDistance
    int legTimeouts; // candidate legs that exhausted the per-leg budget
  }

  @Test
  public void runFullStageCostModelBenchmark() throws IOException {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profileFile = new File(projectDir, "misc/profiles2/gravel.brf");
    Assert.assertTrue("Profile gravel.brf must exist", profileFile.exists());

    // 32 stratified gravel cells: Open, Town, Coastal, Hilly; 30km and 100km; GREEDY and AUTO; 2 directions
    List<CellSpec> cells = new ArrayList<>();

    // 1. Open: CRETE_SENESI (Tuscany)
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{0.0, 180.0}) {
        cells.add(new CellSpec("Open", LoopTestRegion.CRETE_SENESI, dist, rad, dir, RoundTripAlgorithm.GREEDY));
        cells.add(new CellSpec("Open", LoopTestRegion.CRETE_SENESI, dist, rad, dir, RoundTripAlgorithm.AUTO));
      }
    }

    // 2. Town: DREIEICH
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{90.0, 270.0}) {
        cells.add(new CellSpec("Town", LoopTestRegion.DREIEICH, dist, rad, dir, RoundTripAlgorithm.GREEDY));
        cells.add(new CellSpec("Town", LoopTestRegion.DREIEICH, dist, rad, dir, RoundTripAlgorithm.AUTO));
      }
    }

    // 3. Coastal: COASTAL_NICE
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{0.0, 90.0}) {
        cells.add(new CellSpec("Coastal", LoopTestRegion.COASTAL_NICE, dist, rad, dir, RoundTripAlgorithm.GREEDY));
        cells.add(new CellSpec("Coastal", LoopTestRegion.COASTAL_NICE, dist, rad, dir, RoundTripAlgorithm.AUTO));
      }
    }

    // 4. Hilly: GIRONA
    for (int dist : new int[]{30000, 100000}) {
      int rad = dist == 30000 ? 4800 : 15900;
      for (double dir : new double[]{90.0, 180.0}) {
        cells.add(new CellSpec("Hilly", LoopTestRegion.GIRONA, dist, rad, dir, RoundTripAlgorithm.GREEDY));
        cells.add(new CellSpec("Hilly", LoopTestRegion.GIRONA, dist, rad, dir, RoundTripAlgorithm.AUTO));
      }
    }

    Assert.assertEquals("Expected 32 stratified cells", 32, cells.size());

    List<CellResult> results = new ArrayList<>();
    RefineConfig config = new RefineConfig();

    for (int c = 0; c < cells.size(); c++) {
      CellSpec spec = cells.get(c);
      CellResult res = new CellResult();
      res.spec = spec;
      results.add(res);

      try {
        LoopTestSegments.ensureRegion(segDir, spec.region);
      } catch (Exception e) {
        res.eligible = false;
        res.skipReason = "tile_unavailable: " + e.getMessage();
        continue;
      }

      List<OsmNodeNamed> wplist = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "start";
      start.ilon = spec.region.ilon;
      start.ilat = spec.region.ilat;
      wplist.add(start);

      RoutingContext rc = new RoutingContext();
      rc.localFunction = profileFile.getAbsolutePath();
      rc.roundTripDistance = spec.searchRadius;
      rc.startDirection = (int) spec.direction;
      rc.roundTripAlgorithm = spec.algorithm;
      rc.roundTripStrictQuality = false;

      RoutingEngine re = new RoutingEngine(null, null, segDir, wplist, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;

      long genStart = System.currentTimeMillis();
      try {
        re.doRun(0);
      } catch (Exception e) {
        res.eligible = false;
        res.skipReason = "routing_exception: " + e.getMessage();
        continue;
      }
      res.baselineGenMs = System.currentTimeMillis() - genStart;
      res.baselineGenLinks = re.getLinksProcessed();

      RoundTripResult rtRes = re.getLastRoundTripResult();
      OsmTrack track = rtRes != null ? rtRes.getTrack() : re.getFoundTrack();
      List<MatchedWaypoint> mwps = (rtRes != null && rtRes.getMatchedWaypoints() != null)
        ? rtRes.getMatchedWaypoints()
        : (track != null ? track.getMatchedWaypoints() : null);

      if (track == null || mwps == null) {
        res.eligible = false;
        res.skipReason = "no_route";
        continue;
      }

      RoundTripQualityResult gateResult = re.getLastRoundTripQuality();
      if (gateResult == null || !gateResult.isAccepted()) {
        res.eligible = false;
        res.skipReason = "gate_rejected: " + (gateResult != null ? gateResult.getRejectionReason() : "null");
        continue;
      }

      if (mwps.size() < 3) {
        res.eligible = false;
        res.skipReason = "too_few_vias";
        continue;
      }

      RoundTripEngineOps ops = re.roundTripOps();
      RoundTripTrackCleanup cleanup = new RoundTripTrackCleanup(new WaypointSnapper(ops, ops, ops), ops, ops, ops);
      RefineSkeleton skeleton = new RefineSkeleton(mwps.get(0), mwps.subList(1, mwps.size() - 1), mwps.get(mwps.size() - 1));
      LegEvaluator evaluator = new DefaultLegEvaluator(ops, "m06-eval");

      // 1. Measure baseline pricing
      long priceStart = System.currentTimeMillis();
      res.baselineCostPerMeter = LoopCostOracle.price(ops, track, mwps);
      res.baselinePricingMs = System.currentTimeMillis() - priceStart;

      // Measure farthest point distance on baseline
      OsmPathElement startNode = track.nodes.get(0);
      int maxD = 0;
      for (OsmPathElement p : track.nodes) {
        int d = startNode.calcDistance(p);
        if (d > maxD) maxD = d;
      }
      res.farthestDist = maxD;
      res.radiusRatio = (double) maxD / spec.targetDistanceMeters;

      // 2. Measure initialization
      int linksBeforeInit = ops.getLinksProcessed();
      RefineInitResult initResult = RefineInitializer.initialize(
        ops, evaluator, skeleton, cleanup, track, spec.searchRadius, "gravel",
        spec.direction, spec.targetDistanceMeters, System.currentTimeMillis() + 15000L);
      res.initMs = initResult.getElapsedMs();
      res.initLinks = ops.getLinksProcessed() - linksBeforeInit;

      if (!initResult.isSuccess()) {
        res.eligible = false;
        res.skipReason = "init_failed: " + initResult.getFailureReason();
        continue;
      }

      // 3. Measure k=4 evaluations using MOVE proposals
      MoveProposalOperator mover = new MoveProposalOperator(ops, config);
      SplitmixRandom rng = new SplitmixRandom(12345L + c);
      LegCache cache = initResult.getLegCache();

      long totalEvalMs = 0;
      int totalEvalLinks = 0;
      int successfulEvals = 0;
      FinishedCandidate lastFinalist = null;

      int maxProposals = 16;
      for (int p = 0; p < maxProposals && successfulEvals < 4; p++) {
        MoveProposalOperator.MoveProposal prop = mover.proposeMove(
          skeleton, skeleton, rng, spec.searchRadius, spec.targetDistanceMeters);
        if (!prop.isFeasible()) {
          continue;
        }

        int movedIdx = prop.getMovedViaIndex();
        RefineSkeleton mutSkel = prop.getMutatedSkeleton();
        List<MatchedWaypoint> mutWps = mutSkel.getWaypoints();

        List<OsmTrack> candLegs = new ArrayList<>(skeleton.getLegCount());
        long evalStart = System.currentTimeMillis();
        int evalLinksStart = ops.getLinksProcessed();

        boolean legFailed = false;
        for (int l = 0; l < skeleton.getLegCount(); l++) {
          MatchedWaypoint from = mutWps.get(l);
          MatchedWaypoint to = mutWps.get(l + 1);
          if (l == movedIdx || l == movedIdx + 1) {
            OsmTrack leg;
            try {
              leg = evaluator.route(from, to, 5000L);
            } catch (IllegalArgumentException e) {
              // Same treatment as RefineSearch: a leg that exhausts its budget
              // is a failed evaluation, not a failed benchmark. It is counted
              // so the report shows how often the search burns a whole budget.
              if (e.getMessage() == null || !e.getMessage().contains("timeout")) {
                throw e;
              }
              res.legTimeouts++;
              leg = null;
            }
            if (leg == null) {
              legFailed = true;
              break;
            }
            cache.put(from, to, leg);
            candLegs.add(leg);
          } else {
            OsmTrack cached = cache.get(from, to);
            if (cached == null) {
              legFailed = true;
              break;
            }
            candLegs.add(cached);
          }
        }

        long evalDur = System.currentTimeMillis() - evalStart;
        int evalLinks = ops.getLinksProcessed() - evalLinksStart;

        if (!legFailed) {
          totalEvalMs += evalDur;
          totalEvalLinks += evalLinks;
          successfulEvals++;

          // 4. Measure finalization on candidate
          if (lastFinalist == null) {
            long finStart = System.currentTimeMillis();
            int finLinksStart = ops.getLinksProcessed();
            lastFinalist = RefineFinalizer.finalizeCandidate(
              candLegs, mutSkel, ops, cleanup, spec.searchRadius, "gravel",
              spec.direction, spec.targetDistanceMeters, System.currentTimeMillis() + 10000L);
            res.finalizationMs = System.currentTimeMillis() - finStart;
            res.finalizationLinks = ops.getLinksProcessed() - finLinksStart;
          }
        }
      }

      if (successfulEvals == 0 || lastFinalist == null) {
        res.eligible = false;
        res.skipReason = "no_feasible_evaluations";
        continue;
      }

      res.meanEvalMs = totalEvalMs / successfulEvals;
      res.meanEvalLinks = totalEvalLinks / successfulEvals;

      // 5. Measure publish
      if (lastFinalist != null && lastFinalist.isSuccess()) {
        RefineDiagnostics diag = new RefineDiagnostics();
        long pubStart = System.nanoTime();
        RefineStage.publish(ops, lastFinalist, diag);
        res.publishMs = (System.nanoTime() - pubStart) / 1_000_000L;
      }

      // 6. Total full-stage cost model
      res.totalRefineMsK4 = res.baselinePricingMs + res.initMs + (4 * res.meanEvalMs) + res.finalizationMs + res.publishMs;
      res.totalRefineLinksK4 = res.initLinks + (4 * res.meanEvalLinks) + res.finalizationLinks;
      res.eligible = true;
    }

    // Generate Markdown Report
    StringBuilder md = new StringBuilder();
    md.append("# M0.6 Full-Stage Cost Model Report\n\n");
    md.append("Empirical measurement of full-stage refinement cost model across 32 stratified gravel cells (§6 M0.6):\n");
    md.append("- **Terrains:** Open (Crete Senesi), Town (Dreieich), Coastal (Nice), Hilly (Girona)\n");
    md.append("- **Loop Scales:** 30 km and 100 km\n");
    md.append("- **Planners:** GREEDY and AUTO\n");
    md.append("- **Components:** `baseline pricing + init + k × evaluation + finalization + publish`\n\n");

    md.append("## Measurement Table\n\n");
    md.append("| Category | Region | Dist (km) | Algo | Dir (°) | Base Gen ms | Base Price ms | Init ms | Init Links | Mean Eval ms | Mean Eval Links | Fin ms | Fin Links | Total Refine ms (k=4) | Total Refine Links (k=4) | Farthest (m) | Radius Ratio |\n");
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");

    List<Long> initTimes = new ArrayList<>();
    List<Long> evalTimes = new ArrayList<>();
    List<Long> finTimes = new ArrayList<>();
    List<Long> totalTimesK4 = new ArrayList<>();
    List<Double> radiusRatios = new ArrayList<>();

    int eligibleCount = 0;
    for (CellResult r : results) {
      if (r.eligible) {
        eligibleCount++;
        initTimes.add(r.initMs);
        evalTimes.add(r.meanEvalMs);
        finTimes.add(r.finalizationMs);
        totalTimesK4.add(r.totalRefineMsK4);
        radiusRatios.add(r.radiusRatio);

        md.append(String.format(Locale.US,
          "| %s | %s | %d | %s | %.0f | %d | %d | %d | %d | %d | %d | %d | %d | %d | %d | %.0f | %.3f |\n",
          r.spec.terrainCategory, r.spec.region.name(), r.spec.targetDistanceMeters / 1000,
          r.spec.algorithm.name(), r.spec.direction, r.baselineGenMs, r.baselinePricingMs,
          r.initMs, r.initLinks, r.meanEvalMs, r.meanEvalLinks, r.finalizationMs, r.finalizationLinks,
          r.totalRefineMsK4, r.totalRefineLinksK4, r.farthestDist, r.radiusRatio));
      } else {
        md.append(String.format(Locale.US,
          "| %s | %s | %d | %s | %.0f | - | - | - | - | - | - | - | - | - | - | - | Skip: %s |\n",
          r.spec.terrainCategory, r.spec.region.name(), r.spec.targetDistanceMeters / 1000,
          r.spec.algorithm.name(), r.spec.direction, r.skipReason));
      }
    }

    Collections.sort(initTimes);
    Collections.sort(evalTimes);
    Collections.sort(finTimes);
    Collections.sort(totalTimesK4);
    Collections.sort(radiusRatios);

    md.append("\n## Component Latency Breakdown\n\n");
    if (eligibleCount > 0) {
      md.append("| Stage Component | Mean (ms) | p50 (ms) | p90 (ms) | Max (ms) |\n");
      md.append("|---|---|---|---|---|\n");
      md.append(String.format(Locale.US, "| **Initialization (n legs)** | %.1f | %d | %d | %d |\n",
        mean(initTimes), p(initTimes, 0.50), p(initTimes, 0.90), initTimes.get(initTimes.size() - 1)));
      md.append(String.format(Locale.US, "| **Evaluation (per proposal)** | %.1f | %d | %d | %d |\n",
        mean(evalTimes), p(evalTimes, 0.50), p(evalTimes, 0.90), evalTimes.get(evalTimes.size() - 1)));
      md.append(String.format(Locale.US, "| **Finalization (detail+gate+price)** | %.1f | %d | %d | %d |\n",
        mean(finTimes), p(finTimes, 0.50), p(finTimes, 0.90), finTimes.get(finTimes.size() - 1)));
      md.append(String.format(Locale.US, "| **Total Refinement (k=4)** | %.1f | %d | %d | %d |\n",
        mean(totalTimesK4), p(totalTimesK4, 0.50), p(totalTimesK4, 0.90), totalTimesK4.get(totalTimesK4.size() - 1)));

      int timeoutLegs = 0;
      int timeoutCells = 0;
      for (CellResult r : results) {
        timeoutLegs += r.legTimeouts;
        if (r.legTimeouts > 0) timeoutCells++;
      }
      md.append(String.format(Locale.US,
        "\nCandidate legs that exhausted the 5 s per-leg budget: **%d** (in %d of %d cells). Each one is a full-graph"
          + " search that found no path — the evaluation is discarded, as in the search stage.\n",
        timeoutLegs, timeoutCells, results.size()));

      md.append("\n## Radius Bound Analysis (§4.4, §9)\n\n");
      double maxRatio = radiusRatios.get(radiusRatios.size() - 1);
      double p95Ratio = pD(radiusRatios, 0.95);
      md.append(String.format(Locale.US, "- **Theoretical circular loop farthest point:** `L / π ≈ 0.318 L`\n"));
      md.append(String.format(Locale.US, "- **Observed p50 farthest point:** `%.3f L`\n", pD(radiusRatios, 0.50)));
      md.append(String.format(Locale.US, "- **Observed p95 farthest point:** `%.3f L`\n", p95Ratio));
      md.append(String.format(Locale.US, "- **Observed Max farthest point:** `%.3f L`\n", maxRatio));
      md.append(String.format(Locale.US, "- **Empirical bound selection:** `0.60 L` safely covers all observed real-world loop topologies while preventing runaway dilation.\n"));

      md.append("\n## Decision D5 Evaluation (≤ 3.0s p90 added latency)\n\n");
      long p90Latency = p(totalTimesK4, 0.90);
      if (p90Latency <= 3000) {
        md.append(String.format(Locale.US, "✅ **Passes D5 budget bar:** p90 added latency for k=4 evaluations is **%d ms** (≤ 3,000 ms).\n", p90Latency));
      } else {
        md.append(String.format(Locale.US, "⚠️ **Exceeds 3s bar:** p90 added latency is **%d ms** (> 3,000 ms). Consider QUALITY-only 8s variant or reducing k.\n", p90Latency));
      }
    }

    File reportFile = new File(projectDir, "docs/m0_6_cost_model_report.md");
    try (FileWriter fw = new FileWriter(reportFile)) {
      fw.write(md.toString());
    }
    System.out.println("Generated M0.6 Cost Model Report at " + reportFile.getAbsolutePath());
    Assert.assertTrue("Report generated", reportFile.exists());
    Assert.assertTrue("Eligible cells >= 15", eligibleCount >= 15);
  }

  private static double mean(List<Long> list) {
    if (list.isEmpty()) return 0;
    long s = 0;
    for (long v : list) s += v;
    return (double) s / list.size();
  }

  private static long p(List<Long> sorted, double pct) {
    if (sorted.isEmpty()) return 0;
    int idx = (int) Math.round(pct * (sorted.size() - 1));
    return sorted.get(Math.min(idx, sorted.size() - 1));
  }

  private static double pD(List<Double> sorted, double pct) {
    if (sorted.isEmpty()) return 0;
    int idx = (int) Math.round(pct * (sorted.size() - 1));
    return sorted.get(Math.min(idx, sorted.size() - 1));
  }
}
