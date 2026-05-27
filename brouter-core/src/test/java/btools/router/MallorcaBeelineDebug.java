package btools.router;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.Assume;
import org.junit.Test;

import btools.mapaccess.OsmNode;

/**
 * Diagnostic-only test: re-runs the same routing as
 * {@code LoopQualityTest.loopQuality[mallorca_50km_fastbike_W]} for the
 * GREEDY variant, writes the FULL unsubsampled track to a known path, and
 * prints per-edge analysis (long edges, beeline markers, ferry tags,
 * waypoint placements) so we can pin down why visible "beelines" show up
 * in the report when the gate's continuity check says {@code maxGap=0,
 * cont=1.00}.
 *
 * <p>Opt-in via {@code -Dloop.tests=true}. Not part of the standard build.
 */
public class MallorcaBeelineDebug {

  @Test
  public void dumpMallorca50kmFastbikeWGreedy() throws Exception {
    Assume.assumeTrue("Diagnostic — run with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));

    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4 not present", segDir.isDirectory());
    File profileFile = new File(projectDir, "misc/profiles2/fastbike.brf");
    Assume.assumeTrue("fastbike.brf not present", profileFile.exists());

    // Match the test exactly: MALLORCA(2.650, 39.570) with W (270°), 50km.
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = (int) ((2.650 + 180) * 1_000_000);
    start.ilat = (int) ((39.570 + 90) * 1_000_000);
    List<OsmNodeNamed> wplist = new ArrayList<>();
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = 270;
    rctx.roundTripDistance = 8000; // matches SEARCH_RADII for 50km
    rctx.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;

    File outDir = new File("build/reports/loops/debug");
    outDir.mkdirs();
    String outPath = new File(outDir, "mallorca_50km_fastbike_W_greedy_" + System.currentTimeMillis()).getAbsolutePath();

    RoutingEngine re = new RoutingEngine(outPath, outPath, segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = false; // print routing decisions to stdout
    re.doRun(0);

    if (re.getErrorMessage() != null) {
      System.err.println("Routing error: " + re.getErrorMessage());
      return;
    }
    OsmTrack track = re.getFoundTrack();
    if (track == null || track.nodes == null) {
      System.err.println("No track produced.");
      return;
    }

    // Dump per-edge analysis.
    File analysisFile = new File(outDir, "mallorca_50km_fastbike_W_greedy_analysis.txt");
    try (FileWriter fw = new FileWriter(analysisFile)) {
      analyzeEdges(track, fw);
    }
    System.out.println("Wrote analysis: " + analysisFile.getAbsolutePath());
    System.out.println("Wrote GPX:      " + re.getOutfile());
  }

  private static void analyzeEdges(OsmTrack t, FileWriter fw) throws Exception {
    int n = t.nodes.size();
    fw.write("Track: " + n + " nodes, total distance=" + t.distance + "m\n\n");

    // Per-edge: length, tags, marker.
    int totalLen = 0;
    int beelineCount = 0;
    int ferryCount = 0;
    int longUntaggedCount = 0;
    StringBuilder longEdges = new StringBuilder();
    StringBuilder beelineEdges = new StringBuilder();
    StringBuilder ferryEdges = new StringBuilder();
    double cum = 0;

    for (int i = 1; i < n; i++) {
      OsmPathElement a = t.nodes.get(i - 1);
      OsmPathElement b = t.nodes.get(i);
      int len = a.calcDistance(b);
      totalLen += len;
      String tags = (b.message != null) ? b.message.wayKeyValues : null;
      boolean isBeeline = tags != null && tags.contains("direct_segment");
      boolean isFerry = tags != null && tags.contains("route=ferry");
      cum += len;

      double aLat = a.getILat() / 1_000_000.0 - 90;
      double aLon = a.getILon() / 1_000_000.0 - 180;
      double bLat = b.getILat() / 1_000_000.0 - 90;
      double bLon = b.getILon() / 1_000_000.0 - 180;
      String coords = String.format(Locale.US,
        "(%.5f,%.5f)→(%.5f,%.5f)", aLat, aLon, bLat, bLon);

      if (isBeeline) {
        beelineCount++;
        beelineEdges.append(String.format(Locale.US,
          "  edge %4d: %5dm  cum=%.1fkm  %s  tags=%s%n",
          i, len, cum / 1000.0, coords, tags));
      }
      if (isFerry) {
        ferryCount++;
        ferryEdges.append(String.format(Locale.US,
          "  edge %4d: %5dm  cum=%.1fkm  %s  tags=%s%n",
          i, len, cum / 1000.0, coords, tags));
      }
      // Long edges: above 200m AND no tags is highly suspect; above 500m
      // even with tags deserves attention.
      if (len > 500) {
        if (tags == null) longUntaggedCount++;
        longEdges.append(String.format(Locale.US,
          "  edge %4d: %5dm  cum=%.1fkm  %s  tags=%s  costF=%s%n",
          i, len, cum / 1000.0, coords,
          tags != null ? tags : "(none)",
          b.message != null ? String.format(Locale.US, "%.2f", b.message.costfactor) : "?"));
      }
    }

    fw.write(String.format("Summary:\n"));
    fw.write(String.format("  total edges:                 %d\n", n - 1));
    fw.write(String.format("  total length:                %dm (track.distance reports %dm)\n",
      totalLen, t.distance));
    fw.write(String.format("  direct_segment (beeline):    %d\n", beelineCount));
    fw.write(String.format("  ferry edges:                 %d\n", ferryCount));
    fw.write(String.format("  long (>500m) untagged edges: %d\n\n", longUntaggedCount));

    if (beelineCount > 0) {
      fw.write("Beeline (direct_segment) edges:\n");
      fw.write(beelineEdges.toString());
      fw.write("\n");
    }
    if (ferryCount > 0) {
      fw.write("Ferry edges:\n");
      fw.write(ferryEdges.toString());
      fw.write("\n");
    }
    fw.write("Long (>500m) edges (any tagging):\n");
    fw.write(longEdges.toString());
  }
}
