package btools.router;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

/**
 * Capsule DETECTION visualizer — tune toward MULTIPLE SMALLER capsules (one per distinct urban area:
 * Basel core, Saint-Louis, Weil, Riehen, Allschwil, Münchenstein…) instead of one broad blob.
 *
 * <p>The earlier blob came from morphological CLOSING (dilate→erode), which fuses adjacent towns. Here
 * closing is OFF by default and the density threshold is higher, so distinct dense cores stay separate.
 * Writes one polygon per detected capsule to {@code <region>__capsules.geojson} for the viewer, and
 * prints the count + per-capsule cell sizes so the threshold can be tuned. Params (system properties):
 * loop.capsule.densepercentile (0.88), loop.capsule.mindensenodes (12), loop.capsule.nogomincells (2),
 * loop.capsule.closing (false).
 */
public class CapsuleDetectVizTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(20, TimeUnit.MINUTES).build();

  private static final int CELL = RoutingEngine.DESIRABILITY_CELL;
  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};

  private File segDir, loopsDir, gravel;

  @Test
  public void detect() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();

    double pct = Double.parseDouble(System.getProperty("loop.capsule.densepercentile", "0.88"));
    int minNodes = Integer.parseInt(System.getProperty("loop.capsule.mindensenodes", "12"));
    int minCells = Integer.parseInt(System.getProperty("loop.capsule.nogomincells", "2"));
    boolean doClose = Boolean.parseBoolean(System.getProperty("loop.capsule.closing", "false"));
    System.out.println("=== capsule detection (pct=" + pct + " minNodes=" + minNodes + " minCells=" + minCells + " closing=" + doClose + ") ===");

    for (LoopTestRegion region : REGIONS) {
      Map<Long, double[]> grid = probeGrid(region.ilon, region.ilat, 12700);
      Set<Long> dense = CapsuleNogoBuilder.classifyDense(grid, pct, minNodes);
      if (doClose) dense = CapsuleNogoBuilder.closing(dense);
      List<Set<Long>> comps = CapsuleNogoBuilder.components(dense, minCells);
      List<OsmNodeNamed> polys = new ArrayList<>();
      StringBuilder sizes = new StringBuilder();
      for (Set<Long> c : comps) {
        polys.addAll(CapsuleNogoBuilder.polygonsFromCells(c, CELL, 1.0, minCells));
        sizes.append(c.size()).append(" ");
      }
      System.out.println(region.name() + ": gridCells=" + grid.size() + " denseCells=" + dense.size()
        + " capsules=" + comps.size() + " (cell-sizes: " + sizes.toString().trim() + ")");
      writeCapsules(new File(loopsDir, region.name().toLowerCase() + "__capsules.geojson"), polys);
    }
    System.out.println("[detect] wrote __capsules.geojson into " + loopsDir.getAbsolutePath());
  }

  private Map<Long, double[]> probeGrid(int cx, int cy, int radius) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = cx; s.ilat = cy; s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
      rc.roundTripCapsule = true;
      System.setProperty("loop.capsule.nogoweight", "0");
      File o = new File(loopsDir, "_grid");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      return re.desirabilityGrid;
    } catch (Exception e) { return new java.util.HashMap<>(); }
  }

  private void writeCapsules(File f, List<OsmNodeNamed> polys) throws Exception {
    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    boolean first = true;
    for (OsmNodeNamed ng : polys) {
      if (!(ng instanceof OsmNogoPolygon)) continue;
      OsmNogoPolygon p = (OsmNogoPolygon) ng;
      if (!first) sb.append(","); first = false;
      sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"capsule\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
      for (int i = 0; i < p.points.size(); i++) { if (i > 0) sb.append(","); ll(sb, p.points.get(i).x, p.points.get(i).y); }
      if (!p.points.isEmpty()) { sb.append(","); ll(sb, p.points.get(0).x, p.points.get(0).y); }
      sb.append("]}}");
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private static void ll(StringBuilder sb, int ilon, int ilat) {
    sb.append("[").append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6))
      .append(",").append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]");
  }
}
