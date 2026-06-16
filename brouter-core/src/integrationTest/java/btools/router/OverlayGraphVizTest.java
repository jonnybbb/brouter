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
 * Stage A of the TRUE contraction overlay: detect multiple small capsules, build the
 * {@link OverlayGraph} (real portals + real ring/cross corridors + inter-capsule connectors),
 * and dump it as {@code <region>__overlay.geojson} so the contracted graph can be eyeballed on
 * the map BEFORE the loop search is wired on top. Edges are coloured by kind in the viewer.
 *
 * <p>Params (system properties): loop.overlay.maxcapsules (6), loop.overlay.numportals (8),
 * loop.overlay.kconnector (3), loop.overlay.maxconnectorkm (8), loop.capsule.densepercentile (0.88),
 * loop.capsule.mindensenodes (12), loop.capsule.nogomincells (2).
 */
public class OverlayGraphVizTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(40, TimeUnit.MINUTES).build();

  private static final int CELL = RoutingEngine.DESIRABILITY_CELL;
  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.BASEL, LoopTestRegion.FREIBURG};

  private File segDir, loopsDir, gravel;

  @Test
  public void buildOverlay() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    loopsDir = new File(projectDir, "build/capsule-unified/loops"); loopsDir.mkdirs();

    int maxCaps = Integer.parseInt(System.getProperty("loop.overlay.maxcapsules", "6"));
    int numPortals = Integer.parseInt(System.getProperty("loop.overlay.numportals", "8"));
    int kConn = Integer.parseInt(System.getProperty("loop.overlay.kconnector", "3"));
    double maxConnM = Double.parseDouble(System.getProperty("loop.overlay.maxconnectorkm", "8")) * 1000.0;
    double pct = Double.parseDouble(System.getProperty("loop.capsule.densepercentile", "0.88"));
    int minNodes = Integer.parseInt(System.getProperty("loop.capsule.mindensenodes", "12"));
    int minCells = Integer.parseInt(System.getProperty("loop.capsule.nogomincells", "2"));
    System.out.println("=== overlay build (maxCaps=" + maxCaps + " portals=" + numPortals
      + " kConn=" + kConn + " maxConnKm=" + (maxConnM / 1000) + ") ===");

    for (LoopTestRegion region : REGIONS) {
      Map<Long, double[]> grid = probeGrid(region.ilon, region.ilat, 12700);
      Set<Long> dense = CapsuleNogoBuilder.classifyDense(grid, pct, minNodes);
      List<Set<Long>> comps = CapsuleNogoBuilder.components(dense, minCells);
      List<OsmNodeNamed> polyNodes = new ArrayList<>();
      for (Set<Long> c : comps) polyNodes.addAll(CapsuleNogoBuilder.polygonsFromCells(c, CELL, 1.0, minCells));
      List<OsmNogoPolygon> capsules = new ArrayList<>();
      for (OsmNodeNamed n : polyNodes) if (n instanceof OsmNogoPolygon) capsules.add((OsmNogoPolygon) n);

      double softNogo = Double.parseDouble(System.getProperty("loop.overlay.softnogo", "0"));
      OverlayGraph.MaskedRouter router = p2pRouter();
      long t0 = System.currentTimeMillis();
      OverlayGraph g = OverlayGraph.build(region.ilon, region.ilat, capsules, router,
        maxCaps, numPortals, kConn, maxConnM, softNogo);
      long ms = System.currentTimeMillis() - t0;

      int[] byKind = new int[4];
      for (OverlayGraph.Edge e : g.edges) byKind[e.kind]++;
      System.out.println(region.name() + ": capsules=" + capsules.size() + " (contracted " + Math.min(maxCaps, capsules.size())
        + ")  nodes=" + g.nodes.size() + "  edges=" + g.edges.size()
        + " [access=" + byKind[OverlayGraph.ACCESS] + " connector=" + byKind[OverlayGraph.CONNECTOR]
        + " corridor=" + byKind[OverlayGraph.CORRIDOR] + " ring=" + byKind[OverlayGraph.RING] + "]  builtMs=" + ms);

      writeOverlay(new File(loopsDir, region.name().toLowerCase() + "__overlay.geojson"), g);
    }
    System.out.println("[overlay] wrote __overlay.geojson into " + loopsDir.getAbsolutePath());
  }

  private OverlayGraph.MaskedRouter p2pRouter() {
    return (fl, fa, tl, ta, nogos) -> {
      try {
        List<OsmNodeNamed> wp = new ArrayList<>();
        OsmNodeNamed a = new OsmNodeNamed(); a.ilon = fl; a.ilat = fa; a.name = "from"; wp.add(a);
        OsmNodeNamed b = new OsmNodeNamed(); b.ilon = tl; b.ilat = ta; b.name = "to"; wp.add(b);
        RoutingContext rc = new RoutingContext();
        rc.localFunction = gravel.getAbsolutePath();
        if (nogos != null && !nogos.isEmpty()) rc.nogopoints = new ArrayList<>(nogos);
        File o = new File(loopsDir, "_p2p");
        RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
          RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
        re.doRun(0);
        return re.getFoundTrack();
      } catch (Exception e) { return null; }
    };
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

  private static final String[] KIND = {"access", "connector", "corridor", "ring"};

  private void writeOverlay(File f, OverlayGraph g) throws Exception {
    StringBuilder sb = new StringBuilder(1 << 18);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    boolean first = true;
    for (OverlayGraph.Edge e : g.edges) {
      if (e.geom == null || e.geom.nodes == null || e.geom.nodes.size() < 2) continue;
      if (!first) sb.append(","); first = false;
      sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"").append(KIND[e.kind])
        .append("\",\"distM\":").append(e.distM).append(",\"cost\":").append(String.format(Locale.US, "%.0f", e.cost))
        .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
      List<OsmPathElement> ns = e.geom.nodes;
      for (int i = 0; i < ns.size(); i++) { if (i > 0) sb.append(","); ll(sb, ns.get(i).getILon(), ns.get(i).getILat()); }
      sb.append("]}}");
    }
    for (OverlayGraph.Node n : g.nodes) {
      if (!first) sb.append(","); first = false;
      String kind = n.type == OverlayGraph.START ? "ostart" : "oportal";
      sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"").append(kind)
        .append("\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
      ll(sb, n.ilon, n.ilat);
      sb.append("}}");
    }
    sb.append("]}");
    try (Writer w = new FileWriter(f)) { w.write(sb.toString()); }
  }

  private static void ll(StringBuilder sb, int ilon, int ilat) {
    sb.append("[").append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6))
      .append(",").append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]");
  }
}
