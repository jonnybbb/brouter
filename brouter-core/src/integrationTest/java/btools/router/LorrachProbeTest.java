package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import btools.util.CheapRuler;

/**
 * Diagnostic for the basel_80km_gravel_E "Lörrach detour": route the W→E gap point-to-point with
 * gravel and report whether the natural path dips into Lörrach (a local cost preference) or takes a
 * direct northern crossing. Also routes via a northern waypoint (the user's red line near Rümmingen)
 * to compare cost. Prints the highway-class breakdown + closest approach to Lörrach centre.
 */
public class LorrachProbeTest {

  // BRouter ilon/ilat = (deg+180/90)*1e6
  private static int lon(double d) { return (int) ((d + 180.0) * 1e6); }
  private static int lat(double d) { return (int) ((d + 90.0) * 1e6); }
  private static final double LOER_LON = 7.6612, LOER_LAT = 47.6122;

  private File segDir, gravel, out;

  @Test
  public void probe() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    out = new File(projectDir, "build/lorrach"); out.mkdirs();

    // W approach (Binzen side, west of the Wiese) and E continuation (Hauingen/Steinen side, east)
    int[] w = {lon(7.622), lat(47.628)};
    int[] e = {lon(7.700), lat(47.640)};
    int[] north = {lon(7.640), lat(47.648)}; // the user's northern red-line crossing (near Rümmingen)

    System.out.println("=== direct W->E gravel route ===");
    OsmTrack direct = route(new int[][]{w, e});
    report("direct", direct);

    System.out.println("=== W->north->E gravel route (forced via the northern crossing) ===");
    OsmTrack viaNorth = route(new int[][]{w, north, e});
    report("viaNorth", viaNorth);
  }

  private OsmTrack route(int[][] pts) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      for (int i = 0; i < pts.length; i++) {
        OsmNodeNamed n = new OsmNodeNamed(); n.ilon = pts[i][0]; n.ilat = pts[i][1];
        n.name = i == 0 ? "from" : (i == pts.length - 1 ? "to" : "via"); wp.add(n);
      }
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      File o = new File(out, "_p");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
      re.doRun(0);
      return re.getFoundTrack();
    } catch (Exception ex) { return null; }
  }

  private void report(String tag, OsmTrack t) {
    if (t == null || t.nodes == null) { System.out.println("  " + tag + ": no track"); return; }
    double dmin = 1e9; Map<String, Integer> hw = new LinkedHashMap<>();
    int total = 0;
    for (int i = 1; i < t.nodes.size(); i++) {
      OsmPathElement a = t.nodes.get(i - 1), b = t.nodes.get(i);
      int seg = a.calcDistance(b);
      double d = CheapRuler.distance(b.getILon(), b.getILat(), lon(LOER_LON), lat(LOER_LAT));
      if (d < dmin) dmin = d;
      String h = RoadCharacterScore.highwayOf(b.message != null ? b.message.wayKeyValues : null);
      if (h == null) h = "(none)";
      hw.merge(h, seg, Integer::sum); total += seg;
    }
    System.out.println(String.format("  %s: dist=%dm  closestLörrach=%.0fm", tag, t.distance, dmin));
    final int tot = Math.max(1, total);
    hw.entrySet().stream().sorted((x, y) -> y.getValue() - x.getValue()).limit(8).forEach(en ->
      System.out.println(String.format("     %-14s %5dm (%.0f%%)", en.getKey(), en.getValue(), 100.0 * en.getValue() / tot)));
  }
}
