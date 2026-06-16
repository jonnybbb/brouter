package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import btools.util.CheapRuler;

/**
 * Self-contained check (bypasses the flaky gradle report): route basel_80km_gravel_E as AUTO / GREEDY
 * / ISO_GREEDY with waypoint steering OFF vs ON and print the loop's closest approach to Lörrach
 * centre. Steering ON should keep the loop clear of the town (the user's "no city dips").
 */
public class LorrachSteerTest {

  private static int lon(double d) { return (int) ((d + 180.0) * 1e6); }
  private static int lat(double d) { return (int) ((d + 90.0) * 1e6); }
  private static final double LOER_LON = 7.6612, LOER_LAT = 47.6122;

  private File segDir, gravel, out;

  @Test
  public void steer() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    segDir = new File(projectDir, "segments4");
    gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    out = new File(projectDir, "build/lorrach"); out.mkdirs();

    int[] targets = {30000, 50000, 75000, 80000, 100000};
    int[] radii = {4800, 8000, 11937, 12700, 15900};
    int[] dirs = {90, 270}; String[] dl = {"E", "W"};
    System.out.println("=== Basel gravel AUTO — closest approach to Lörrach centre (steer off vs on) ===");
    for (int i = 0; i < targets.length; i++) {
      for (int d = 0; d < dirs.length; d++) {
        double[] off = approach(route(RoundTripAlgorithm.AUTO, false, radii[i], dirs[d]));
        double[] on = approach(route(RoundTripAlgorithm.AUTO, true, radii[i], dirs[d]));
        String flag = (off[0] < 800 && on[0] >= 800) ? "  <-- FIXED" : (off[0] < 800 && on[0] < 800 ? "  (still dips)" : "");
        System.out.println(String.format("  basel_%dkm_%s  off=%5.0fm(d%.0f) on=%5.0fm(d%.0f)%s",
          targets[i] / 1000, dl[d], off[0], off[1], on[0], on[1], flag));
      }
    }
  }

  private double[] approach(OsmTrack t) {
    double dmin = 1e9;
    if (t != null && t.nodes != null) {
      for (OsmPathElement n : t.nodes) {
        double dd = CheapRuler.distance(n.getILon(), n.getILat(), lon(LOER_LON), lat(LOER_LAT));
        if (dd < dmin) dmin = dd;
      }
    }
    return new double[]{dmin, t != null ? t.distance : 0};
  }

  private OsmTrack route(RoundTripAlgorithm algo, boolean steer) { return route(algo, steer, 12700, 90); }

  private OsmTrack route(RoundTripAlgorithm algo, boolean steer, int radius, int dir) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.ilon = lon(7.590); s.ilat = lat(47.560); s.name = "from"; wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = gravel.getAbsolutePath();
      rc.startDirection = dir;
      rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = algo;
      rc.roundTripStrictQuality = false;
      rc.roundTripDenseResidential = steer; // build boxes -> waypoint steering active
      File o = new File(out, "_s");
      RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      OsmTrack t = re.getFoundTrack();
      return (t != null) ? t : re.getLastRejectedTrack();
    } catch (Exception e) { return null; }
  }
}
