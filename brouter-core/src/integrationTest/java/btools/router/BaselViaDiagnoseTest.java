package btools.router;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import btools.util.CheapRuler;

/**
 * Why did a basel_80km_gravel_E via land in a village? Route it with waypoint steering ON, then for
 * each planned via print its coordinates, whether it falls inside a detected dense box, and the
 * nearest box (centre + radius). If a village-via is NOT inside any box, the box detector missed the
 * village → steering had nothing to penalise (the real cause).
 */
public class BaselViaDiagnoseTest {

  private static int lon(double d) { return (int) ((d + 180.0) * 1e6); }
  private static int lat(double d) { return (int) ((d + 90.0) * 1e6); }

  @Test
  public void diagnose() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File gravel = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4 missing", new File(segDir, "E5_N45.rd5").isFile());
    File out = new File(projectDir, "build/lorrach"); out.mkdirs();

    List<OsmNodeNamed> wp = new ArrayList<>();
    OsmNodeNamed s = new OsmNodeNamed(); s.ilon = lon(7.590); s.ilat = lat(47.560); s.name = "from"; wp.add(s);
    RoutingContext rc = new RoutingContext();
    rc.localFunction = gravel.getAbsolutePath();
    rc.startDirection = 90;
    rc.roundTripDistance = 12700;
    rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
    rc.roundTripStrictQuality = false;
    rc.roundTripDenseResidential = true; // build boxes + steering
    File o = new File(out, "_d");
    RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);

    List<OsmNodeNamed> boxes = rc.denseBoxes;
    System.out.println("=== dense boxes detected: " + (boxes == null ? 0 : boxes.size()) + " ===");
    if (boxes != null) {
      for (OsmNodeNamed b : boxes) {
        System.out.println(String.format("  box centre (%.4f,%.4f) radius=%.0fm",
          (b.ilon - 180000000) / 1e6, (b.ilat - 90000000) / 1e6, b.radius));
      }
    }

    RoundTripResult rtr = re.getLastRoundTripResult();
    List<OsmNodeNamed> vias = (rtr != null) ? rtr.getLoopWaypoints() : null;
    System.out.println("=== planned vias (steering ON) ===");
    if (vias != null) {
      for (int i = 0; i < vias.size(); i++) {
        OsmNodeNamed v = vias.get(i);
        boolean inBox = rc.isWithinDenseBox(v.ilon, v.ilat);
        double nearest = 1e9;
        if (boxes != null) {
          for (OsmNodeNamed b : boxes) {
            double d = CheapRuler.distance(v.ilon, v.ilat, b.ilon, b.ilat) - b.radius;
            if (d < nearest) nearest = d;
          }
        }
        System.out.println(String.format("  via %d (%.4f,%.4f)  inDenseBox=%-5b  distToNearestBoxEdge=%.0fm",
          i + 1, (v.ilon - 180000000) / 1e6, (v.ilat - 90000000) / 1e6, inBox, nearest));
      }
    }
  }
}
