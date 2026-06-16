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
import java.util.concurrent.TimeUnit;

import btools.util.CheapRuler;

/**
 * Demonstrator (increment 1 of the working-graph contraction): build the ring+cross
 * skeleton from REAL Basel roads and dump it as GeoJSON so it can be eyeballed. Proves the
 * user's model — outer ring of real boundary portals + two real crossing corridors — is a
 * sensible, fully-routable replacement for the dense interior, before wiring it into the loop
 * planner. Uses local segments4 + gravel; skips if absent.
 */
public class CapsuleSkeletonDemoTest {

  @Rule
  public Timeout timeout = Timeout.builder().withTimeout(15, TimeUnit.MINUTES).build();

  @Test
  public void buildBaselSkeleton() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profile = new File(projectDir, "misc/profiles2/gravel.brf");
    Assume.assumeTrue("segments4/E5_N45.rd5 missing", new File(segDir, "E5_N45.rd5").isFile());
    Assume.assumeTrue("gravel.brf missing", profile.exists());
    File outDir = new File(projectDir, "build/capsule-skeleton");
    outDir.mkdirs();

    int cx = LoopTestRegion.BASEL.ilon, cy = LoopTestRegion.BASEL.ilat;
    double radiusM = 4000; // approximate Basel dense-capsule radius

    CapsuleSkeleton.Router router = (fl, fa, tl, ta) -> {
      try {
        List<OsmNodeNamed> wp = new ArrayList<>();
        OsmNodeNamed a = new OsmNodeNamed(); a.ilon = fl; a.ilat = fa; a.name = "from"; wp.add(a);
        OsmNodeNamed b = new OsmNodeNamed(); b.ilon = tl; b.ilat = ta; b.name = "to"; wp.add(b);
        RoutingContext rc = new RoutingContext();
        rc.localFunction = profile.getAbsolutePath();
        File o = new File(outDir, "_p2p");
        RoutingEngine re = new RoutingEngine(o.getAbsolutePath(), o.getAbsolutePath(), segDir, wp, rc,
          RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
        re.doRun(0);
        return re.getFoundTrack();
      } catch (Exception e) {
        return null;
      }
    };

    CapsuleSkeleton.Result skel = CapsuleSkeleton.build(cx, cy, radiusM, 8, router);
    System.out.println("BASEL skeleton: " + skel.portalCount() + "/8 portals, "
      + skel.ring.size() + " ring segs, " + skel.cross.size() + " cross corridors");

    StringBuilder sb = new StringBuilder(1 << 16);
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
    boolean first = true;
    // capsule circle (approx, for context)
    first = comma(sb, first);
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"capsule\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i <= 48; i++) {
      int[] p = CheapRuler.destination(cx, cy, radiusM, 360.0 * i / 48);
      if (i > 0) sb.append(",");
      appendLonLat(sb, p[0], p[1]);
    }
    sb.append("]}}");
    // ring
    for (OsmTrack t : skel.ring) { first = comma(sb, first); appendTrack(sb, t, "ring"); }
    // cross
    for (OsmTrack t : skel.cross) { first = comma(sb, first); appendTrack(sb, t, "cross"); }
    // portals
    for (int[] p : skel.portals) {
      if (p == null) continue;
      first = comma(sb, first);
      sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"portal\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
      appendLonLat(sb, p[0], p[1]);
      sb.append("}}");
    }
    // start marker
    first = comma(sb, first);
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"start\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":");
    appendLonLat(sb, cx, cy);
    sb.append("}}");
    sb.append("]}");
    try (Writer w = new FileWriter(new File(outDir, "basel.geojson"))) { w.write(sb.toString()); }
    System.out.println("[capsule-skeleton] wrote " + new File(outDir, "basel.geojson").getAbsolutePath());
  }

  private static boolean comma(StringBuilder sb, boolean first) {
    if (!first) sb.append(",");
    return false;
  }

  private static void appendTrack(StringBuilder sb, OsmTrack t, String kind) {
    sb.append("{\"type\":\"Feature\",\"properties\":{\"kind\":\"").append(kind)
      .append("\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    for (int i = 0; i < t.nodes.size(); i++) {
      OsmPathElement n = t.nodes.get(i);
      if (i > 0) sb.append(",");
      appendLonLat(sb, n.getILon(), n.getILat());
    }
    sb.append("]}}");
  }

  private static void appendLonLat(StringBuilder sb, int ilon, int ilat) {
    sb.append("[").append(String.format(Locale.US, "%.6f", (ilon - 180000000) / 1e6))
      .append(",").append(String.format(Locale.US, "%.6f", (ilat - 90000000) / 1e6)).append("]");
  }
}
