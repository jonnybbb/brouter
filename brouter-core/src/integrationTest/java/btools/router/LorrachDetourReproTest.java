package btools.router;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;

/**
 * Regression anchor for the via-placement lobe fixed by the angular-sweep +
 * unimodal-radius terms in {@link GreedyRoundTripPlanner}.
 *
 * <p>Before the fix, basel_80km_gravel_E [auto] placed a via (via3) in south
 * Lörrach at ~(47.6143,7.6553) — the SAME bearing-from-start as the prior via
 * but with the radius collapsed 18km→8km — forcing a ~1km lobe down into the
 * town and back. The sweep term penalises a via that fails to advance the loop's
 * angular sweep; the unimodal term forbids the radius climbing back out after the
 * apogee. The test asserts no intermediate via lands near the old lobe tip.
 *
 * <p>The 80km case is deterministic and asserted. The 100km case is run for
 * diagnostics only (its AUTO route varies run-to-run). Set {@code -Ddump.tag=<t>}
 * to also write route/via GeoJSON to {@code /tmp} for rendering.
 */
public class LorrachDetourReproTest {

  private static final double TIP_LAT = 47.6143, TIP_LON = 7.6553;
  /** The fix relocates the offending via >5km away; 1km leaves comfortable margin. */
  private static final int LOBE_TIP_CLEARANCE_M = 1000;

  @Test
  public void traceBaselGravelEast() throws Exception {
    trace(12700, "80km", true);   // single Lörrach lobe — asserted
    trace(15900, "100km", false); // multi-lobe tangle — diagnostic only (nondeterministic)
  }

  private void trace(int radius, String label, boolean assertNoLobe) throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profileFile = new File(projectDir, "misc/profiles2/gravel.brf");
    String dumpTag = System.getProperty("dump.tag"); // null = no /tmp dumps

    System.out.println("================= " + label + " AUTO =================");
    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = (int) ((7.590 + 180.0) * 1e6);
    start.ilat = (int) ((47.560 + 90.0) * 1e6);
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = 90; // East
    rctx.roundTripDistance = radius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    rctx.roundTripStrictQuality = false;

    File out = File.createTempFile("lorrach", "");
    RoutingEngine re = new RoutingEngine(out.getAbsolutePath(), out.getAbsolutePath(),
      segDir, wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    OsmTrack track = re.getFoundTrack();
    org.junit.Assume.assumeNotNull(track); // skip if segments unavailable
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    double[] cum = new double[n];
    for (int k = 1; k < n; k++) cum[k] = cum[k - 1] + nodes.get(k - 1).calcDistance(nodes.get(k));

    if (dumpTag != null) {
      StringBuilder rj = new StringBuilder("[");
      for (int k = 0; k < n; k++) {
        if (k > 0) rj.append(",");
        rj.append(String.format(java.util.Locale.US, "[%.6f,%.6f]",
          nodes.get(k).getILat() / 1e6 - 90.0, nodes.get(k).getILon() / 1e6 - 180.0));
      }
      rj.append("]");
      java.nio.file.Files.write(java.nio.file.Paths.get("/tmp/route_" + label + "_" + dumpTag + ".json"),
        rj.toString().getBytes());
    }

    System.out.println("track dist=" + track.distance + " nodes=" + n
      + " corridorX=" + RoundTripQualityGate.countCorridorCrossings(nodes)
      + " selfX=" + RoundTripQualityGate.countSelfIntersections(track));

    OsmPathElement tip = OsmPathElement.create(
      (int) ((TIP_LON + 180.0) * 1e6), (int) ((TIP_LAT + 90.0) * 1e6), (short) 0, null);
    List<MatchedWaypoint> vias = track.matchedWaypoints;
    int minIntermediateViaToTip = Integer.MAX_VALUE;
    System.out.println("vias (" + (vias == null ? 0 : vias.size()) + "): name | pos | arc | road");
    if (vias != null) {
      for (MatchedWaypoint v : vias) {
        int vlon = v.waypoint != null ? v.waypoint.ilon : v.crosspoint.ilon;
        int vlat = v.waypoint != null ? v.waypoint.ilat : v.crosspoint.ilat;
        OsmPathElement vp = OsmPathElement.create(vlon, vlat, (short) 0, null);
        int near = 0;
        for (int k = 0; k < n; k++) if (nodes.get(k).calcDistance(vp) < nodes.get(near).calcDistance(vp)) near = k;
        OsmPathElement nn = nodes.get(near);
        String ntags = nn.message != null && nn.message.wayKeyValues != null ? nn.message.wayKeyValues : "-";
        System.out.printf("  via name=%-14s (%.5f,%.5f) arc=%.1fkm road=%s%n",
          v.name, vlat / 1e6 - 90.0, vlon / 1e6 - 180.0, cum[near] / 1000, ntags);
        boolean intermediate = !"from".equals(v.name) && !"to".equals(v.name);
        if (intermediate) minIntermediateViaToTip = Math.min(minIntermediateViaToTip, vp.calcDistance(tip));
      }
    }

    if (assertNoLobe) {
      assertTrue("an intermediate via sits on the old Lörrach lobe tip ("
          + minIntermediateViaToTip + "m ≤ " + LOBE_TIP_CLEARANCE_M
          + "m) — the via-placement lobe has regressed",
        minIntermediateViaToTip > LOBE_TIP_CLEARANCE_M);
    }
  }
}
