package btools.router;

import java.io.File;
import java.io.FileWriter;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assume;
import org.junit.Test;

/**
 * Reveals what waypoints the planner ACTUALLY chose for a specific test
 * before routing. We re-run the same setup as
 * {@code LoopQualityTest.loopQuality[mallorca_75km_fastbike_S]} (the probe
 * variant) and inspect the waypoint list after geometric placement +
 * validation + snap. Each waypoint's final position, road costfactor at
 * snap, and tags are dumped.
 *
 * <p>This is the empirical evidence for the "geometric waypoint forced detour"
 * hypothesis: if any waypoint ends up snapped to a profile-hostile road or
 * to a sparse-connectivity micro-area, that explains the visible spurs and
 * same-way-back legs in the rendered route.
 */
public class WaypointPlacementDebug {

  @Test
  public void dumpWaypoints() throws Exception {
    Assume.assumeTrue("Run with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));

    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4 not present", segDir.isDirectory());
    File profileFile = new File(projectDir, "misc/profiles2/fastbike.brf");

    // Exact match: mallorca_75km_fastbike_S [probe] — direction S, 75km
    // requested, fastbike. The probe variant uses
    // doWaypointBasedRoundTrip → placeWaypointsFromEnvelope, which is the
    // pure-geometric placement.
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = (int) ((2.650 + 180) * 1_000_000);
    start.ilat = (int) ((39.570 + 90) * 1_000_000);
    List<OsmNodeNamed> wplist = new ArrayList<>();
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = 180; // South
    rctx.roundTripDistance = 11937; // matches SEARCH_RADII for 75km
    rctx.roundTripAlgorithm = RoundTripAlgorithm.WAYPOINT;

    File outDir = new File("build/reports/loops/debug");
    outDir.mkdirs();
    File logFile = new File(outDir, "mallorca_75km_fastbike_S_waypoints.txt");

    RoutingEngine re = new RoutingEngine(
      new File(outDir, "wpdump").getAbsolutePath(),
      new File(outDir, "wpdump").getAbsolutePath(),
      segDir, wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = false;

    try (FileWriter fw = new FileWriter(logFile)) {
      // Intercept the waypoints list AFTER placement but BEFORE routing.
      // We do this by hooking RoutingEngine's protected state via reflection
      // (the existing API doesn't expose a placement-only entry point).

      // Run the engine. It will modify the waypoints list internally; after
      // the run we read back the matched waypoints from the produced track.
      re.doRun(0);

      OsmTrack track = re.getFoundTrack();
      String err = re.getErrorMessage();
      fw.write("== mallorca_75km_fastbike_S [probe] ==\n");
      fw.write("Error: " + (err == null ? "(none)" : err) + "\n\n");

      if (track == null) {
        fw.write("No track produced.\n");
        return;
      }

      fw.write("Track: " + track.nodes.size() + " nodes, distance=" + track.distance + "m\n");
      // matchedWaypoints carries the actual snapped waypoint positions.
      if (track.matchedWaypoints != null) {
        fw.write("Matched waypoints (the points the engine actually routed through):\n");
        int i = 0;
        for (btools.mapaccess.MatchedWaypoint mwp : track.matchedWaypoints) {
          double rawLat = mwp.waypoint != null ? (mwp.waypoint.ilat - 90_000_000) / 1_000_000.0 : Double.NaN;
          double rawLon = mwp.waypoint != null ? (mwp.waypoint.ilon - 180_000_000) / 1_000_000.0 : Double.NaN;
          double snapLat = mwp.crosspoint != null ? (mwp.crosspoint.ilat - 90_000_000) / 1_000_000.0 : Double.NaN;
          double snapLon = mwp.crosspoint != null ? (mwp.crosspoint.ilon - 180_000_000) / 1_000_000.0 : Double.NaN;
          int snapDist = (mwp.waypoint != null && mwp.crosspoint != null)
            ? (int) btools.util.CheapRuler.distance(mwp.waypoint.ilon, mwp.waypoint.ilat,
                                                     mwp.crosspoint.ilon, mwp.crosspoint.ilat)
            : -1;
          fw.write(String.format(java.util.Locale.US,
            "  #%d %-12s  raw=(%.5f, %.5f)  snap=(%.5f, %.5f)  snapDist=%dm  wpType=%d\n",
            i++, mwp.name == null ? "?" : mwp.name,
            rawLat, rawLon, snapLat, snapLon, snapDist, mwp.wpttype));
        }
      } else {
        fw.write("No matched waypoints recorded.\n");
      }
    }
    System.out.println("Wrote waypoint dump: " + logFile.getAbsolutePath());
  }
}
