package btools.router.known;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoundTripAlgorithm;
import btools.router.RoundTripQualityGate;
import btools.router.RoundTripQualityGate.HostileStretch;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

import org.junit.Assume;
import org.junit.Test;

/**
 * Multi-variant investigation of {@code alpine_innsbruck_100km_fastbike_N}.
 * Komoot proves clean 90-105km fastbike loops exist in this area (the
 * "Notburga-Steg — Reintaler See" 105km tour, "Schlitterer See —
 * Innradweg" 93km tour). Brouter rejects every variant. This test
 * surfaces the failure mode for each variant: distance, self-crossings,
 * hostile-stretch, etc.
 */
public class AlpineInnsbruck100kmInvestigation {

  private File profileFile(String name) {
    File p = new File("misc/profiles2/" + name + ".brf");
    if (!p.exists()) p = new File("../misc/profiles2/" + name + ".brf");
    return p.exists() ? p : null;
  }

  @Test
  public void inspectAllVariants() throws Exception {
    Assume.assumeTrue("loop-tests opt-in", Boolean.getBoolean("loop.tests"));
    File segDir = new File("../segments4");
    if (!segDir.exists()) segDir = new File("segments4");
    File segFile = new File(segDir, "E10_N45.rd5");
    Assume.assumeTrue("segment present at " + segFile.getAbsolutePath(), segFile.exists());
    File pf = profileFile("fastbike");
    Assume.assumeTrue("profile present", pf != null);

    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
      RoundTripAlgorithm.WAYPOINT, RoundTripAlgorithm.ISOCHRONE,
      RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      List<OsmNodeNamed> wplist = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from";
      start.ilon = 191461700; // 11.4617 E
      start.ilat = 137282800; // 47.2828 N (Innsbruck)
      wplist.add(start);

      RoutingContext rctx = new RoutingContext();
      rctx.localFunction = pf.getAbsolutePath();
      rctx.startDirection = 0; // N
      rctx.roundTripDistance = 15900; // 100km loop
      rctx.roundTripAlgorithm = algo;

      RoutingEngine re = new RoutingEngine(null, null, segDir, wplist, rctx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);

      String err = re.getErrorMessage();
      OsmTrack track = re.getFoundTrack();
      OsmTrack rejected = re.getLastRejectedTrack();
      OsmTrack inspect = track != null ? track : rejected;

      System.out.println("\n=== " + algo + " ===");
      System.out.println("  error: " + (err == null ? "PASSED" : err));
      if (track != null) {
        System.out.println("  track: " + track.distance + "m / " + track.nodes.size() + " nodes");
      }
      if (rejected != null) {
        HostileStretch hs = RoundTripQualityGate.worstHostileStretchPaved(rejected);
        System.out.println("  rejected-track: " + rejected.distance + "m / " + rejected.nodes.size() + " nodes");
        if (hs.isPresent()) {
          System.out.printf("  worst-hostile: %dm at (%.6f, %.6f) tags: %s%n",
            hs.meters, hs.startLat(), hs.startLon(), hs.startTags);
        }
      }
    }
  }
}
