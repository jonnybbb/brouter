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
 * Focused investigation of {@code dreieich_50km_fastbike_W} — the
 * scenario where greedy still rejects after Phase 2 v4 but iso_greedy
 * passes. Outputs the WHERE of the 2290m hostile stretch so we can
 * inspect it on OSM and decide if the road network has an alternative.
 *
 * Run with: {@code -Dloop.tests=true --tests
 * "btools.router.known.DreiEichWFastbikeInvestigation"}
 */
public class DreiEichWFastbikeInvestigation {

  private File profileFile(String name) {
    File p = new File("misc/profiles2/" + name + ".brf");
    if (!p.exists()) p = new File("../misc/profiles2/" + name + ".brf");
    return p.exists() ? p : null;
  }

  @Test
  public void locateHostileStretch() throws Exception {
    Assume.assumeTrue("loop-tests opt-in", Boolean.getBoolean("loop.tests"));
    File segDir = new File("../segments4");
    if (!segDir.exists()) segDir = new File("segments4");
    File segFile = new File(segDir, "E10_N50.rd5");
    Assume.assumeTrue("segment present at " + segFile.getAbsolutePath(), segFile.exists());
    File pf = profileFile("fastbike");
    Assume.assumeTrue("profile present", pf != null);

    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      List<OsmNodeNamed> wplist = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from";
      start.ilon = 188661500;
      start.ilat = 140042500;
      wplist.add(start);

      RoutingContext rctx = new RoutingContext();
      rctx.localFunction = pf.getAbsolutePath();
      rctx.startDirection = 270;
      rctx.roundTripDistance = 8000;
      rctx.roundTripAlgorithm = algo;

      RoutingEngine re = new RoutingEngine(null, null, segDir, wplist, rctx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);

      String err = re.getErrorMessage();
      OsmTrack track = re.getFoundTrack();
      OsmTrack rejected = re.getLastRejectedTrack();
      OsmTrack inspectMe = track != null ? track : rejected;

      System.out.println("\n=== " + algo + " ===");
      System.out.println("  error: " + (err == null ? "none (accepted)" : err));
      System.out.println("  track: " + (track == null ? "null" : track.distance + "m / " + track.nodes.size() + " nodes"));
      System.out.println("  rejected: " + (rejected == null ? "null" : rejected.distance + "m / " + rejected.nodes.size() + " nodes"));
      if (inspectMe != null && inspectMe.nodes != null) {
        HostileStretch hs = RoundTripQualityGate.worstHostileStretchPaved(inspectMe);
        if (hs.isPresent()) {
          System.out.printf("  worst-hostile: %dm starting at (%.6f, %.6f) ending at (%.6f, %.6f)%n",
            hs.meters, hs.startLat(), hs.startLon(), hs.endLat(), hs.endLon());
          System.out.println("  startTags: " + hs.startTags);
          System.out.println("  endTags:   " + hs.endTags);
          System.out.printf("  OSM link: https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=15/%.5f/%.5f%n",
            hs.startLat(), hs.startLon(), hs.startLat(), hs.startLon());
        } else {
          System.out.println("  no hostile stretch detected");
        }
      }
    }
  }
}
