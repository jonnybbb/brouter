package btools.router;

import org.junit.Assume;
import org.junit.Test;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** SCRATCH: dig into WHY ISOCHRONE loses to WAYPOINT on specific cases.
 * Opt-in only — gated behind {@code -Dloop.tests=true} so it cannot
 * accidentally run in standard CI. */
public class IsochroneDiagnosticTest {

  @Test
  public void diagnose() throws Exception {
    Assume.assumeTrue(
      "Scratch diagnostic — opt in with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profile = new File(projectDir, "misc/profiles2/fastbike.brf");

    // cases where W>I previously (composite delta noted)
    int[][] cases = {
      {193400000, 142520000, 0,   30000, /*urban_berlin*/0},
      {193400000, 142520000, 90,  30000, /*urban_berlin*/0},
      {191400000, 137260000, 0,   30000, /*alpine_innsbruck*/0},
      {191400000, 137260000, 270, 30000, /*alpine_innsbruck*/0},
    };
    String[] names = {"berlin_30km_d0", "berlin_30km_d90", "alpine_30km_d0", "alpine_30km_d270"};

    for (int ci = 0; ci < cases.length; ci++) {
      int ilon = cases[ci][0], ilat = cases[ci][1], dir = cases[ci][2], targetL = cases[ci][3];
      int radius = (int) Math.round(targetL / (2 * Math.PI));
      System.out.println("\n##### " + names[ci] + " (r=" + radius + ") #####");
      LoopQualityMetrics w = run(segDir, profile, ilon, ilat, dir, radius, RoundTripAlgorithm.WAYPOINT, targetL, "W");
      LoopQualityMetrics i = run(segDir, profile, ilon, ilat, dir, radius, RoundTripAlgorithm.ISOCHRONE, targetL, "I");
      if (w == null || i == null) { System.out.println("  one failed"); continue; }
      System.out.println(String.format(Locale.US,
        "  WAYPOINT : comp=%.3f ratio=%.2f reuse=%.0f%% dirDelta=%.0f° cont=%.2f compact=%.2f close=%dm gap=%dm",
        w.compositeScore(), w.getDistanceRatio(), w.getRoadReusePercent(),
        w.getDirectionDeltaDegrees(), w.getContinuityScore(), w.getCompactnessScore(),
        w.getClosureDistanceMeters(), w.getMaxGapMeters()));
      System.out.println(String.format(Locale.US,
        "  ISOCHRONE: comp=%.3f ratio=%.2f reuse=%.0f%% dirDelta=%.0f° cont=%.2f compact=%.2f close=%dm gap=%dm",
        i.compositeScore(), i.getDistanceRatio(), i.getRoadReusePercent(),
        i.getDirectionDeltaDegrees(), i.getContinuityScore(), i.getCompactnessScore(),
        i.getClosureDistanceMeters(), i.getMaxGapMeters()));
      System.out.println("  delta (W-I) per dimension:");
      System.out.println(String.format(Locale.US,
        "    comp=%.3f  ratio=%.2f  reuse=%.0f%% dirDelta=%.0f°  cont=%.2f  compact=%.2f  close=%dm  gap=%dm",
        w.compositeScore() - i.compositeScore(),
        w.getDistanceRatio() - i.getDistanceRatio(),
        w.getRoadReusePercent() - i.getRoadReusePercent(),
        w.getDirectionDeltaDegrees() - i.getDirectionDeltaDegrees(),
        w.getContinuityScore() - i.getContinuityScore(),
        w.getCompactnessScore() - i.getCompactnessScore(),
        w.getClosureDistanceMeters() - i.getClosureDistanceMeters(),
        w.getMaxGapMeters() - i.getMaxGapMeters()));
    }
  }

  private LoopQualityMetrics run(File segDir, File profile, int ilon, int ilat, int dir,
                                 int radius, RoundTripAlgorithm algo, int targetL, String tag) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed s = new OsmNodeNamed(); s.name = "from"; s.ilon = ilon; s.ilat = ilat; wps.add(s);
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profile.getAbsolutePath();
    rc.startDirection = dir; rc.roundTripDistance = radius;
    rc.roundTripAlgorithm = algo;
    RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    try { re.doRun(0); } catch (Exception e) { return null; }
    OsmTrack t = re.getFoundTrack();
    if (t == null) return null;
    return LoopQualityMetrics.compute(t, targetL, dir);
  }
}
