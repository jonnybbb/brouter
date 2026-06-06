package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assume;
import org.junit.Test;

/**
 * Local generator (uncommitted-style): produce a ~50 km GRAVEL round-trip loop
 * starting in central Basel and heading NORTH-EAST (toward the Dinkelberg /
 * southern Black Forest), and write it to {@code tmp/basel_gravel_ne_50k.gpx}.
 *
 * Run: {@code ./gradlew :brouter-core:integrationTest --tests BaselGravelLoopGpxTest -Dloop.segments.noupdate=true}
 */
public class BaselGravelLoopGpxTest {

  @org.junit.Rule
  public org.junit.rules.Timeout t = org.junit.rules.Timeout.builder()
    .withTimeout(15, java.util.concurrent.TimeUnit.MINUTES).build();

  // Central Basel (near Claraplatz / Kleinbasel), lon/lat. Overridable via -Dloop.* .
  private static final double START_LON = Double.parseDouble(System.getProperty("loop.lon", "7.5926"));
  private static final double START_LAT = Double.parseDouble(System.getProperty("loop.lat", "47.5650"));
  private static final double LOOP_M = Double.parseDouble(System.getProperty("loop.km", "50")) * 1000.0;
  private static final int DIR_NE = Integer.parseInt(System.getProperty("loop.dir", "45"));
  private static final String NAME = System.getProperty("loop.name", "basel_gravel_ne_50k");

  @Test
  public void generateBaselGravelNeLoop() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    String profileName = System.getProperty("loop.profile", "gravel");
    File profileFile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
    Assume.assumeTrue("segments missing", segDir.isDirectory());
    Assume.assumeTrue("gravel profile missing", profileFile.exists());

    File outDir = new File(projectDir, "tmp");
    outDir.mkdirs();
    String outBase = new File(outDir, NAME).getAbsolutePath();

    List<OsmNodeNamed> wl = new ArrayList<>();
    OsmNodeNamed s = new OsmNodeNamed();
    s.name = "from";
    s.ilon = (int) Math.round((START_LON + 180) * 1e6);
    s.ilat = (int) Math.round((START_LAT + 90) * 1e6);
    wl.add(s);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = DIR_NE;
    // codebase convention: roundTripDistance is the loop "radius" = circumference / 2π
    rctx.roundTripDistance = (int) Math.round(LOOP_M / (2 * Math.PI));
    rctx.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;

    RoutingEngine re = new RoutingEngine(
      outBase, null, segDir, wl, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);

    OsmTrack tr = re.getFoundTrack();
    if (tr == null) tr = re.getLastRejectedTrack();
    if (tr == null) {
      System.out.println("BASEL-LOOP: no track produced (error=" + re.getErrorMessage() + ")");
      return;
    }
    File gpxFile = new File(outDir, NAME + ".gpx");
    String gpx = new FormatGpx(rctx).format(tr);
    try (java.io.FileWriter fw = new java.io.FileWriter(gpxFile)) { fw.write(gpx); }

    System.out.println(String.format(
      "BASEL-LOOP: distKm=%.1f nodes=%d gpx=%s (%d bytes) err=%s",
      tr.distance / 1000.0, tr.nodes == null ? 0 : tr.nodes.size(),
      gpxFile.getAbsolutePath(), gpxFile.length(), re.getErrorMessage()));
  }
}
