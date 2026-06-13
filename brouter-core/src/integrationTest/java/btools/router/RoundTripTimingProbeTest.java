package btools.router;

import org.junit.Assume;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Locale;

/**
 * Report-only wall-clock probe for round-trip generation: production AUTO
 * (lenient), seed 0, across the full distance band (30–100 km) and all three
 * bike profiles, in one flat (FREIBURG) and one urban (URBAN_BERLIN) region.
 * Never asserts quality — it measures how long generation takes so latency
 * expectations rest on numbers, not anecdotes. mtb is included even where the
 * loop-quality suite excludes it: a timing answer is useful regardless of
 * whether the resulting loop clears the quality bars.
 *
 * <p>Opt-in via {@code -Dtiming.probe=true}; results land in
 * {@code build/seed-calibration-timing/<label>.csv}.
 */
@RunWith(Parameterized.class)
public class RoundTripTimingProbeTest {

  private static final LoopTestRegion[] REGIONS = {LoopTestRegion.FREIBURG, LoopTestRegion.URBAN_BERLIN};
  private static final int[] TARGET_DISTANCES = {30000, 50000, 75000, 100000};
  private static final int[] SEARCH_RADII = {4800, 8000, 11937, 15900};
  private static final String[] PROFILES = {"fastbike", "gravel", "mtb"};
  private static final double DIRECTION = 90;

  @Rule
  public org.junit.rules.Timeout perTestTimeout = org.junit.rules.Timeout.builder()
    .withTimeout(10, java.util.concurrent.TimeUnit.MINUTES)
    .withLookingForStuckThread(true)
    .build();

  @Parameterized.Parameter(0)
  public LoopTestRegion region;
  @Parameterized.Parameter(1)
  public int targetDistanceMeters;
  @Parameterized.Parameter(2)
  public int searchRadius;
  @Parameterized.Parameter(3)
  public String profileName;
  @Parameterized.Parameter(4)
  public String testLabel;

  @Rule
  public TemporaryFolder outputDir = new TemporaryFolder();

  private File projectDir;

  @Parameterized.Parameters(name = "{4}")
  public static Collection<Object[]> data() {
    List<Object[]> params = new ArrayList<>();
    for (LoopTestRegion region : REGIONS) {
      for (int i = 0; i < TARGET_DISTANCES.length; i++) {
        for (String profile : PROFILES) {
          String label = String.format(Locale.US, "%s_%dkm_%s",
            region.name().toLowerCase(Locale.US), TARGET_DISTANCES[i] / 1000, profile);
          params.add(new Object[]{region, TARGET_DISTANCES[i], SEARCH_RADII[i], profile, label});
        }
      }
    }
    return params;
  }

  @Before
  public void setUp() throws Exception {
    projectDir = new File(".").getCanonicalFile().getParentFile();
  }

  @Test
  public void timingProbe() throws Exception {
    Assume.assumeTrue("timing probe is opt-in (-Dtiming.probe=true)",
      Boolean.getBoolean("timing.probe"));
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
    Assume.assumeTrue("Profile not found: " + profileFile.getAbsolutePath(), profileFile.exists());

    File outDir = new File(projectDir, "build/seed-calibration-timing");
    //noinspection ResultOfMethodCallIgnored
    outDir.mkdirs();

    long t0 = System.currentTimeMillis();
    OsmTrack track = null;
    String error = null;
    try {
      List<OsmNodeNamed> wplist = new ArrayList<>();
      OsmNodeNamed start = new OsmNodeNamed();
      start.name = "from";
      start.ilon = region.ilon;
      start.ilat = region.ilat;
      wplist.add(start);

      RoutingContext rctx = new RoutingContext();
      rctx.localFunction = profileFile.getAbsolutePath();
      rctx.startDirection = (int) DIRECTION;
      rctx.roundTripDistance = searchRadius;
      rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
      rctx.roundTripStrictQuality = false;

      String outPath = new File(outputDir.getRoot(), testLabel).getAbsolutePath();
      RoutingEngine re = new RoutingEngine(outPath, outPath, segDir, wplist, rctx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.doRun(0);
      error = re.getErrorMessage();
      track = re.getFoundTrack();
    } catch (Exception e) {
      error = e.getClass().getSimpleName() + ": " + e.getMessage();
    }
    long planMs = System.currentTimeMillis() - t0;

    boolean formed = error == null && track != null && track.nodes != null && !track.nodes.isEmpty();
    boolean warned = formed && track.message != null && track.message.contains("Warning:");
    String safeError = error == null ? "" : error.replace(',', ';').replace('\n', ' ');
    String row = String.format(Locale.US, "%s,%s,%d,%s,%d,%d,%d,%d,%s%n",
      testLabel, region.name(), targetDistanceMeters / 1000, profileName,
      formed ? 1 : 0, warned ? 1 : 0, formed ? track.distance : 0, planMs, safeError);
    System.out.println("TIMING " + row.trim());

    File out = new File(outDir, testLabel + ".csv");
    try (FileWriter fw = new FileWriter(out)) {
      fw.write("label,region,distKm,profile,formed,warned,trackDistM,planMs,error\n");
      fw.write(row);
    }
  }
}
