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
 * Report-only probe for loop-review backlog item 4 (sub-600m detection
 * floor). Measures, on FULL-RESOLUTION shipped tracks (the report cache's
 * Douglas-Peucker coords are unusable — decimation fabricates/hides
 * near-revisit pairs, cf. the 100km-formation-skips incident), how many
 * additional near-revisit spans the teardrop detector would flag if its
 * MIN_ARC floor were lowered from 600m to 300m or 100m.
 *
 * <p>The hypothesis under test: sub-600m spans are dominated by legitimate
 * geometry — alpine switchbacks return within the 60m proximity epsilon at
 * 100-500m arc BY CONSTRUCTION — so lowering the floor inflates the flag
 * rate with terrain-forced false positives (the locked spur-detector
 * decision: "do NOT keep geometric-threshold tuning"). Two switchback-rich
 * regions (Innsbruck, Garmisch) vs two flat-dense ones (Freiburg, Berlin)
 * give the alpine-skew signal.
 *
 * <p>Opt-in via {@code -Dfloor.probe=true}; results land in
 * {@code build/floor-probe/<label>.csv}.
 */
@RunWith(Parameterized.class)
public class SubArcFloorProbeTest {

  private static final LoopTestRegion[] REGIONS = {
    LoopTestRegion.ALPINE_INNSBRUCK, LoopTestRegion.GARMISCH,
    LoopTestRegion.FREIBURG, LoopTestRegion.URBAN_BERLIN,
  };
  private static final int[] TARGET_DISTANCES = {30000, 50000};
  private static final int[] SEARCH_RADII = {4800, 8000};
  private static final String[] PROFILES = {"fastbike", "gravel"};
  private static final double[] DIRECTIONS = {0, 90};
  /** Candidate floors to measure, vs the locked 600m production floor. */
  private static final int[] CANDIDATE_FLOORS = {100, 300};

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
  public double direction;
  @Parameterized.Parameter(5)
  public String testLabel;

  @Rule
  public TemporaryFolder outputDir = new TemporaryFolder();

  private File projectDir;

  @Parameterized.Parameters(name = "{5}")
  public static Collection<Object[]> data() {
    List<Object[]> params = new ArrayList<>();
    for (LoopTestRegion region : REGIONS) {
      for (int i = 0; i < TARGET_DISTANCES.length; i++) {
        for (String profile : PROFILES) {
          for (double dir : DIRECTIONS) {
            String label = String.format(Locale.US, "%s_%dkm_%s_d%.0f",
              region.name().toLowerCase(Locale.US), TARGET_DISTANCES[i] / 1000, profile, dir);
            params.add(new Object[]{region, TARGET_DISTANCES[i], SEARCH_RADII[i], profile, dir, label});
          }
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
  public void floorProbe() throws Exception {
    Assume.assumeTrue("floor probe is opt-in (-Dfloor.probe=true)",
      Boolean.getBoolean("floor.probe"));
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
    Assume.assumeTrue("Profile not found: " + profileFile.getAbsolutePath(), profileFile.exists());
    Assume.assumeTrue("Profile " + profileName + " not supported for " + region.name(),
      region.supportedProfiles.contains(profileName));

    File outDir = new File(projectDir, "build/floor-probe");
    //noinspection ResultOfMethodCallIgnored
    outDir.mkdirs();

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = region.ilon;
    start.ilat = region.ilat;
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = (int) direction;
    rctx.roundTripDistance = searchRadius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    rctx.roundTripStrictQuality = false;

    String outPath = new File(outputDir.getRoot(), testLabel).getAbsolutePath();
    RoutingEngine re = new RoutingEngine(outPath, outPath, segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    OsmTrack track = re.getFoundTrack();

    StringBuilder csv = new StringBuilder(
      "label,region,distKm,profile,dir,formed,trackDistM,floorM,bandSpans,bandArcM\n");
    String base = String.format(Locale.US, "%s,%s,%d,%s,%.0f",
      testLabel, region.name(), targetDistanceMeters / 1000, profileName, direction);
    if (re.getErrorMessage() == null && track != null && track.nodes != null && track.nodes.size() >= 4) {
      // Band counts: [floor, nextFloor) so each row isolates what ONLY that
      // floor reduction would newly flag; 600 row = today's production band.
      int[] floors = {CANDIDATE_FLOORS[0], CANDIDATE_FLOORS[1], 600};
      int[] uppers = {CANDIDATE_FLOORS[1], 600, 10000};
      for (int i = 0; i < floors.length; i++) {
        int spans = 0;
        long arcSum = 0;
        List<OsmPathElement> nodes = track.nodes;
        double[] cum = new double[nodes.size()];
        for (int k = 1; k < nodes.size(); k++) {
          cum[k] = cum[k - 1] + nodes.get(k - 1).calcDistance(nodes.get(k));
        }
        for (int[] s : LoopQualityMetrics.nearRevisitSpans(nodes, 60.0, floors[i], uppers[i])) {
          double arc = cum[s[1]] - cum[s[0]];
          if (cum[nodes.size() - 1] > 0 && arc > 0.85 * cum[nodes.size() - 1]) {
            continue; // the loop's own closure, mirroring teardropSeverity()
          }
          spans++;
          arcSum += Math.round(arc);
        }
        csv.append(String.format(Locale.US, "%s,1,%d,%d,%d,%d%n",
          base, track.distance, floors[i], spans, arcSum));
      }
    } else {
      csv.append(String.format(Locale.US, "%s,0,0,,,%n", base));
    }
    File out = new File(outDir, testLabel + ".csv");
    try (FileWriter fw = new FileWriter(out)) {
      fw.write(csv.toString());
    }
    System.out.println("FLOORPROBE " + testLabel + " done");
  }
}
