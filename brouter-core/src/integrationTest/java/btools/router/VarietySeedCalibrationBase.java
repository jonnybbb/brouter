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
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Variety-seed calibration matrix (ADR-0001). Report-only: it never asserts
 * route quality — it MEASURES how {@code alternativeidx} seeds 0–4 change
 * round-trip generation, so {@link GreedyRoundTripPlanner#VARIETY_JITTER_AMPLITUDE}
 * can be tuned from evidence rather than feel. Two questions per case:
 * <ol>
 *   <li><b>Divergence</b> — how different is seed N's loop from seed 0's
 *       (1 − Jaccard over track node coordinates)? Near 0 means the seed is
 *       inert in that terrain; near 1 means a structurally different loop.</li>
 *   <li><b>Quality drift</b> — do seeded loops fail the gate (lenient Warning)
 *       or degrade (cost/m, distance ratio, RouteChoiceScore) vs seed 0?</li>
 * </ol>
 * Runs production AUTO, lenient (the shipped configuration), with a fixed
 * startDirection per case so divergence is attributable to the seed alone
 * (the direction-focus invariant keeps the heading constant by design).
 * Results land in {@code build/seed-calibration/<label>.csv}, one file per
 * case so parallel forks never contend; aggregate them after the run.
 */
@RunWith(Parameterized.class)
public abstract class VarietySeedCalibrationBase {

  protected static final int[] TARGET_DISTANCES = {30000, 50000};
  protected static final int[] SEARCH_RADII = {4800, 8000};
  protected static final String[] PROFILES = {"fastbike", "gravel"};
  protected static final double[] DIRECTIONS = {0, 90};
  protected static final String[] DIR_LABELS = {"N", "E"};
  protected static final int SEED_COUNT = 5;

  // 5 sequential AUTO plans per case; AUTO can run several candidates each.
  @Rule
  public org.junit.rules.Timeout perTestTimeout = org.junit.rules.Timeout.builder()
    .withTimeout(15, java.util.concurrent.TimeUnit.MINUTES)
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

  protected static Collection<Object[]> dataForRegion(LoopTestRegion region) {
    List<Object[]> params = new ArrayList<>();
    for (int i = 0; i < TARGET_DISTANCES.length; i++) {
      for (String profile : PROFILES) {
        for (int d = 0; d < DIRECTIONS.length; d++) {
          if (region.isSeaBlockedDirection(DIRECTIONS[d])) continue;
          String label = String.format(Locale.US, "%s_%dkm_%s_%s",
            region.name().toLowerCase(Locale.US), TARGET_DISTANCES[i] / 1000, profile, DIR_LABELS[d]);
          params.add(new Object[]{region, TARGET_DISTANCES[i], SEARCH_RADII[i], profile, DIRECTIONS[d], label});
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
  public void seedMatrix() throws Exception {
    // Opt-in: 160 AUTO plans are too heavy for every integrationTest run.
    // Run with: ./gradlew :brouter-core:integrationTest -Dseed.calibration=true \
    //   --tests '*VarietySeedCalibration*'
    Assume.assumeTrue("seed calibration is opt-in (-Dseed.calibration=true)",
      Boolean.getBoolean("seed.calibration"));
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
    Assume.assumeTrue("Profile not found: " + profileFile.getAbsolutePath(), profileFile.exists());
    Assume.assumeTrue("Profile " + profileName + " not supported for " + region.name(),
      region.supportedProfiles.contains(profileName));

    File outDir = new File(projectDir, "build/seed-calibration");
    //noinspection ResultOfMethodCallIgnored
    outDir.mkdirs();

    StringBuilder csv = new StringBuilder(
      "label,region,distKm,profile,dir,seed,formed,warned,trackDistM,distRatio,reusePct,dirDelta,costPerM,rcs,divVsSeed0,planMs,error\n");
    Set<Long> seed0Nodes = null;

    for (int seed = 0; seed < SEED_COUNT; seed++) {
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
        rctx.startDirection = (int) direction;
        rctx.roundTripDistance = searchRadius;
        rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
        // Production configuration: lenient, so a gate-failed loop still ships
        // (with a Warning in track.message) and we can measure its divergence.
        rctx.roundTripStrictQuality = false;
        rctx.setAlternativeIdx(seed);

        String outPath = new File(outputDir.getRoot(), testLabel + "_s" + seed).getAbsolutePath();
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
      String base = String.format(Locale.US, "%s,%s,%d,%s,%.0f,%d",
        testLabel, region.name(), targetDistanceMeters / 1000, profileName, direction, seed);
      if (formed) {
        boolean warned = track.message != null && track.message.contains("Warning:");
        Set<Long> nodes = nodeSet(track);
        String div = "";
        if (seed == 0) {
          seed0Nodes = nodes;
          div = "0.0000";
        } else if (seed0Nodes != null) {
          div = String.format(Locale.US, "%.4f", 1.0 - jaccard(seed0Nodes, nodes));
        }
        LoopQualityMetrics m = LoopQualityMetrics.compute(track, targetDistanceMeters, direction);
        double rcs = RouteChoiceScore.score(track, targetDistanceMeters, profileName, null, direction).qualityScore();
        csv.append(String.format(Locale.US, "%s,1,%d,%d,%.3f,%.1f,%.1f,%.3f,%.3f,%s,%d,%n",
          base, warned ? 1 : 0, track.distance, m.getDistanceRatio(), m.getRoadReusePercent(),
          m.getDirectionDeltaDegrees(), m.getAverageCostPerMeter(), rcs, div, planMs));
      } else {
        String safeError = error == null ? "no track" : error.replace(',', ';').replace('\n', ' ');
        csv.append(String.format(Locale.US, "%s,0,0,0,,,,,,,%d,%s%n", base, planMs, safeError));
      }
      System.out.println("SEEDCAL " + base + (formed ? " formed" : " FAILED(" + error + ")")
        + " in " + planMs + "ms");
    }

    File out = new File(outDir, testLabel + ".csv");
    try (FileWriter fw = new FileWriter(out)) {
      fw.write(csv.toString());
    }
  }

  /** Track node coordinates packed to longs — the identity set for divergence. */
  private static Set<Long> nodeSet(OsmTrack track) {
    Set<Long> set = new HashSet<>();
    for (OsmPathElement n : track.nodes) {
      set.add(((long) n.getILon() << 32) | (n.getILat() & 0xFFFFFFFFL));
    }
    return set;
  }

  private static double jaccard(Set<Long> a, Set<Long> b) {
    if (a.isEmpty() && b.isEmpty()) return 1.0;
    int inter = 0;
    for (Long x : a) {
      if (b.contains(x)) inter++;
    }
    return inter / (double) (a.size() + b.size() - inter);
  }
}
