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

import btools.mapaccess.MatchedWaypoint;

/**
 * Near-revisit FEATURE probe (follow-up to ADR-0002): emits, per detected
 * near-revisit span, the feature vector that separates real defects from
 * legitimate geometry — so the next labeling pass works on features instead
 * of raw arc/proximity thresholds (which ADR-0002 measured to be
 * non-discriminating). Report-only: routes are never modified.
 *
 * <p>Features per span (see the loop-review discussion):
 * <ol>
 *   <li><b>retraceFrac</b> — back-arm length fraction on out-arm nodes:
 *       ~1 = antenna (true out-and-back), ~0 = teardrop/lobe.</li>
 *   <li><b>armSepMed/MaxM</b> — arm-separation profile: thin along the whole
 *       arc = antenna fingerprint; fat middle = legitimate petal
 *       (Blumberg, 1511m, was a labeled negative).</li>
 *   <li><b>netElevM / grossAscendM</b> — switchback exoneration: a hairpin
 *       stack revisits itself while climbing; a pointless antenna is flat.</li>
 *   <li><b>costRatio</b> — span cost/m over track cost/m: ≥1.2 was the
 *       junk-road-pocket fingerprint of the wide-mouth-bulge repair.</li>
 *   <li><b>tipToGenViaM</b> — tip at a generated via = placement artifact
 *       (Ettingen's tip WAS via3), routing the finding to the placement fix.</li>
 *   <li><b>avoidability</b> — the decisive, graph-based feature: route
 *       mouth→tip as a plain request at alternativeidx 0 (natural arm) and 1
 *       (arm penalized — the classic alternative mechanism IS a
 *       corridor-poisoned re-route). An alternative arm that exists, barely
 *       retraces, and costs ≤2× the natural arm ⇒ the span was avoidable.</li>
 * </ol>
 *
 * <p>Opt-in via {@code -Dfeature.probe=true}; one CSV per case in
 * {@code build/feature-probe/} (fork-safe). minArc 300 so the borderline
 * [300,600) band is covered too; filter rows by arcM as needed.
 */
@RunWith(Parameterized.class)
public class NearRevisitFeatureProbeTest {

  private static final LoopTestRegion[] REGIONS = {
    LoopTestRegion.ALPINE_INNSBRUCK, LoopTestRegion.GARMISCH,
    LoopTestRegion.FREIBURG, LoopTestRegion.URBAN_BERLIN,
  };
  private static final int[] TARGET_DISTANCES = {30000, 50000};
  private static final int[] SEARCH_RADII = {4800, 8000};
  private static final String[] PROFILES = {"fastbike", "gravel"};
  private static final double[] DIRECTIONS = {0, 90};
  private static final double MIN_ARC_M = 300;
  private static final double MAX_ARC_M = 10000;
  /** Cost control: avoidability spends 2 routed requests per span. */
  private static final int MAX_AVOIDABILITY_SPANS_PER_ROUTE = 3;
  /** Arm subsampling cap for the O(out×back) separation scan. */
  private static final int ARM_SAMPLE_CAP = 200;

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
  private File segDir;
  private File profileFile;

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
    segDir = new File(projectDir, "segments4");
    profileFile = new File(projectDir, "misc/profiles2/" + profileName + ".brf");
  }

  @Test
  public void featureProbe() throws Exception {
    Assume.assumeTrue("feature probe is opt-in (-Dfeature.probe=true)",
      Boolean.getBoolean("feature.probe"));
    LoopTestSegments.ensureRegion(segDir, region);
    Assume.assumeTrue("Profile not found: " + profileFile.getAbsolutePath(), profileFile.exists());
    Assume.assumeTrue("Profile " + profileName + " not supported for " + region.name(),
      region.supportedProfiles.contains(profileName));

    File outDir = new File(projectDir, "build/feature-probe");
    //noinspection ResultOfMethodCallIgnored
    outDir.mkdirs();

    // The loop under inspection: production AUTO, lenient, seed 0.
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
    RoutingEngine re = new RoutingEngine(
      new File(outputDir.getRoot(), testLabel).getAbsolutePath(),
      new File(outputDir.getRoot(), testLabel).getAbsolutePath(),
      segDir, wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    OsmTrack track = re.getFoundTrack();

    StringBuilder csv = new StringBuilder("label,region,distKm,profile,dir,spanIdx,arcM,"
      + "retraceFrac,armSepMedM,armSepMaxM,netElevM,grossAscendM,costRatio,tipToGenViaM,"
      + "arm0DistM,alt1DistM,alt1RetraceFrac,avoidable\n");
    String base = String.format(Locale.US, "%s,%s,%d,%s,%.0f",
      testLabel, region.name(), targetDistanceMeters / 1000, profileName, direction);

    if (re.getErrorMessage() == null && track != null && track.nodes != null && track.nodes.size() >= 4) {
      List<OsmPathElement> nodes = track.nodes;
      double[] cum = new double[nodes.size()];
      for (int k = 1; k < nodes.size(); k++) {
        cum[k] = cum[k - 1] + nodes.get(k - 1).calcDistance(nodes.get(k));
      }
      double perim = cum[nodes.size() - 1];
      double trackCostPerM = track.distance > 0 ? track.cost / (double) track.distance : 0;

      int spanIdx = 0;
      List<int[]> keptSpans = new ArrayList<>();
      List<String> spanRows = new ArrayList<>();
      for (int[] s : LoopQualityMetrics.nearRevisitSpans(nodes, 60.0, MIN_ARC_M, MAX_ARC_M)) {
        double arc = cum[s[1]] - cum[s[0]];
        if (perim > 0 && arc > 0.85 * perim) {
          continue; // the loop's own closure
        }
        spanIdx++;
        String row = spanFeatures(base, spanIdx, nodes, cum, s[0], s[1], track, trackCostPerM);
        csv.append(row);
        keptSpans.add(s);
        spanRows.add(row.trim());
      }
      if (!keptSpans.isEmpty()) {
        writeSpanGeoJson(new File(outDir, testLabel + ".geojson"), nodes, cum, keptSpans, spanRows);
      }
    }
    File out = new File(outDir, testLabel + ".csv");
    try (FileWriter fw = new FileWriter(out)) {
      fw.write(csv.toString());
    }
    System.out.println("FEATUREPROBE " + testLabel + " done");
  }

  /**
   * Visual-labeling artifact: the full loop (subsampled) plus, per span, the
   * out arm, back arm and tip with the feature row attached as properties —
   * rendered by build/feature-probe/index.html (generated by the aggregation
   * step) so spans can be labeled defect/forced on the map in seconds.
   */
  private void writeSpanGeoJson(File file, List<OsmPathElement> nodes, double[] cum,
                                List<int[]> spans, List<String> rows) throws Exception {
    StringBuilder gj = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
    gj.append("{\"type\":\"Feature\",\"properties\":{\"role\":\"route\"},\"geometry\":");
    appendLineString(gj, nodes, 0, nodes.size() - 1, 4000);
    gj.append("}");
    for (int k = 0; k < spans.size(); k++) {
      int i0 = spans.get(k)[0];
      int i1 = spans.get(k)[1];
      int tipIdx = tipIndex(nodes, i0, i1);
      String props = "\"span\":" + (k + 1) + ",\"csv\":\"" + rows.get(k).replace("\"", "") + "\"";
      gj.append(",{\"type\":\"Feature\",\"properties\":{\"role\":\"outArm\",").append(props).append("},\"geometry\":");
      appendLineString(gj, nodes, i0, tipIdx, 2000);
      gj.append("},{\"type\":\"Feature\",\"properties\":{\"role\":\"backArm\",").append(props).append("},\"geometry\":");
      appendLineString(gj, nodes, tipIdx, i1, 2000);
      gj.append("},{\"type\":\"Feature\",\"properties\":{\"role\":\"tip\",").append(props).append("},\"geometry\":")
        .append("{\"type\":\"Point\",\"coordinates\":[")
        .append(lon(nodes.get(tipIdx))).append(",").append(lat(nodes.get(tipIdx))).append("]}}");
    }
    gj.append("]}");
    try (FileWriter fw = new FileWriter(file)) {
      fw.write(gj.toString());
    }
  }

  private static void appendLineString(StringBuilder gj, List<OsmPathElement> nodes,
                                       int from, int to, int maxPts) {
    gj.append("{\"type\":\"LineString\",\"coordinates\":[");
    int n = to - from + 1;
    int stride = Math.max(1, n / maxPts);
    boolean first = true;
    for (int k = from; k <= to; k += stride) {
      if (!first) gj.append(",");
      first = false;
      gj.append("[").append(lon(nodes.get(k))).append(",").append(lat(nodes.get(k))).append("]");
    }
    if ((to - from) % stride != 0) {
      gj.append(",[").append(lon(nodes.get(to))).append(",").append(lat(nodes.get(to))).append("]");
    }
    gj.append("]}");
  }

  private static double lon(OsmPathElement e) {
    return e.getILon() / 1e6 - 180.0;
  }

  private static double lat(OsmPathElement e) {
    return e.getILat() / 1e6 - 90.0;
  }

  private static int tipIndex(List<OsmPathElement> nodes, int i0, int i1) {
    OsmPathElement mouth = nodes.get(i0);
    int tipIdx = i0;
    double best = -1;
    for (int k = i0; k <= i1; k++) {
      double d = mouth.calcDistance(nodes.get(k));
      if (d > best) {
        best = d;
        tipIdx = k;
      }
    }
    return tipIdx;
  }

  private String spanFeatures(String base, int spanIdx, List<OsmPathElement> nodes,
                              double[] cum, int i0, int i1, OsmTrack track, double trackCostPerM) {
    double arc = cum[i1] - cum[i0];
    OsmPathElement mouth = nodes.get(i0);

    // Tip = span node farthest (airline) from the mouth.
    int tipIdx = i0;
    double tipDist = -1;
    for (int k = i0; k <= i1; k++) {
      double d = mouth.calcDistance(nodes.get(k));
      if (d > tipDist) {
        tipDist = d;
        tipIdx = k;
      }
    }
    OsmPathElement tip = nodes.get(tipIdx);

    // Out arm [i0..tip], back arm [tip..i1].
    List<OsmPathElement> outArm = new ArrayList<>(nodes.subList(i0, tipIdx + 1));
    List<OsmPathElement> backArm = new ArrayList<>(nodes.subList(tipIdx, i1 + 1));
    OsmTrack outArmTrack = new OsmTrack();
    outArmTrack.nodes.addAll(outArm);
    outArmTrack.buildMap();
    OsmTrack backArmTrack = new OsmTrack();
    backArmTrack.nodes.addAll(backArm);
    double retraceFrac = GreedyRoundTripPlanner.reuseFraction(backArmTrack, outArmTrack);

    // Arm separation profile: nearest out-arm node per back-arm node.
    List<OsmPathElement> outS = subsample(outArm);
    List<OsmPathElement> backS = subsample(backArm);
    List<Double> seps = new ArrayList<>();
    for (OsmPathElement b : backS) {
      double best = Double.MAX_VALUE;
      for (OsmPathElement o : outS) {
        double d = b.calcDistance(o);
        if (d < best) best = d;
      }
      if (best < Double.MAX_VALUE) seps.add(best);
    }
    seps.sort(null);
    double sepMed = seps.isEmpty() ? -1 : seps.get(seps.size() / 2);
    double sepMax = seps.isEmpty() ? -1 : seps.get(seps.size() - 1);

    // Elevation: net mouth→tip + gross ascend over the span.
    double netElev = Math.abs(tip.getElev() - mouth.getElev());
    double gross = 0;
    for (int k = i0 + 1; k <= i1; k++) {
      double d = nodes.get(k).getElev() - nodes.get(k - 1).getElev();
      if (d > 0) gross += d;
    }

    // Span cost/m vs track cost/m (cumulative per-node costs).
    double spanCost = nodes.get(i1).cost - nodes.get(i0).cost;
    double costRatio = (arc > 0 && trackCostPerM > 0) ? (spanCost / arc) / trackCostPerM : -1;

    // Tip proximity to a generated via (placement-artifact signal).
    double tipToVia = -1;
    if (track.matchedWaypoints != null) {
      for (MatchedWaypoint mwp : track.matchedWaypoints) {
        if (!mwp.generated || mwp.crosspoint == null) continue;
        double d = tip.calcDistance(mwp.crosspoint);
        if (tipToVia < 0 || d < tipToVia) tipToVia = d;
      }
    }

    // Avoidability: mouth→tip at alternativeidx 0 (natural arm) and 1
    // (arm-penalized alternative). Bounded per route for cost control.
    String arm0Dist = "", alt1Dist = "", alt1Retrace = "", avoidable = "";
    if (spanIdx <= MAX_AVOIDABILITY_SPANS_PER_ROUTE) {
      OsmTrack arm0 = routePlain(mouth, tip, 0);
      OsmTrack alt1 = routePlain(mouth, tip, 1);
      if (arm0 != null) arm0Dist = String.valueOf(arm0.distance);
      if (alt1 != null) {
        alt1Dist = String.valueOf(alt1.distance);
        double af = GreedyRoundTripPlanner.reuseFraction(alt1, outArmTrack);
        alt1Retrace = String.format(Locale.US, "%.2f", af);
        long armRef = arm0 != null ? arm0.distance : Math.round(cum[tipIdx] - cum[i0]);
        avoidable = (af < 0.3 && armRef > 0 && alt1.distance <= 2L * armRef) ? "1" : "0";
      }
    }

    return String.format(Locale.US, "%s,%d,%.0f,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f,%.0f,%s,%s,%s,%s%n",
      base, spanIdx, arc, retraceFrac, sepMed, sepMax, netElev, gross, costRatio, tipToVia,
      arm0Dist, alt1Dist, alt1Retrace, avoidable);
  }

  /** Plain point-to-point request used by the avoidability feature. */
  private OsmTrack routePlain(OsmPathElement from, OsmPathElement to, int alternativeIdx) {
    try {
      List<OsmNodeNamed> wps = new ArrayList<>();
      OsmNodeNamed a = new OsmNodeNamed();
      a.name = "from";
      a.ilon = from.getILon();
      a.ilat = from.getILat();
      wps.add(a);
      OsmNodeNamed b = new OsmNodeNamed();
      b.name = "to";
      b.ilon = to.getILon();
      b.ilat = to.getILat();
      wps.add(b);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profileFile.getAbsolutePath();
      rc.setAlternativeIdx(alternativeIdx);
      RoutingEngine eng = new RoutingEngine(null, null, segDir, wps, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUTING);
      eng.quite = true;
      eng.doRun(60_000);
      return eng.getErrorMessage() == null ? eng.getFoundTrack() : null;
    } catch (Exception e) {
      return null;
    }
  }

  private static List<OsmPathElement> subsample(List<OsmPathElement> arm) {
    if (arm.size() <= ARM_SAMPLE_CAP) return arm;
    List<OsmPathElement> out = new ArrayList<>(ARM_SAMPLE_CAP);
    double step = (double) (arm.size() - 1) / (ARM_SAMPLE_CAP - 1);
    for (int k = 0; k < ARM_SAMPLE_CAP; k++) {
      out.add(arm.get((int) Math.round(k * step)));
    }
    return out;
  }
}
