package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import btools.util.CheapRuler;
import org.junit.Assume;
import org.junit.Test;

/**
 * Measurement (not a pass/fail test): quantifies how often TODAY's explicit-via
 * round-trip path (plain Dijkstra through user vias) produces a route that
 * <em>retraces</em> or <em>undershoots</em> the requested distance — the two
 * defects that a hypothetical "greedy-with-anchors" planner would address.
 *
 * <p>Purpose: decide, with data instead of intuition, whether greedy-with-vias
 * is worth building. If retrace/undershoot are rare on a representative via-loop
 * corpus, the feature is low value. If common, the numbers tell us whether a
 * surgical fix (retrace-aware closing leg + length padding) suffices or the full
 * planner change is warranted.
 *
 * <p>Opt-in (touches real {@code segments4/} tiles and routes ~dozens of loops):
 * <pre>./gradlew :brouter-core:test --tests '*ExplicitViaRetraceMeasurementTest' -Dvia.measure=true</pre>
 *
 * <p>Method: for each region × supported profile × loop radius × via-count, place
 * {@code N} synthetic user vias evenly spread on a circle of the target radius
 * around the start (well-spread, i.e. the explicit-via path's <em>best</em> case),
 * route {@code start → via1 … viaN → start}, then classify the result with the
 * production {@link RoundTripQualityGate}. We record the canonical
 * {@link RouteShape}, the gate's total reuse ratio (the retrace metric), and the
 * distance ratio vs the requested {@code 2π·radius}.
 */
public class ExplicitViaRetraceMeasurementTest {

  /** Loop radii in metres → loop circumference ≈ 2π·r (~19 km and ~38 km). */
  private static final int[] RADII_M = {3000, 6000};
  /** Number of user vias to scatter around the loop. 1 = out-and-back. */
  private static final int[] VIA_COUNTS = {1, 2, 3};
  /** Base bearing the via ring is rotated to. */
  private static final int BASE_DIRECTION = 30;

  /** A route is "meaningfully retracing" above this share of reused distance. */
  private static final double RETRACE_RATIO_THRESHOLD = 0.10;
  /** A route "undershoots" when it is below this fraction of the requested loop. */
  private static final double UNDERSHOOT_RATIO = 0.85;

  private record Case(LoopTestRegion region, String profile, int radiusM, int viaCount) {}

  private record Outcome(Case c, boolean routed, String failure,
                         RouteShape shape, double reuseRatio, double distanceRatio,
                         int distanceM) {}

  @Test
  public void measureExplicitViaRetraceAndUndershoot() throws Exception {

    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    Assume.assumeTrue("segments4/ not found at " + segDir.getAbsolutePath(), segDir.isDirectory());

    List<Outcome> outcomes = new ArrayList<>();

    for (LoopTestRegion region : LoopTestRegion.values()) {
      LoopTestSegments.ensureRegion(segDir, region);
      for (String profile : region.supportedProfiles) {
        File profileFile = new File(projectDir, "misc/profiles2/" + profile + ".brf");
        if (!profileFile.exists()) {
          continue;
        }
        for (int radius : RADII_M) {
          for (int viaCount : VIA_COUNTS) {
            outcomes.add(runCase(new Case(region, profile, radius, viaCount), segDir, profileFile));
          }
        }
      }
    }

    report(outcomes);
  }

  private Outcome runCase(Case c, File segDir, File profileFile) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = c.region().ilon;
    start.ilat = c.region().ilat;
    wps.add(start);

    // Scatter N vias on the target-radius circle, evenly spread so the explicit-via
    // path gets a loop-shaped skeleton (its best case, not a contrived collinear one).
    for (int i = 0; i < c.viaCount(); i++) {
      double bearing = (BASE_DIRECTION + 360.0 * i / c.viaCount()) % 360.0;
      int[] p = CheapRuler.destination(start.ilon, start.ilat, c.radiusM(), bearing);
      OsmNodeNamed via = new OsmNodeNamed();
      via.name = "via" + (i + 1);
      via.ilon = p[0];
      via.ilat = p[1];
      wps.add(via);
    }

    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    rc.startDirection = BASE_DIRECTION;
    rc.roundTripDistance = c.radiusM();
    rc.turnInstructionMode = 0;

    double expected = 2 * Math.PI * c.radiusM();
    try {
      RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      re.quite = true;
      re.doRun(0);

      OsmTrack track = re.getFoundTrack();
      if (track == null || track.nodes == null || track.nodes.isEmpty()) {
        String msg = re.getErrorMessage() == null ? "no track" : re.getErrorMessage();
        return new Outcome(c, false, msg, null, 0, 0, 0);
      }

      RoundTripQualityResult q = RoundTripQualityGate.evaluate(
        track, expected, c.profile(), false, true, true);
      double ratio = expected > 0 ? track.distance / expected : 0;
      return new Outcome(c, true, null, q.getShape(), q.getTotalReuseRatio(), ratio, track.distance);
    } catch (Throwable t) {
      // Unsnappable via, routing island, etc. — record as a non-route outcome.
      return new Outcome(c, false, t.getClass().getSimpleName() + ": " + t.getMessage(),
        null, 0, 0, 0);
    }
  }

  private void report(List<Outcome> outcomes) {
    System.out.println();
    System.out.println("================ EXPLICIT-VIA RETRACE / UNDERSHOOT MEASUREMENT ================");
    System.out.printf("%-16s %-9s %6s %4s  %-10s %14s  %8s %8s%n",
      "region", "profile", "radius", "vias", "result", "shape", "reuse%", "dist/req");
    System.out.println("-------------------------------------------------------------------------------");

    for (Outcome o : outcomes) {
      if (o.routed()) {
        System.out.printf("%-16s %-9s %5dm %4d  %-10s %14s  %7.0f%% %8.2f%n",
          o.c().region(), o.c().profile(), o.c().radiusM(), o.c().viaCount(),
          "routed", o.shape(), o.reuseRatio() * 100, o.distanceRatio());
      } else {
        System.out.printf("%-16s %-9s %5dm %4d  %-10s %s%n",
          o.c().region(), o.c().profile(), o.c().radiusM(), o.c().viaCount(),
          "no-route", abbreviate(o.failure()));
      }
    }

    List<Outcome> routed = outcomes.stream().filter(Outcome::routed).toList();
    System.out.println("-------------------------------------------------------------------------------");
    System.out.printf("cases: %d total, %d routed, %d no-route%n",
      outcomes.size(), routed.size(), outcomes.size() - routed.size());

    if (routed.isEmpty()) {
      System.out.println("No routed via-loops — cannot measure (check segment coverage).");
      return;
    }

    summariseGroup("ALL routed", routed);
    for (int viaCount : VIA_COUNTS) {
      List<Outcome> g = routed.stream().filter(o -> o.c().viaCount() == viaCount).toList();
      if (!g.isEmpty()) {
        summariseGroup(viaCount + " via" + (viaCount == 1 ? " (out-and-back)" : "s"), g);
      }
    }

    System.out.println();
    System.out.println("Reading: high reuse% / shape≠STRICT_LOOP at LOW via counts, falling toward 0");
    System.out.println("as vias increase, is the signature that greedy-with-vias helps only the");
    System.out.println("near-out-and-back end — and that a retrace-aware CLOSING LEG would capture");
    System.out.println("most of the benefit without a full planner rewrite.");
    System.out.println("===============================================================================");
  }

  private void summariseGroup(String label, List<Outcome> g) {
    long retrace = g.stream().filter(o -> o.reuseRatio() > RETRACE_RATIO_THRESHOLD).count();
    long notStrict = g.stream().filter(o -> o.shape() != RouteShape.STRICT_LOOP).count();
    long undershoot = g.stream().filter(o -> o.distanceRatio() < UNDERSHOOT_RATIO).count();
    int n = g.size();
    System.out.printf(
      "  %-22s n=%-3d  retrace>%.0f%%: %2d (%3.0f%%)   shape≠STRICT: %2d (%3.0f%%)   "
        + "undershoot<%.2f: %2d (%3.0f%%)   median reuse=%.0f%%  median dist/req=%.2f%n",
      label, n, RETRACE_RATIO_THRESHOLD * 100,
      retrace, pct(retrace, n), notStrict, pct(notStrict, n),
      UNDERSHOOT_RATIO, undershoot, pct(undershoot, n),
      median(g.stream().map(o -> o.reuseRatio() * 100).sorted().toList()),
      median(g.stream().map(Outcome::distanceRatio).sorted().toList()));
  }

  private static double pct(long k, int n) {
    return n == 0 ? 0 : 100.0 * k / n;
  }

  private static double median(List<Double> sorted) {
    if (sorted.isEmpty()) {
      return 0;
    }
    List<Double> s = new ArrayList<>(sorted);
    Collections.sort(s);
    int m = s.size() / 2;
    return s.size() % 2 == 1 ? s.get(m) : (s.get(m - 1) + s.get(m)) / 2.0;
  }

  private static String abbreviate(String s) {
    if (s == null) {
      return "";
    }
    String oneLine = s.replaceAll("\\s+", " ").trim();
    return oneLine.length() <= 90 ? oneLine : oneLine.substring(0, 90) + "…";
  }
}
