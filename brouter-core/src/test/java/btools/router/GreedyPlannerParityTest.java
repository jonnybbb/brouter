package btools.router;

import org.junit.Assert;
import org.junit.Assume;
import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import btools.router.roundtrip.RoundTripAlgorithm;
import btools.router.roundtrip.RoundTripResult;

/**
 * Refactor-parity pin for {@code GreedyRoundTripPlanner.plan()} (code-review
 * finding #2: splitting the 880-line method into session / candidate-round /
 * loop-assembler / outcome-evaluator pieces). Fingerprints the planner's FULL
 * observable output — routed polyline, loop waypoints, decision telemetry —
 * across a scenario grid on the bundled Dreieich fixture, and asserts it is
 * bit-identical to goldens captured on the pre-refactor code. Any structural
 * extraction that reorders, drops, or duplicates a state update in the plan
 * loop changes at least one fingerprint and fails here immediately, without
 * waiting for the (slow, real-map) loop-quality matrix.
 *
 * <p>Every scenario also runs TWICE with fresh engines and asserts the two
 * fingerprints are equal — the determinism precondition that makes golden
 * comparison meaningful (the same property ADR-0001's variety-seed sentinel
 * pins at loop granularity; here it is pinned at full-telemetry granularity).
 *
 * <p>These goldens are CHANGE DETECTORS, not correctness claims: any
 * deliberate behavior change to the planner (scoring weights, budgets,
 * candidate policy) is EXPECTED to break them. Recapture in that case:
 * run with {@code -Dgreedy.parity.print=true}, which prints the current
 * fingerprints (and skips the golden assertions), then paste the printed
 * block over the {@code GOLDENS} entries below.
 *
 * <p>Wall-clock-dependent fields (runtimeMillis, the diagnostics list — its
 * budget line embeds elapsed ms) are deliberately excluded from the
 * fingerprint. Deadline branches never fire on the fixture (plans finish in
 * well under a second against a 30s budget), so the remaining fields are
 * decision-determined, not time-determined.
 */
public class GreedyPlannerParityTest {

  private static final boolean PRINT_MODE = Boolean.getBoolean("greedy.parity.print");

  /** Scenario grid: algo|profile|direction|radius|seed → golden fingerprint. */
  private static final Map<String, String> GOLDENS = new LinkedHashMap<>();

  static {
    // Recaptured 2026-08-29 (closure-phase levers + phase-1 cost for non-paved
    // profiles; the fastbike rows are unchanged).
    // Captured on the pre-refactor baseline (2026-07-18, branch
    // roundtrip-upstream-v2 after review findings #1/#3/#4). See class doc
    // for the recapture procedure.
    GOLDENS.put("GREEDY|gravel|0|1000|0", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=57;cr=8;rk=6;ri=0;rn=8;ai=0;an=2;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("GREEDY|gravel|90|1000|0", "err=-;n=194;d=5583;h=33b63e41ba389b47;pd=6085;tol=true;fb=-;wp=f93285ddeb12ec6f;cg=89;cr=13;rk=11;ri=0;rn=13;ai=0;an=3;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("GREEDY|gravel|180|1000|0", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=57;cr=8;rk=6;ri=0;rn=8;ai=0;an=2;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("GREEDY|gravel|270|1000|0", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=57;cr=8;rk=6;ri=0;rn=8;ai=0;an=2;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("ISO_GREEDY|gravel|90|1000|0", "err=-;n=244;d=6432;h=cda577fb4518a5b2;pd=6552;tol=true;fb=-;wp=d3715b6d4fa88959;cg=135;cr=13;rk=13;ri=3;rn=10;ai=1;an=2;aq=0;ps=-1;ph=0.7400;fc=false;gc=true");
    GOLDENS.put("ISO_GREEDY|gravel|270|1000|0", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=85;cr=8;rk=7;ri=1;rn=7;ai=0;an=2;aq=0;ps=-1;ph=0.7500;fc=false;gc=false");
    GOLDENS.put("GREEDY|trekking|90|1000|0", "err=-;n=175;d=4165;h=b144da87588d8287;pd=6166;tol=true;fb=-;wp=c4281fef53473ff0;cg=120;cr=18;rk=15;ri=0;rn=18;ai=0;an=3;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    // Recaptured 2026-07-25 (scorer shape terms + mtb cost band). The SHIPPED
    // ROUTE is unchanged — same node count, same distance, same polyline hash
    // f7042709e1b6e3d — only the planner's internal selection moved, and it
    // moved better: the plan is now within tolerance (tol false→true) with no
    // "best error=6.3%" fallback, off a different via set, having routed 13
    // candidates instead of 18. RouteChoiceScore drives the greedy planner's
    // own top-K, so a scorer change is expected to show up here.
    GOLDENS.put("ISO_GREEDY|trekking|270|1000|0", "err=-;n=175;d=4165;h=b144da87588d8287;pd=5464;tol=false;fb=best error=13.2%;wp=44deb0089a17cafd;cg=173;cr=18;rk=14;ri=2;rn=16;ai=0;an=2;aq=0;ps=-1;ph=0.7200;fc=false;gc=true");
    GOLDENS.put("GREEDY|fastbike|90|1000|0", "err=greedy round trip planner produced no acceptable loop: could not build any loop;track=-;pd=0;tol=false;fb=could not build any loop;wp=0;cg=0;cr=0;rk=0;ri=0;rn=0;ai=0;an=0;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("ISO_GREEDY|fastbike|90|1000|0", "err=greedy round trip planner produced no acceptable loop: could not build any loop;track=-;pd=0;tol=false;fb=could not build any loop;wp=0;cg=0;cr=0;rk=0;ri=0;rn=0;ai=0;an=0;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("GREEDY|gravel|90|1000|7", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=57;cr=8;rk=6;ri=0;rn=8;ai=0;an=2;aq=0;ps=-1;ph=NaN;fc=false;gc=false");
    GOLDENS.put("ISO_GREEDY|gravel|90|1000|7", "err=-;n=230;d=6242;h=283c9ac6933f4b7;pd=6335;tol=true;fb=-;wp=3b95d653a8945793;cg=84;cr=8;rk=7;ri=1;rn=7;ai=0;an=2;aq=0;ps=-1;ph=0.7400;fc=false;gc=true");
  }

  @Test
  public void plannerOutputMatchesPreRefactorGoldens() {
    StringBuilder printBlock = new StringBuilder();
    StringBuilder mismatches = new StringBuilder();
    for (Map.Entry<String, String> e : GOLDENS.entrySet()) {
      String key = e.getKey();
      String fp1 = fingerprintFor(key);
      String fp2 = fingerprintFor(key);
      // Determinism is load-bearing in BOTH modes: goldens are only
      // meaningful if the same request reproduces the same plan.
      Assert.assertEquals("nondeterministic planner output for " + key, fp1, fp2);
      if (PRINT_MODE) {
        printBlock.append("    GOLDENS.put(\"").append(key).append("\", \"")
          .append(fp1).append("\");\n");
      } else if (!e.getValue().equals(fp1)) {
        mismatches.append("\n  ").append(key)
          .append("\n    golden:  ").append(e.getValue())
          .append("\n    current: ").append(fp1);
      }
    }
    if (PRINT_MODE) {
      System.out.println("=== GreedyPlannerParityTest current fingerprints ===");
      System.out.print(printBlock);
      System.out.println("=== end fingerprints ===");
      Assume.assumeTrue("print mode: fingerprints captured, assertions skipped", false);
    }
    Assert.assertEquals("planner fingerprints diverged from the pre-refactor goldens"
      + " (deliberate behavior change? recapture with -Dgreedy.parity.print=true):"
      + mismatches, "", mismatches.toString());
  }

  private static String fingerprintFor(String key) {
    String[] parts = key.split("\\|");
    RoundTripAlgorithm algo = RoundTripAlgorithm.valueOf(parts[0]);
    String profile = parts[1];
    int direction = Integer.parseInt(parts[2]);
    int radius = Integer.parseInt(parts[3]);
    int seed = Integer.parseInt(parts[4]);
    RoutingEngine re = RoundTripFixture.engine(profile, direction, radius, rc -> {
      rc.roundTripAlgorithm = algo;
      if (seed > 0) {
        rc.setAlternativeIdx(seed);
      }
    });
    return fingerprint(re);
  }

  /** Deterministic digest of everything the planner decided (no wall-clock fields). */
  private static String fingerprint(RoutingEngine re) {
    StringBuilder sb = new StringBuilder(160);
    String err = re.getErrorMessage();
    sb.append("err=").append(err == null ? "-" : err.replace('\n', ' '));
    OsmTrack track = re.getFoundTrack();
    if (track == null || track.nodes == null) {
      sb.append(";track=-");
    } else {
      sb.append(";n=").append(track.nodes.size())
        .append(";d=").append(track.distance)
        .append(";h=").append(Long.toHexString(polylineHash(track.nodes)));
    }
    RoundTripResult r = re.getLastRoundTripResult();
    if (r == null) {
      sb.append(";planner=-");
    } else {
      sb.append(";pd=").append(r.getTotalDistanceMeters())
        .append(";tol=").append(r.isWithinTolerance())
        .append(";fb=").append(r.getFallbackReason() == null ? "-" : r.getFallbackReason())
        .append(";wp=").append(Long.toHexString(waypointsHash(r.getLoopWaypoints())))
        .append(";cg=").append(r.getCandidatesGenerated())
        .append(";cr=").append(r.getCandidatesRouted())
        .append(";rk=").append(r.getReturnChecksPerformed())
        .append(";ri=").append(r.getRoutedIsoCandidates())
        .append(";rn=").append(r.getRoutedNonIsoCandidates())
        .append(";ai=").append(r.getAcceptedIsoLegs())
        .append(";an=").append(r.getAcceptedNonIsoLegs())
        .append(";aq=").append(r.getAcceptedQuotaInjectedLegs())
        .append(";ps=").append(r.getPoolDemotedAtStep())
        .append(";ph=").append(Double.isNaN(r.getIsoPoolHealthScore())
          ? "NaN" : String.format(Locale.US, "%.4f", r.getIsoPoolHealthScore()))
        .append(";fc=").append(r.isForcedCorridorAccepted())
        .append(";gc=").append(r.isInternalGraphNativeCompared());
    }
    return sb.toString();
  }

  /** Order-sensitive FNV-1a over the polyline's integer coordinates. */
  private static long polylineHash(List<OsmPathElement> nodes) {
    long h = 0xcbf29ce484222325L;
    for (OsmPathElement n : nodes) {
      h = (h ^ n.getILon()) * 0x100000001b3L;
      h = (h ^ n.getILat()) * 0x100000001b3L;
    }
    return h;
  }

  private static long waypointsHash(List<OsmNodeNamed> wps) {
    if (wps == null) return 0;
    long h = 0xcbf29ce484222325L;
    for (OsmNodeNamed w : wps) {
      h = (h ^ w.ilon) * 0x100000001b3L;
      h = (h ^ w.ilat) * 0x100000001b3L;
    }
    return h;
  }
}
