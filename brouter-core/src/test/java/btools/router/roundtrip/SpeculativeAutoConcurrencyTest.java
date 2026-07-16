package btools.router.roundtrip;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Assume;
import org.junit.Test;

import btools.router.OsmNodeNamed;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

/**
 * The speculative AUTO path under real threads ({@code
 * -DroundTripSpeculativeAutoGreedy=true}): the GREEDY child runs in parallel,
 * is joined (or terminated) before its result is read, releases its permit,
 * and leaves no thread behind — including when the PARENT is terminated
 * mid-competition (the termination cascade). Everything else in the AUTO
 * suite runs the default sequential path; this is the only coverage of the
 * opt-in concurrency.
 */
public class SpeculativeAutoConcurrencyTest {

  private static final String FLAG = "roundTripSpeculativeAutoGreedy";

  @After
  public void clearFlag() {
    System.clearProperty(FLAG);
  }

  private static RoutingEngine autoEngine() {
    File projectDir;
    try {
      projectDir = new File(".").getCanonicalFile().getParentFile();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    File segDir = new File(projectDir, "brouter-map-creator/build/resources/test/tmp/segments");
    File profile = new File(projectDir, "misc/profiles2/trekking.brf");
    Assume.assumeTrue("fixture segments not built", segDir.isDirectory());
    Assume.assumeTrue("trekking profile missing", profile.isFile());

    List<OsmNodeNamed> wps = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = 188720000; // Dreieich fixture origin
    start.ilat = 140000000;
    wps.add(start);
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profile.getAbsolutePath();
    rc.startDirection = 90;
    rc.roundTripDistance = 1500;
    rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    RoutingEngine re = new RoutingEngine(null, null, segDir, wps, rc,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    return re;
  }

  private static void awaitPermits(int expected) throws InterruptedException {
    // The speculative child releases its permit in its own thread's finally,
    // which can complete a beat after the parent read its result slot.
    long deadline = System.currentTimeMillis() + 10_000;
    while (AutoCompetitionStrategy.availableParallelAutoPermits() < expected
        && System.currentTimeMillis() < deadline) {
      Thread.sleep(20);
    }
    assertEquals("speculative permit must be released (leak degrades AUTO to sequential)",
      expected, AutoCompetitionStrategy.availableParallelAutoPermits());
  }

  private static void awaitNoGreedyChildThread() throws InterruptedException {
    long deadline = System.currentTimeMillis() + 10_000;
    while (System.currentTimeMillis() < deadline) {
      boolean alive = false;
      for (Thread t : Thread.getAllStackTraces().keySet()) {
        if ("roundtrip-auto-greedy".equals(t.getName()) && t.isAlive()) {
          alive = true;
          break;
        }
      }
      if (!alive) return;
      Thread.sleep(20);
    }
    throw new AssertionError("a roundtrip-auto-greedy child thread is still alive");
  }

  @Test
  public void speculativeCompetitionCompletesAndReleasesPermit() throws Exception {
    int permits = AutoCompetitionStrategy.availableParallelAutoPermits();
    Assume.assumeTrue("no speculative permits on this machine", permits > 0);
    System.setProperty(FLAG, "true");

    RoutingEngine re = autoEngine();
    re.doRun(60_000);

    // Track XOR error, same as the sequential competition.
    if (re.getFoundTrack() == null) {
      assertNotNull("no track must come with an error", re.getErrorMessage());
    }
    awaitPermits(permits);
    awaitNoGreedyChildThread();
  }

  @Test
  public void terminatingTheParentStopsTheSpeculativeChild() throws Exception {
    int permits = AutoCompetitionStrategy.availableParallelAutoPermits();
    Assume.assumeTrue("no speculative permits on this machine", permits > 0);
    System.setProperty(FLAG, "true");

    RoutingEngine re = autoEngine();
    Thread request = new Thread(() -> re.doRun(60_000), "speculative-auto-request");
    request.start();
    // Let the competition spawn its children, then pre-empt the parent — the
    // cascade must terminate live children (and late-registered ones fire
    // immediately), so the request unwinds long before its 60s budget.
    Thread.sleep(300);
    re.terminate();
    request.join(30_000);
    assertFalse("terminated request must unwind promptly", request.isAlive());

    awaitPermits(permits);
    awaitNoGreedyChildThread();
  }
}
