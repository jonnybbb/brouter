package btools.router;

import org.junit.Assert;
import org.junit.Test;

/**
 * Geometric- and algorithm-quality guards for round-trip routing that run in CI
 * against the bundled Dreieich fixture (see {@link RoundTripFixture}). These
 * complement {@link RoundTripInvariantTest} (which validates the default/AUTO
 * algorithm's structural invariants across profiles/directions) by covering
 * aspects the existing CI suites do not:
 *
 * <ul>
 *   <li><b>Geometric cleanliness</b> — the whole point of the AUTO redesign was
 *       to replace the probe-spike chaos pattern (many self-crossings) with
 *       clean loops. That was only guarded by the segments-gated Mallorca test
 *       (skipped in CI). This pins self-crossings on the fixture.</li>
 *   <li><b>Explicit GREEDY / ISO_GREEDY validity</b> — the scenario suite forces
 *       WAYPOINT and ISOCHRONE; the greedy variants (which assemble a loop from
 *       merged legs) were only validated structurally via AUTO's internal
 *       choice. Here they are forced and validated directly.</li>
 *   <li><b>AUTO competition is entered and the winner recorded</b> — previously
 *       only asserted in the segments-gated competition suite.</li>
 * </ul>
 *
 * <p>The fixture is a tiny synthetic tile, so loops only form in directions with
 * enough road; east (90°) reliably yields a loop for every algorithm here. Other
 * directions hit the quality gate's distance band and are covered by the
 * gated real-geography suite ({@link LoopQualityTest}) and {@link RoundTripContractTest}.
 */
public class RoundTripQualityFixtureTest {

  private static final int EAST = 90;
  private static final int RADIUS = 2000;

  /** Clean loops measure 0 self-crossings on the fixture; allow a small margin
   *  for routing-engine variance while still failing the chaos pattern (many). */
  private static final int MAX_SELF_CROSSINGS = 2;

  /** Greedy-merged loops retrace a short shared stem near the origin, so allow
   *  more reuse than the strict 30% AUTO invariant while still requiring a loop. */
  private static final double MAX_REUSE_PCT = 40.0;

  @Test
  public void autoLoopIsGeometricallyCleanAndValid() {
    assertCleanLoop(RoundTripAlgorithm.AUTO);
  }

  @Test
  public void greedyVariantsProduceCleanValidLoops() {
    assertCleanLoop(RoundTripAlgorithm.GREEDY);
    assertCleanLoop(RoundTripAlgorithm.ISO_GREEDY);
  }

  @Test
  public void autoCompetitionAdoptsAndRecordsWinner() {
    RoutingEngine re = RoundTripFixture.engine("trekking", EAST, RADIUS,
      rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO);
    Assert.assertNull("AUTO completed: " + re.getErrorMessage(), re.getErrorMessage());
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull("AUTO produced a track", track);
    Assert.assertNotNull("AUTO track carries a message", track.message);
    // The competition adopted a candidate and recorded which algorithm won.
    Assert.assertTrue("AUTO message records the competition winner: " + track.message,
      track.message.contains("AUTO selected"));
  }

  /**
   * Forced GREEDY/ISO_GREEDY bypass the competition but still run through the
   * shared finalize path; the result must record the standard info line (not be
   * left with only the planner's internal note), proving the adopted track is
   * fully finalized.
   */
  @Test
  public void forcedGreedyVariantsAreFullyFinalized() {
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      RoutingEngine re = RoundTripFixture.engine("trekking", EAST, RADIUS,
        rc -> rc.roundTripAlgorithm = algo);
      Assert.assertNull(algo + " completed: " + re.getErrorMessage(), re.getErrorMessage());
      OsmTrack track = re.getFoundTrack();
      Assert.assertNotNull(algo + " produced a track", track);
      Assert.assertFalse(algo + " bypasses the competition (no AUTO summary)",
        track.message != null && track.message.contains("AUTO selected"));
      Assert.assertNotNull(algo + " info line present", track.messageList);
      Assert.assertFalse(algo + " info line non-empty", track.messageList.isEmpty());
    }
  }

  private void assertCleanLoop(RoundTripAlgorithm algo) {
    RoutingEngine re = RoundTripFixture.engine("trekking", EAST, RADIUS,
      rc -> rc.roundTripAlgorithm = algo);
    Assert.assertNull(algo + " completed: " + re.getErrorMessage(), re.getErrorMessage());
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull(algo + ": fixture should form an eastward loop", track);

    RoundTripFixture.assertValidLoop(track, algo + "_dir90_r" + RADIUS, MAX_REUSE_PCT);

    int selfCrossings = RoundTripFixture.countSelfCrossings(track);
    Assert.assertTrue(algo + ": loop must be geometrically clean — self-crossings "
        + selfCrossings + " > " + MAX_SELF_CROSSINGS,
      selfCrossings <= MAX_SELF_CROSSINGS);
  }
}
