package btools.router;

import org.junit.Assert;
import org.junit.Test;

/**
 * Geometric- and algorithm-quality guards for <em>small</em> round-trip loops
 * that run in CI against the bundled Dreieich fixture (see {@link RoundTripFixture}).
 * They complement {@link RoundTripInvariantTest} (structural invariants of the
 * default/AUTO algorithm across profiles/directions) by covering aspects no CI
 * suite touched:
 *
 * <ul>
 *   <li><b>Omnidirectional cleanliness</b> — small loops in <em>all four</em>
 *       compass directions for AUTO and both greedy variants. The whole point of
 *       the AUTO redesign was to replace the probe-spike chaos pattern (many
 *       self-crossings) with clean loops; that was only guarded by the
 *       segments-gated Mallorca test (skipped in CI). Here it runs on the fixture.</li>
 *   <li><b>Explicit GREEDY / ISO_GREEDY validity</b> — the scenario suite forces
 *       only WAYPOINT and ISOCHRONE; the greedy variants (which assemble a loop
 *       from merged legs) are forced and validated directly here.</li>
 *   <li><b>AUTO competition entered + winner recorded</b>, and forced variants
 *       fully finalized — previously only in the segments-gated competition suite.</li>
 *   <li><b>Profile policy</b> — a paved-only profile must reject the fixture's
 *       path/track terrain with a clear error and no degenerate track.</li>
 *   <li><b>Radius is honoured</b> — a larger search radius yields a longer loop.</li>
 * </ul>
 *
 * <p>The fixture is a ~3 km synthetic tile. The {@code gravel} profile forms a
 * clean loop in every direction at small radii (matrix-verified across
 * algorithm/direction/radius), so these tests are reliable rather than
 * direction-fragile. Larger radii and real-geography shape quality live in the
 * gated suite ({@link LoopQualityTest}).
 */
public class RoundTripQualityFixtureTest {

  private static final String PROFILE = "gravel";
  private static final int RADIUS = 1000;
  private static final int EAST = 90;
  private static final int[] DIRECTIONS = {0, 90, 180, 270};

  /** Clean loops measure 0–1 self-crossings on the fixture; allow a small margin
   *  while still failing the chaos pattern (many crossings). */
  private static final int MAX_SELF_CROSSINGS = 2;

  /** Greedy-merged loops retrace a short shared stem near the origin, so allow
   *  more reuse than the strict 30% AUTO invariant while still requiring a loop. */
  private static final double MAX_REUSE_PCT = 40.0;

  @Test
  public void omnidirectionalSmallLoopsAreCleanAndValid() {
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.AUTO, RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      for (int dir : DIRECTIONS) {
        assertCleanLoop(algo, dir);
      }
    }
  }

  @Test
  public void autoCompetitionAdoptsAndRecordsWinner() {
    RoutingEngine re = RoundTripFixture.engine(PROFILE, EAST, RADIUS,
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
   * fully finalized — and must not carry an AUTO summary.
   */
  @Test
  public void forcedGreedyVariantsAreFullyFinalized() {
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      RoutingEngine re = RoundTripFixture.engine(PROFILE, EAST, RADIUS,
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

  /**
   * Profile policy: a paved-only road-bike profile must reject the fixture's
   * unpaved path/track terrain through the quality gate — a clear error and no
   * degenerate track, never a silently-bad loop on hostile ways.
   */
  @Test
  public void pavedOnlyProfileRejectsHostileFixtureCleanly() {
    RoutingEngine re = RoundTripFixture.engine("fastbike", EAST, RADIUS,
      rc -> rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO);
    Assert.assertNotNull("paved-only profile must fail on the unpaved fixture",
      re.getErrorMessage());
    Assert.assertNull("a rejected route must not return a track", re.getFoundTrack());
    Assert.assertTrue("error should explain the rejection: " + re.getErrorMessage(),
      re.getErrorMessage().contains("rejected") || re.getErrorMessage().contains("hostile")
        || re.getErrorMessage().contains("no acceptable route"));
  }

  /** A larger search radius must yield a longer loop (the radius is honoured). */
  @Test
  public void largerRadiusYieldsLongerLoop() {
    OsmTrack small = loop(RoundTripAlgorithm.AUTO, EAST, 800);
    OsmTrack large = loop(RoundTripAlgorithm.AUTO, EAST, 1500);
    Assert.assertNotNull("r800 loop", small);
    Assert.assertNotNull("r1500 loop", large);
    Assert.assertTrue("r1500 loop (" + large.distance + "m) must be clearly longer than r800 ("
        + small.distance + "m)", large.distance > small.distance * 1.2);
  }

  // -------------------------------------------------------------------------

  private void assertCleanLoop(RoundTripAlgorithm algo, int dir) {
    String label = algo + "_dir" + dir + "_r" + RADIUS;
    RoutingEngine re = RoundTripFixture.engine(PROFILE, dir, RADIUS,
      rc -> rc.roundTripAlgorithm = algo);
    Assert.assertNull(label + " completed: " + re.getErrorMessage(), re.getErrorMessage());
    OsmTrack track = re.getFoundTrack();
    Assert.assertNotNull(label + ": fixture should form a small loop", track);

    RoundTripFixture.assertValidLoop(track, label, MAX_REUSE_PCT);

    int selfCrossings = RoundTripFixture.countSelfCrossings(track);
    Assert.assertTrue(label + ": loop must be geometrically clean — self-crossings "
        + selfCrossings + " > " + MAX_SELF_CROSSINGS,
      selfCrossings <= MAX_SELF_CROSSINGS);
  }

  private OsmTrack loop(RoundTripAlgorithm algo, int dir, int radius) {
    return RoundTripFixture.engine(PROFILE, dir, radius,
      rc -> rc.roundTripAlgorithm = algo).getFoundTrack();
  }
}
