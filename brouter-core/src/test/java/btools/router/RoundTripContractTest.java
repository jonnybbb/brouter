package btools.router;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Behavioural contract for round-trip routing, exercised across edge radii
 * (small/medium/large relative to the Dreieich fixture) where loops may or may
 * not be feasible:
 * <ul>
 *   <li><b>Success implies a real loop</b> — the engine must never report success
 *       (null error) while returning a degenerate stub. If a loop cannot be formed
 *       it must fail with a clear error and no track.</li>
 *   <li><b>Determinism</b> — identical inputs produce an identical track.</li>
 * </ul>
 */
@RunWith(Parameterized.class)
public class RoundTripContractTest {

  // Lower bounds the engine guarantees for any reported loop. Tighter than the engine's
  // coarse "is it a loop at all" guard (400 m closure) because a successful loop on real
  // road data closes within metres; this asserts the quality, not just the guard.
  private static final int MIN_LOOP_NODES = 6;
  private static final int MIN_LOOP_METERS = 200;
  private static final int CLOSURE_MAX_M = 150;

  @Parameterized.Parameter(0)
  public String profile;
  @Parameterized.Parameter(1)
  public int direction;
  @Parameterized.Parameter(2)
  public int radius;

  @Parameterized.Parameters(name = "{0}_dir{1}_r{2}")
  public static Collection<Object[]> data() {
    List<Object[]> params = new ArrayList<>();
    for (String profile : new String[]{"trekking", "fastbike", "gravel", "mtb"}) {
      for (int dir : new int[]{0, 90, 180, 270}) {
        // span feasible to over-constrained radii on the small fixture
        for (int radius : new int[]{500, 1500, 5000}) {
          params.add(new Object[]{profile, dir, radius});
        }
      }
    }
    return params;
  }

  /** Either a valid loop, or a clean failure — never a degenerate "success". */
  @Test
  public void successImpliesValidLoop() {
    RoutingEngine re = RoundTripFixture.engine(profile, direction, radius);
    String err = re.getErrorMessage();
    OsmTrack track = re.getFoundTrack();

    if (err != null) {
      // Acceptable: a loop is genuinely infeasible for this direction/radius on the
      // fixture. The contract only requires that failure be explicit and trackless.
      Assert.assertNull(label() + ": error set but a track was still returned", track);
      return;
    }

    Assert.assertNotNull(label() + ": success reported but no track", track);
    Assert.assertTrue(label() + ": success reported with degenerate loop ("
        + track.nodes.size() + " nodes, " + track.distance + "m)",
      track.nodes.size() >= MIN_LOOP_NODES && track.distance >= MIN_LOOP_METERS);

    int closing = track.nodes.get(0).calcDistance(track.nodes.get(track.nodes.size() - 1));
    Assert.assertTrue(label() + ": loop does not close, gap " + closing + "m",
      closing <= CLOSURE_MAX_M);
  }

  /**
   * Identical inputs must yield an identical OUTCOME — and that includes failure.
   *
   * <p>This used to {@code Assume} its way past every case the fixture cannot form a loop
   * for, which silently disabled the determinism contract on <b>31 of 48</b> parameter
   * combinations: the assertion only ever ran where a loop happened to exist. Infeasible
   * cases are the ones most likely to expose non-determinism — they run the full retry
   * ladder and every fallback before giving up — so they were exactly the wrong cases to
   * skip. A deterministic engine must fail the same way twice: same message, still no track.
   */
  @Test
  public void deterministic() {
    RoutingEngine re1 = RoundTripFixture.engine(profile, direction, radius);
    String err1 = re1.getErrorMessage();
    OsmTrack t1 = re1.getFoundTrack();

    RoutingEngine re2 = RoundTripFixture.engine(profile, direction, radius);
    String err2 = re2.getErrorMessage();
    OsmTrack t2 = re2.getFoundTrack();

    Assert.assertEquals(label() + ": one run failed and the other did not",
      err1 == null, err2 == null);
    if (err1 != null) {
      // Failure determinism: same reason, and the trackless contract holds both times.
      // Compare with elapsed times normalised: the engine embeds a wall-clock
      // duration in some failure messages ("tried 4 candidates in 497ms"), which
      // varies run to run by construction. Everything that carries meaning —
      // the reason, the candidate count, the tier — must still match exactly.
      Assert.assertEquals(label() + ": error message differs between runs",
        withoutTimings(err1), withoutTimings(err2));
      Assert.assertNull(label() + ": error set but run 1 returned a track", t1);
      Assert.assertNull(label() + ": error set but run 2 returned a track", t2);
      return;
    }

    Assert.assertNotNull(label() + ": second run produced no track", t2);
    Assert.assertEquals(label() + ": node count differs between runs",
      t1.nodes.size(), t2.nodes.size());
    for (int i = 0; i < t1.nodes.size(); i++) {
      OsmPathElement a = t1.nodes.get(i), b = t2.nodes.get(i);
      Assert.assertTrue(label() + ": node " + i + " differs between runs",
        a.getILon() == b.getILon() && a.getILat() == b.getILat());
    }
  }

  /** Strip embedded wall-clock durations so determinism is asserted on content, not timing. */
  private static String withoutTimings(String msg) {
    return msg == null ? null : msg.replaceAll("\\d+\\s?ms", "<ms>");
  }

  private String label() {
    return profile + "_dir" + direction + "_r" + radius;
  }
}
