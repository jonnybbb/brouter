package btools.router.roundtrip;

import java.util.ArrayList;

import org.junit.Assert;
import org.junit.Test;

import btools.router.OsmPathElement;
import btools.router.OsmTrack;

/**
 * The post-routing ring-retry trigger: a directional lobe that placed but did
 * not route into a real loop must be retried as an encircling ring.
 */
public class FastStrategyTest {

  private static OsmTrack track(int nodes, int distance) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    for (int i = 0; i < nodes; i++) {
      t.nodes.add(OsmPathElement.create(1000 * i, 1000 * i, (short) 0, null));
    }
    t.distance = distance;
    return t;
  }

  @Test
  public void degenerateOutcomeTriggersOnMissingOrStubTracks() {
    RoundTripRequest request = new RoundTripRequest(null);

    request.track = null;
    Assert.assertTrue("no track routes -> retry", FastStrategy.degenerateOutcome(request));

    request.track = track(3, 5000);
    Assert.assertTrue("below the node floor -> retry", FastStrategy.degenerateOutcome(request));

    request.track = track(20, 150);
    Assert.assertTrue("below the length floor -> retry", FastStrategy.degenerateOutcome(request));
  }

  @Test
  public void degenerateOutcomeAcceptsARealLoop() {
    RoundTripRequest request = new RoundTripRequest(null);
    request.track = track(20, 5000);
    Assert.assertFalse(FastStrategy.degenerateOutcome(request));
  }

  // ---- shape-retry cascade: the length-band rung -------------------------
  // The cascade is shape-first by design and says so ("length never decides
  // here"). That is right between two loops the rider would accept as the
  // asked size, and wrong when one of them is not a plausible answer at all:
  // Garmisch 96 km east shipped a 0.67x loop over a 0.91x one because the
  // 0.67x loop scored higher compactness and nothing above that rung broke
  // the tie. This rung fires ONLY in that case — exactly one candidate inside
  // the same tolerant band the length-correction pass already uses.

  @Test
  public void lengthBandRungPrefersTheCandidateInsideTheBand() {
    Assert.assertEquals("0.91x in band beats 0.67x out of band",
      1, FastStrategy.lengthBandPreference(0.67, 0.91));
    Assert.assertEquals("and symmetrically the other way",
      -1, FastStrategy.lengthBandPreference(0.91, 0.67));
  }

  @Test
  public void lengthBandRungAbstainsWhenBothOrNeitherAreInBand() {
    Assert.assertEquals("both plausible lengths — shape must decide",
      0, FastStrategy.lengthBandPreference(0.90, 1.10));
    Assert.assertEquals("both implausible — shape must decide",
      0, FastStrategy.lengthBandPreference(0.60, 1.50));
    Assert.assertEquals("identical ratios never break a tie",
      0, FastStrategy.lengthBandPreference(0.67, 0.67));
  }

  @Test
  public void lengthBandRungUsesTheCorrectionPassBand() {
    // Same [0.75, 1.25] the radius correction treats as "length satisfied
    // either way" — one band for the whole tier, edges inclusive.
    Assert.assertTrue(FastStrategy.inTolerantLengthBand(FastStrategy.RADIUS_TOLERANT_UNDERSHOOT));
    Assert.assertTrue(FastStrategy.inTolerantLengthBand(FastStrategy.RADIUS_MAX_OVERSHOOT));
    Assert.assertFalse(FastStrategy.inTolerantLengthBand(
      FastStrategy.RADIUS_TOLERANT_UNDERSHOOT - 0.01));
    Assert.assertFalse(FastStrategy.inTolerantLengthBand(
      FastStrategy.RADIUS_MAX_OVERSHOOT + 0.01));
  }

  @Test
  public void lengthBandRungIsInertWithoutAKnownAsk() {
    // A non-positive ask (no radius, no explicit length) must not synthesise
    // an opinion — ratios are meaningless there.
    Assert.assertEquals(0, FastStrategy.lengthBandPreference(0.0, 0.0));
  }
}
