package btools.router;

import java.util.ArrayList;

import org.junit.Assert;
import org.junit.Test;

/**
 * Unit tests for {@link GraphNativeCandidateProvider}'s per-radius expansion
 * cache.
 */
public class GraphNativeCandidateProviderTest {

  /**
   * Regression guard: an empty/failed Dijkstra expansion must NOT be cached.
   * The provider caches expansion pools per (position, rounded radius) to avoid
   * recomputing the expensive expansion. If a transient/empty result were cached,
   * every subsequent attempt at the same radius would be silently served the
   * empty pool with no re-attempt and no further signal — making "could not form
   * a loop" indistinguishable from "the expansion momentarily failed".
   */
  @Test
  public void emptyExpansionIsNotCachedSoRetriesReattempt() {
    final int[] calls = {0};
    RoutingContext rc = new RoutingContext();
    rc.localFunction = RoundTripFixture.profileFile("gravel").getAbsolutePath();
    RoutingEngine engine = new RoutingEngine(null, null, RoundTripFixture.segmentDir(), new ArrayList<>(), rc) {
      @Override
      IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                     OsmTrack refTrack, boolean includeCandidateTracks) {
        calls[0]++;
        return null; // simulate a transient / empty expansion failure
      }
    };

    GraphNativeCandidateProvider provider = new GraphNativeCandidateProvider(engine);
    int ilon = 180_000_000;
    int ilat = 90_000_000;
    // Two attempts at the same position and (rounded) radius, no refTrack -> the
    // cache path. The empty result of the first must not suppress the second.
    provider.candidatesForStep(ilon, ilat, 500.0, 1, 5, ilon, ilat, 0.0, null);
    provider.candidatesForStep(ilon, ilat, 500.0, 1, 5, ilon, ilat, 0.0, null);

    Assert.assertEquals(
      "an empty expansion must not be cached: the second attempt at the same radius must re-run it",
      2, calls[0]);
  }
}
