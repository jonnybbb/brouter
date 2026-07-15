package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.util.CheapRuler;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;

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
    StubEngineOps ops = new StubEngineOps() {
      @Override
      public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                            OsmTrack refTrack, boolean includeCandidateTracks) {
        calls[0]++;
        return null; // simulate a transient / empty expansion failure
      }
    };

    GraphNativeCandidateProvider provider = new GraphNativeCandidateProvider(ops);
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

  // ---- buildTemplates: window / dedupe / rank / cap / forwarding ----------
  // Exercised through candidatesForStep with a stubbed runIsochroneExpansion
  // (bundled profile, no segment data), since buildTemplates is private.

  private static final int FROM_ILON = 180_000_000;
  private static final int FROM_ILAT = 90_000_000;

  /** A provider whose Dijkstra expansion always returns {@code pool}. */
  private static GraphNativeCandidateProvider providerReturning(final List<IsoCandidate> pool) {
    return new GraphNativeCandidateProvider(new StubEngineOps() {
      @Override
      public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius,
                                                            OsmTrack refTrack, boolean includeCandidateTracks) {
        return new IsochroneExpansionResult(null, pool);
      }
    });
  }

  /**
   * Minimal seam stub: the provider touches only the expansion and the log.
   * Everything else throws — a test reaching further has outgrown this stub.
   */
  private abstract static class StubEngineOps implements RoundTripEngineOps {
    @Override
    public void logInfo(String msg) {
    }

    @Override
    public List<MatchedWaypoint> matchedWaypoints() {
      throw new UnsupportedOperationException();
    }

    @Override
    public List<OsmNodeNamed> waypoints() {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack foundTrack() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setFoundTrack(OsmTrack track) {
      throw new UnsupportedOperationException();
    }

    @Override
    public String errorMessage() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setErrorMessage(String message) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void doRouting(long budgetMs) {
      throw new UnsupportedOperationException();
    }

    @Override
    public RoundTripEffortPolicy roundTripEffortPolicy() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setRoundTripEffortPolicy(RoundTripEffortPolicy policy) {
      throw new UnsupportedOperationException();
    }

    @Override
    public long roundTripRequestDeadline() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setRoundTripRequestDeadline(long deadlineMillis) {
      throw new UnsupportedOperationException();
    }

    @Override
    public long roundTripRoutingBudgetMs() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setRoundTripRoutingBudgetMs(long budgetMs) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setRoundTripSearchRadius(double searchRadius) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setLastRoundTripResult(RoundTripResult result) {
      throw new UnsupportedOperationException();
    }

    @Override
    public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius) {
      throw new UnsupportedOperationException();
    }

    @Override
    public String startTileMissingError(OsmNodeNamed start) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void writeAdoptedTrackOutput(OsmTrack track) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void terminate() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void cleanupRoutingResources() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void logException(Throwable t) {
      throw new UnsupportedOperationException();
    }

    @Override
    public FastPlacementOps fastPlacementOps() {
      throw new UnsupportedOperationException();
    }

    @Override
    public double getRandomDirectionFromData(OsmNodeNamed wp, double searchRadius) {
      throw new UnsupportedOperationException();
    }

    @Override
    public RoundTripQualityResult boundedGateVerdict() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setBoundedGateVerdict(RoundTripQualityResult verdict) {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack[] greedyLegTracks() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setGreedyLegTracks(OsmTrack[] tracks) {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack lastRejectedTrack() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setLastRejectedTrack(OsmTrack track) {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean roundTripFerriesAllowed() {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean roundTripQualityHardReject(RoundTripQualityResult quality) {
      throw new UnsupportedOperationException();
    }

    @Override
    public long remainingRequestBudgetMs() {
      throw new UnsupportedOperationException();
    }

    @Override
    public java.io.File segmentDir() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void logThrowable(Throwable t) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setExplicitViaRoundTrip(boolean explicitVia) {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean roundTripForcedCorridorAccepted() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setRoundTripForcedCorridorAccepted(boolean accepted) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void addLinksProcessed(long links) {
      throw new UnsupportedOperationException();
    }

    @Override
    public IsochroneExpansionResult runIsochroneExpansion(OsmNodeNamed start, double searchRadius, OsmTrack refTrack, boolean includeCandidateTracks) {
      throw new UnsupportedOperationException();
    }

    @Override
    public btools.router.RoutingContext routingContext() {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean isTerminated() {
      throw new UnsupportedOperationException();
    }

    @Override
    public long startTime() {
      throw new UnsupportedOperationException();
    }

    @Override
    public long maxRunningTime() {
      throw new UnsupportedOperationException();
    }

    @Override
    public double roundTripSearchRadius() {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean isRoundTripMode() {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean explicitViaRoundTrip() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void recalcTrack(OsmTrack track) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void buildPointsFromCircle(List<OsmNodeNamed> waypoints, double startAngle,
                                      double searchRadius, int points) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void consolidateRoundTripVoiceHints(OsmTrack track) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setMatchedWaypoints(List<MatchedWaypoint> waypoints) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setStartTime(long startTimeMillis) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setMaxRunningTime(long maxRunningTimeMillis) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setTransientExpansionDeadline(long deadlineMillis) {
      throw new UnsupportedOperationException();
    }

    @Override
    public double airDistanceCostFactor() {
      throw new UnsupportedOperationException();
    }

    @Override
    public void setAirDistanceCostFactor(double factor) {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack findTrack(String operationName, MatchedWaypoint startWp,
                              MatchedWaypoint endWp, OsmTrack costCuttingTrack,
                              OsmTrack refTrack, boolean fastPartialRecalc) {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack retrackForDetail(OsmTrack rawTrack, MatchedWaypoint startWp,
                                     MatchedWaypoint endWp, OsmTrack refTrack) {
      throw new UnsupportedOperationException();
    }

    @Override
    public MatchedWaypoint profileAwareMatchPoint(int ilon, int ilat, String name,
                                                                   double maxSnapDist) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void resetCache(boolean detailed) {
      throw new UnsupportedOperationException();
    }

    @Override
    public OsmTrack findTrackUnguided(String operationName, MatchedWaypoint startWp,
                                      MatchedWaypoint endWp) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void matchWaypointsToNodes(List<MatchedWaypoint> waypoints, double maxDistance) {
      throw new UnsupportedOperationException();
    }
  }

  private static IsoCandidate isoAt(double airDist, double bearing, OsmTrack routed) {
    int[] pos = CheapRuler.destination(FROM_ILON, FROM_ILAT, airDist, bearing);
    int bucket = ((int) (bearing / 10.0)) % 36;
    return new IsoCandidate(pos[0], pos[1], bearing, airDist, (int) (airDist * 1.3),
      bucket, 5, 100, routed);
  }

  private static List<RoundTripCandidateProvider.CandidatePoint> query(
    List<IsoCandidate> pool, double airRadius) {
    return providerReturning(pool).candidatesForStep(
      FROM_ILON, FROM_ILAT, airRadius, 1, 5, FROM_ILON, FROM_ILAT, 0.0, null);
  }

  @Test
  public void buildTemplatesAppliesWindowBounds() {
    // Window is [0.5, 1.65] × airRadius. (HIGH 1.65 is deliberately wider than
    // the iso provider's 1.6 — pinned here so the divergence is intentional.)
    double airRadius = 2000;
    IsoCandidate keptNear = isoAt(2000, 90, null);   // 1.00× → kept
    IsoCandidate keptWide = isoAt(3250, 180, null);  // 1.625× → kept (iso would drop)
    List<IsoCandidate> pool = new ArrayList<>();
    pool.add(isoAt(800, 0, null));                   // 0.40× → dropped
    pool.add(keptNear);
    pool.add(keptWide);
    pool.add(isoAt(3400, 270, null));                // 1.70× → dropped

    List<RoundTripCandidateProvider.CandidatePoint> result = query(pool, airRadius);
    Assert.assertEquals("only the two in-window candidates survive", 2, result.size());
    java.util.Set<Integer> ilons = new java.util.HashSet<>();
    for (RoundTripCandidateProvider.CandidatePoint cp : result) ilons.add(cp.ilon);
    Assert.assertTrue("in-window near candidate kept", ilons.contains(keptNear.ilon));
    Assert.assertTrue("1.625× candidate kept (graph window is wider than iso)",
      ilons.contains(keptWide.ilon));
  }

  @Test
  public void buildTemplatesRanksByDistanceErrorAscending() {
    double airRadius = 2000;
    IsoCandidate near = isoAt(2000, 90, null);  // error ≈ 0
    IsoCandidate far = isoAt(3000, 180, null);  // error ≈ 1000
    List<IsoCandidate> pool = new ArrayList<>();
    pool.add(far);  // inserted first to prove the sort, not insertion order
    pool.add(near);

    List<RoundTripCandidateProvider.CandidatePoint> result = query(pool, airRadius);
    Assert.assertEquals(2, result.size());
    Assert.assertEquals("closest-to-target air distance ranks first",
      near.ilon, result.get(0).ilon);
  }

  @Test
  public void buildTemplatesDedupesIdenticalCells() {
    double airRadius = 2000;
    IsoCandidate a = isoAt(2000, 90, null);
    // Same position, different metadata → same dedupe cell → collapses to one.
    IsoCandidate dup = new IsoCandidate(a.ilon, a.ilat, a.bearingFromStart,
      a.airDistanceFromStart, 2600, a.bucket, 9, 75, null);
    List<IsoCandidate> pool = new ArrayList<>();
    pool.add(a);
    pool.add(dup);

    Assert.assertEquals("near-identical positions dedupe to one",
      1, query(pool, airRadius).size());
  }

  @Test
  public void buildTemplatesCapsAtCandidateCap() {
    double airRadius = 2000;
    List<IsoCandidate> pool = new ArrayList<>();
    for (int i = 0; i < 50; i++) {
      double dist = 1100 + i * 40;       // 1100..3060, all inside the window
      double bearing = (i * 37) % 360;   // spread → distinct positions/cells
      pool.add(isoAt(dist, bearing, null));
    }
    Assert.assertEquals("pool truncated to CANDIDATE_CAP (36)",
      36, query(pool, airRadius).size());
  }

  @Test
  public void buildTemplatesForwardsRoutedTrackAndZeroesIsoCost() {
    double airRadius = 2000;
    OsmTrack leg = new OsmTrack();
    IsoCandidate c = isoAt(2000, 90, leg);
    List<IsoCandidate> pool = new ArrayList<>();
    pool.add(c);

    List<RoundTripCandidateProvider.CandidatePoint> result = query(pool, airRadius);
    Assert.assertEquals(1, result.size());
    Assert.assertSame("the exact graph leg is forwarded for direct adoption",
      leg, result.get(0).routedTrack);
    Assert.assertEquals("start-centered iso-cost field is deliberately left unset",
      RoundTripCandidateProvider.NO_ISO_COST, result.get(0).costFromStart, 0.0);
  }
}
