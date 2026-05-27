package btools.router;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/** Pure-function tests for {@link RoundTripQualityGate}. These tests construct
 * synthetic {@link OsmTrack} objects and verify the gate accepts good routes
 * and rejects every hard-fail case named in the production-safety spec. */
public class RoundTripQualityGateTest {

  // ---- Algorithm aliases sanity (also covered by RoundTripAlgorithmTest) ----

  @Test
  public void pavedProfileRecognition() {
    assertTrue(RoundTripQualityGate.isPavedProfile("fastbike"));
    assertTrue(RoundTripQualityGate.isPavedProfile("road"));
    assertTrue(RoundTripQualityGate.isPavedProfile("racing"));
    assertFalse(RoundTripQualityGate.isPavedProfile("gravel"));
    assertFalse(RoundTripQualityGate.isPavedProfile("mtb"));
    assertFalse(RoundTripQualityGate.isPavedProfile("trekking"));
    assertFalse(RoundTripQualityGate.isPavedProfile(null));
  }

  // ---- Happy path ---------------------------------------------------------

  @Test
  public void acceptsCleanCloseLoop() {
    OsmTrack good = squareLoop(/*sideMeters*/ 5000);
    // 4 edges × ~5km = ~20km loop — pass desired = actual so ratio = 1.0
    assertNull(RoundTripQualityGate.validate(good, good.distance, "fastbike"));
  }

  @Test
  public void acceptsAtExactRatioBoundary() {
    OsmTrack track = squareLoop(2500);
    // Reference the actual computed distance so the test is robust to
    // exact distance-formula differences. Test that ratio at MIN and MAX
    // boundaries is accepted.
    double dist = track.distance;
    double desiredAtMin = dist / RoundTripQualityGate.MIN_DISTANCE_RATIO;
    double desiredAtMax = dist / RoundTripQualityGate.MAX_DISTANCE_RATIO;
    assertNull("at MIN_DISTANCE_RATIO should pass",
      RoundTripQualityGate.validate(track, desiredAtMin, "fastbike"));
    assertNull("at MAX_DISTANCE_RATIO should pass",
      RoundTripQualityGate.validate(track, desiredAtMax, "fastbike"));
  }

  // ---- Hard-fail cases (one per spec criterion) ---------------------------

  @Test
  public void rejectsNullTrack() {
    assertNotNull(RoundTripQualityGate.validate(null, 20000, "fastbike"));
  }

  @Test
  public void rejectsTooFewNodes() {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    t.nodes.add(makeNode(0, 0));
    t.nodes.add(makeNode(1000, 0));
    t.distance = 1000;
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected too-few-nodes, got: " + reason, reason.contains("too few nodes"));
  }

  @Test
  public void rejectsClosureGapTooLarge() {
    OsmTrack t = squareLoop(5000);
    // Move the last node 500m away from the start
    OsmPathElement last = t.nodes.get(t.nodes.size() - 1);
    t.nodes.set(t.nodes.size() - 1, makeNodeRaw(last.getILon() + 5000, last.getILat()));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected closure rejection, got: " + reason, reason.contains("closure"));
  }

  @Test
  public void rejectsRatioBelowMin() {
    OsmTrack t = squareLoop(2000);  // 8km
    // 8km / 20km = 0.4 → below MIN_DISTANCE_RATIO 0.5
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected ratio rejection, got: " + reason, reason.contains("ratio"));
  }

  @Test
  public void rejectsRatioAboveMax() {
    OsmTrack t = squareLoop(5000); // 20km
    // 20km / 10km = 2.0 → above MAX_DISTANCE_RATIO 1.8
    String reason = RoundTripQualityGate.validate(t, 10000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected ratio rejection, got: " + reason, reason.contains("ratio"));
  }

  @Test
  public void rejectsExcessiveReuse() {
    // 4 edges out (1km each) + 4 edges back (re-traced) + 1 extra short edge
    // → 4/9 ≈ 44%? no, we need > 50%. Use 3 out + 3 back = 6 edges, 3 reused
    // = exactly 50% by edge count, but by distance-weighted reuse the first
    // visit isn't reuse so it's 3/6 = 50% — exactly at the boundary.
    // Push past with one extra reused edge.
    OsmTrack heavy = new OsmTrack();
    heavy.nodes = new ArrayList<>();
    heavy.nodes.add(makeNode(0, 0));
    heavy.nodes.add(makeNode(1000, 0));
    heavy.nodes.add(makeNode(2000, 0));
    heavy.nodes.add(makeNode(3000, 0));
    heavy.nodes.add(makeNode(2000, 0)); // reuse #1
    heavy.nodes.add(makeNode(1000, 0)); // reuse #2
    heavy.nodes.add(makeNode(2000, 0)); // reuse #3 (re-revisit of 1000-2000)
    heavy.nodes.add(makeNode(1000, 0)); // reuse #4 (re-revisit of 2000-1000)
    heavy.nodes.add(makeNode(0, 0));    // reuse #5 (0-1000)
    int hd = 0;
    for (int i = 1; i < heavy.nodes.size(); i++) {
      hd += heavy.nodes.get(i - 1).calcDistance(heavy.nodes.get(i));
      MessageData m = msgCostfactor(1.5f, "highway=residential surface=asphalt");
      heavy.nodes.get(i).message = m;
    }
    heavy.distance = hd;
    String reason = RoundTripQualityGate.validate(heavy, hd, "fastbike");
    assertNotNull(reason);
    // The new semantic classifier rejects this synthetic zigzag (an edge
    // visited 3+ times within a contiguous reuse stretch) as accidental
    // backtracking, not just "high reuse percentage". Either rejection
    // wording is acceptable — the contract is that the route is rejected.
    assertTrue("expected reuse/retrace rejection, got: " + reason,
      reason.contains("reuse") || reason.contains("retrace") || reason.contains("backtrack")
        || reason.contains("out-and-back"));
  }

  @Test
  public void rejectsBeelineMarkedWaypoint() {
    OsmTrack t = squareLoop(5000);
    t.matchedWaypoints = new ArrayList<>();
    MatchedWaypoint mwp = new MatchedWaypoint();
    mwp.wpttype = MatchedWaypoint.WAYPOINT_TYPE_DIRECT;
    t.matchedWaypoints.add(mwp);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected beeline rejection, got: " + reason, reason.contains("beeline"));
  }

  @Test
  public void rejectsDirectSegmentMarkerInTrackGeometry() {
    OsmTrack t = squareLoopWithMessage(5000,
      msgWayTags("highway=residential surface=asphalt direct_segment=true"));
    String reason = RoundTripQualityGate.validate(t, t.distance, "fastbike");
    assertNotNull(reason);
    assertTrue("expected direct_segment beeline rejection, got: " + reason,
      reason.contains("beeline") || reason.contains("direct_segment"));
  }

  @Test
  public void rejectsFerrySegmentByDefault() {
    OsmTrack t = squareLoopWithMessage(5000,
      msgWayTags("highway=service surface=asphalt route=ferry"));
    RoundTripQualityResult result = RoundTripQualityGate.evaluate(
      t, t.distance, "fastbike", false, false);
    assertFalse("ferry must reject by default", result.isAccepted());
    assertTrue("expected ferry rejection, got: " + result.getRejectionReason(),
      result.getRejectionReason().contains("ferry"));
  }

  @Test
  public void acceptsFerrySegmentWhenExplicitlyAllowed() {
    OsmTrack t = squareLoopWithMessage(5000,
      msgWayTags("highway=service surface=asphalt route=ferry"));
    RoundTripQualityResult result = RoundTripQualityGate.evaluate(
      t, t.distance, "fastbike", false, false, true);
    assertTrue("explicit ferry opt-in should leave clean geometry acceptable: "
      + result.getRejectionReason(), result.isAccepted());
  }

  @Test
  public void pavedProfileRejectsPathHeavyRoute() {
    OsmTrack t = squareLoopWithMessage(5000, msgWayTags("highway=path"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected hostile-segment rejection, got: " + reason,
      reason.contains("profile-hostile") || reason.contains("path/track/unpaved"));
  }

  @Test
  public void pavedProfileRejectsHighCostFactorSpike() {
    // 100% of edges have costfactor=10 (e.g. forced onto a grade-5 track)
    OsmTrack t = squareLoopWithMessage(5000, msgCostfactor(10.0f, "highway=residential"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected hostile rejection on cost-spike, got: " + reason,
      reason.contains("profile-hostile"));
  }

  // ---- Phase 2 v2 — worst-contiguous helper -------------------------------

  @Test
  public void worstContiguousHelperReturnsZeroForCleanTrack() {
    OsmTrack t = squareLoopWithMessage(5000, msgCostfactor(1.5f, "highway=residential"));
    assertEquals(0, RoundTripQualityGate.worstContiguousHostileMetersPaved(t));
  }

  @Test
  public void worstContiguousHelperFindsLongestStretch() {
    // Same metric the gate uses internally — verify the helper sees the
    // full hostile run and not just a single edge. Use the existing
    // trackWithContiguousHostile(total, contiguous, scattered) helper.
    OsmTrack t = trackWithContiguousHostile(20000, 800, 0);
    int worst = RoundTripQualityGate.worstContiguousHostileMetersPaved(t);
    // CheapRuler scaling means each 100m chunk → ~120m actual; 800 nominal
    // meters across 8 chunks → ~900-1000m measured. Accept a wide band.
    assertTrue("expected 700-1100m worst-contiguous, got " + worst,
      worst >= 700 && worst <= 1100);
  }

  @Test
  public void worstContiguousHelperHandlesEmptyTrack() {
    OsmTrack t = new OsmTrack();
    t.nodes = new java.util.ArrayList<>();
    assertEquals(0, RoundTripQualityGate.worstContiguousHostileMetersPaved(t));
  }

  @Test
  public void missingMetadataFractionCountsOnlyUnverifiedEdges() {
    OsmTrack t = squareLoop(5000);
    t.nodes.get(2).message = null;
    t.nodes.get(4).message = msgNoMetadata();
    assertEquals(0.5, RoundTripQualityGate.missingMetadataFraction(t), 0.05);
  }

  @Test
  public void pavedProfileRejectsRouteWithMissingMetadata() {
    // No per-edge messages — engine can't prove the edges are paved
    OsmTrack t = squareLoopNoMessage(5000);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected missing-metadata rejection, got: " + reason,
      reason.contains("missing/unknown metadata"));
  }

  @Test
  public void pavedProfileAcceptsCleanResidentialRoute() {
    // costfactor=1.5 (residential-ish), tags are clearly paved
    OsmTrack t = squareLoopWithMessage(5000, msgCostfactor(1.5f, "highway=residential"));
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNull("clean paved route should pass: " + reason, reason);
  }

  @Test
  public void nonPavedProfileSkipsHostilityCheck() {
    // Same path-heavy route that fastbike rejects → gravel/MTB accept it
    OsmTrack t = squareLoopWithMessage(5000, msgWayTags("highway=path"));
    assertNull(RoundTripQualityGate.validate(t, 20000, "gravel"));
    assertNull(RoundTripQualityGate.validate(t, 20000, "mtb"));
  }

  @Test
  public void pavedProfileToleratesMinorHostileSegments() {
    // 95% residential, 5% path → below MAX_HOSTILE_FRACTION (10%)
    OsmTrack t = mixedSurfaceLoop(/*sideMeters*/ 5000, /*hostileFractionPct*/ 5);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNull("5% path content should pass: " + reason, reason);
  }

  // ---- Phase 1.1 — asphalt-surface exemption ----------------------------

  @Test
  public void pathWithAsphaltSurfaceIsNotHostile() {
    // Common OSM mis-tagging: rural cycleway tagged highway=path but
    // surface=asphalt. Paved infrastructure — must pass.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=path surface=asphalt";
    m.costfactor = 1.5f;
    assertFalse("path + asphalt is paved cycleway, not hostile",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pathWithPavedSurfaceIsNotHostile() {
    // surface=paved is the generic version of surface=asphalt.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=footway surface=paved";
    m.costfactor = 1.5f;
    assertFalse("footway + paved is paved cycleway, not hostile",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pathWithoutSurfaceTagStaysHostile() {
    // No surface= tag — engine can't prove it's paved; keep the
    // conservative behaviour (treat as hostile).
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=path";
    m.costfactor = 1.5f;
    assertTrue("path without surface tag stays hostile",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pathWithGravelSurfaceStaysHostile() {
    // surface=gravel is NOT a hard surface — must remain hostile even
    // though highway=path is in the overridable list.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=path surface=gravel";
    m.costfactor = 1.5f;
    assertTrue("path + gravel is unpaved, hostile",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void plainTrackWithAsphaltSurfaceIsNotHostile() {
    // B.2 evidence: cyclists ride asphalt-surfaced tracks routinely.
    // 2+ Freiburg routes share the Kandel "Saubergweg" tagged exactly
    // this way (highway=track surface=asphalt, no tracktype), and
    // adjacent ways are paved service roads (highway=service
    // surface=asphalt). Asphalt is asphalt; tracktype absence is just
    // OSM under-tagging.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt";
    m.costfactor = 1.5f;
    assertFalse("plain track + asphalt is paved infrastructure, not hostile",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade1BicycleTrackIsNotHostile() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1 bicycle=yes";
    m.costfactor = 1.5f;
    assertFalse("paved grade1 bicycle track is road-bike suitable",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade1CycleRouteTrackIsNotHostileEvenWithHighCost() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1 route_bicycle_lcn=yes";
    m.costfactor = 8.0f;
    assertFalse("official paved grade1 cycle-route track should override cost spike",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade2BicycleTrackStaysHostileForRoadBike() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade2 bicycle=yes";
    m.costfactor = 1.5f;
    assertTrue("grade2 track is not the narrow road-bike exemption",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  // ---- Phase 2 v5 — paved grade1 track without explicit bicycle tag ------

  @Test
  public void pavedGrade1TrackWithoutBicycleTagIsNotHostile() {
    // GPX-replay evidence (52 Basel routes, 8 false positives) shows
    // cyclists routinely ride paved grade1 tracks that OSM doesn't
    // bother tagging with bicycle=*. The Seewen road (47.5125, 7.6524)
    // appears in 4 different road-bike GPXs.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1";
    m.costfactor = 1.5f;
    assertFalse("paved grade1 track without explicit restriction must be acceptable: "
      + m.wayKeyValues, RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade1TrackWithFootPermissiveIsNotHostile() {
    // The Seewen-area pattern from the Basel corpus: foot=permissive
    // but no bicycle=*. Default-permissive tagging convention.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1 foot=permissive";
    m.costfactor = 1.5f;
    assertFalse("paved grade1 + foot=permissive should be acceptable",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade1TrackWithBicycleNoStaysHostile() {
    // Explicit bicycle=no restriction must still trip the gate even on
    // a paved grade1 track — the relaxation is not unconditional.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1 bicycle=no";
    m.costfactor = 1.5f;
    assertTrue("explicit bicycle=no restriction must override relaxation",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pavedGrade1TrackWithAccessPrivateStaysHostile() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=asphalt tracktype=grade1 access=private";
    m.costfactor = 1.5f;
    assertTrue("access=private must override relaxation",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  // ---- Phase 2 v5 (B.1 cascade) — grade1 + cycle network → paved -----------

  @Test
  public void grade1TrackOnCycleNetworkWithoutSurfaceIsNotHostile() {
    // B.1 evidence: Baar grade1+lcn track appears in 2 different
    // Freiburg routes. OSM grade1 = "Solid hard surface, typically
    // tarmac/asphalt/concrete" and the cycle-network tag is curated
    // evidence the way is rideable.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track tracktype=grade1 route_bicycle_lcn=yes";
    m.costfactor = 1.5f;
    assertFalse("grade1 + local cycle network is paved cycle infrastructure",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void grade1TrackOnRegionalCycleNetworkIsNotHostile() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track tracktype=grade1 route_bicycle_rcn=yes";
    m.costfactor = 1.5f;
    assertFalse("grade1 + regional cycle network is paved cycle infrastructure",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void grade1TrackWithoutCycleNetworkOrSurfaceStaysHostile() {
    // Without ANY positive evidence (no surface, no cycle network) and
    // only grade1, we keep the conservative default. Grade1 alone is a
    // hint, not enough; the predicate needs at least one corroborating
    // signal.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track tracktype=grade1";
    m.costfactor = 1.5f;
    assertTrue("grade1 alone, no surface, no network → keep conservative reject",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void grade2CycleNetworkTrackStaysHostile() {
    // B.1 is grade1-only. Grade2 (compacted/gravel cycle routes) is
    // not road-bike-suitable even with a cycle-network tag — the
    // Freiburg "Rinken" road appears in 4 routes tagged
    // grade2+compacted+lcn and is correctly rejected.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=track surface=compacted tracktype=grade2 route_bicycle_lcn=yes";
    m.costfactor = 1.5f;
    assertTrue("grade2 + cycle network is gravel-bike-suitable, not road-bike",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  // ---- Phase 1.2 — cobblestone/pebblestone are paved (rough, rideable) --

  @Test
  public void cobblestoneIsNotHostile() {
    // Rough but rideable — German pedestrian zones, Belgian cycleways.
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=residential surface=cobblestone";
    m.costfactor = 1.5f;
    assertFalse("cobblestone is rough-but-rideable paved",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  @Test
  public void pebblestoneIsNotHostile() {
    MessageData m = new MessageData();
    m.wayKeyValues = "highway=residential surface=pebblestone";
    m.costfactor = 1.5f;
    assertFalse("pebblestone is rough-but-rideable paved",
      RoundTripQualityGate.isHostileForPavedProfile(m));
  }

  // ---- Phase 1.3 — contiguous-hostile sub-cap ---------------------------

  @Test
  public void shortContiguousHostileRunBelowCapAccepted() {
    // ~6% total hostile (1200m of 20km), longest contiguous run only 400m
    // → accept (under both total-fraction and contiguous caps).
    OsmTrack t = trackWithContiguousHostile(20000, 400, 800);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNull("short contiguous hostile under sub-cap: " + reason, reason);
  }

  @Test
  public void longContiguousHostileRunAboveCapRejected() {
    // ~9% total hostile, longest contiguous run 1800m → reject with
    // "contiguous" prefix (over the 1500m sub-cap) even though total
    // fraction is below the 10% cap.
    OsmTrack t = trackWithContiguousHostile(20000, 1800, 0);
    String reason = RoundTripQualityGate.validate(t, 20000, "fastbike");
    assertNotNull("long contiguous hostile must reject", reason);
    assertTrue("rejection prefix is 'contiguous': " + reason,
      reason.startsWith("contiguous "));
  }

  @Test
  public void worstHostileStretchReportsCoordinatesAndTags() {
    OsmTrack t = trackWithContiguousHostile(20000, 1800, 0);

    RoundTripQualityGate.HostileStretch stretch =
      RoundTripQualityGate.worstHostileStretchPaved(t);

    assertTrue(stretch.isPresent());
    assertTrue("expected hostile stretch near requested length, got " + stretch.meters,
      stretch.meters >= 1700);
    assertEquals(0, stretch.startIndex);
    assertTrue(stretch.endIndex > stretch.startIndex);
    assertEquals(t.nodes.get(stretch.startIndex).getILon(), stretch.startIlon);
    assertEquals(t.nodes.get(stretch.startIndex).getILat(), stretch.startIlat);
    assertEquals(t.nodes.get(stretch.endIndex).getILon(), stretch.endIlon);
    assertEquals(t.nodes.get(stretch.endIndex).getILat(), stretch.endIlat);
    assertTrue(stretch.startTags.contains("highway=path"));
    assertTrue(stretch.endTags.contains("surface=ground"));
    assertTrue(stretch.describe().contains("highway=path"));
  }

  @Test
  public void totalFractionRejectionPathRemainsActive() {
    // ~15% total hostile but contiguous bursts under 1500m. The
    // contiguous sub-cap doesn't fire (≤ 1500m), so the existing total-
    // fraction rejection takes over. Helper chunks are ~120m (not 100m)
    // due to CheapRuler scaling at 50°N, so 1000m of declared contiguous
    // turns into ~1200m actual — comfortably under the 1500m sub-cap.
    OsmTrack t = trackWithContiguousHostile(40000, 1000, 5000);
    String reason = RoundTripQualityGate.validate(t, 40000, "fastbike");
    assertNotNull(reason);
    assertTrue("expected total-fraction prefix: " + reason,
      reason.contains("of distance on profile-hostile ways"));
  }

  // ---- helpers ------------------------------------------------------------

  /** Square loop with 4 edges of {@code side} meters each — total ~4×side.
   * Edges carry clean residential metadata so the paved-profile gate accepts
   * the route unless the test deliberately overrides it. */
  private static OsmTrack squareLoop(int side) {
    OsmTrack t = squareLoopNoMessage(side);
    MessageData clean = msgCostfactor(1.5f, "highway=residential surface=asphalt");
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = clone(clean);
    }
    return t;
  }

  /** Same geometry but without per-edge messages — used by the missing-metadata
   * test to verify the gate rejects routes whose paved-ness can't be proven. */
  private static OsmTrack squareLoopNoMessage(int side) {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    t.nodes.add(makeNode(0, 0));
    t.nodes.add(makeNode(side, 0));
    t.nodes.add(makeNode(side, side));
    t.nodes.add(makeNode(0, side));
    t.nodes.add(makeNode(0, 0)); // close
    int dist = 0;
    for (int i = 1; i < t.nodes.size(); i++) {
      dist += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
    }
    t.distance = dist;
    return t;
  }

  /** Square loop where every edge carries the supplied {@link MessageData}. */
  private static OsmTrack squareLoopWithMessage(int side, MessageData m) {
    OsmTrack t = squareLoop(side);
    for (int i = 1; i < t.nodes.size(); i++) {
      t.nodes.get(i).message = clone(m);
    }
    return t;
  }

  /**
   * Build a synthetic clean loop (square geometry, every edge unique) of
   * approximately {@code totalMeters} total length, then tag a contiguous
   * run of {@code contiguousHostileMeters} along one side and scatter
   * {@code scatteredHostileMeters} of additional hostile bursts on the
   * opposite side (broken up by non-hostile edges to avoid extending the
   * contiguous run). The geometry is loop-shaped to avoid tripping the
   * reuse classifier as SCENIC_OUT_AND_BACK.
   */
  private static OsmTrack trackWithContiguousHostile(int totalMeters,
                                                     int contiguousHostileMeters,
                                                     int scatteredHostileMeters) {
    // 4-sided square loop with edges subdivided into 100m chunks. Each side
    // is totalMeters / 4 long; we choose chunk count per side to give
    // fine-grained tagging control while keeping geometry clearly loop-like.
    int side = totalMeters / 4;
    int chunkLen = 100;
    int chunksPerSide = side / chunkLen;
    if (chunksPerSide < 1) chunksPerSide = 1;

    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    // Four sides of the square, with intermediate chunk nodes:
    //   (0,0) → (side,0) → (side,side) → (0,side) → (0,0)
    addSquareSide(t, 0, 0, +chunkLen, 0, chunksPerSide); // east side
    addSquareSide(t, side, 0, 0, +chunkLen, chunksPerSide); // north side
    addSquareSide(t, side, side, -chunkLen, 0, chunksPerSide); // west side
    addSquareSide(t, 0, side, 0, -chunkLen, chunksPerSide); // south side
    t.nodes.add(makeNode(0, 0)); // close the loop

    int contigEdges = Math.min(contiguousHostileMeters / chunkLen, chunksPerSide);
    int scatteredEdges = Math.min(scatteredHostileMeters / chunkLen, chunksPerSide);
    // Place the contiguous hostile run at the START of the east side.
    // Place the scattered hostile bursts on the west side, every-other
    // chunk, so each is separated by a non-hostile edge.
    int dist = 0;
    int totalEdges = t.nodes.size() - 1;
    int westSideStart = 2 * chunksPerSide + 1;
    int westSideEnd = 3 * chunksPerSide;
    for (int i = 1; i < t.nodes.size(); i++) {
      dist += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
      boolean isContigHostile = i <= contigEdges;
      int westIdx = i - westSideStart;
      boolean isScatteredHostile = westIdx >= 0 && westIdx < chunksPerSide
        && westIdx % 2 == 0 && (westIdx / 2) < scatteredEdges;
      MessageData m = (isContigHostile || isScatteredHostile)
        ? msgWayTags("highway=path surface=ground")
        : msgCostfactor(1.0f, "highway=residential surface=asphalt");
      t.nodes.get(i).message = m;
    }
    t.distance = dist;
    return t;
  }

  /** Push {@code n} segments of (dx, dy) starting from (startX, startY)
   * into the track. The starting node is added only if the track is empty
   * (other sides chain off the previous one's endpoint). */
  private static void addSquareSide(OsmTrack t, int startX, int startY,
                                    int dx, int dy, int n) {
    if (t.nodes.isEmpty()) t.nodes.add(makeNode(startX, startY));
    for (int i = 1; i <= n; i++) {
      t.nodes.add(makeNode(startX + dx * i, startY + dy * i));
    }
  }

  /** Square loop where the first {@code hostileFractionPct}% of edges are
   * path/track and the rest are residential. */
  private static OsmTrack mixedSurfaceLoop(int side, int hostileFractionPct) {
    OsmTrack t = squareLoop(side);
    // 4 edges; flip k of them to "hostile" based on percentage
    int hostileEdges = (4 * hostileFractionPct) / 100;
    for (int i = 1; i < t.nodes.size(); i++) {
      MessageData m = (i <= hostileEdges)
        ? msgWayTags("highway=path")
        : msgCostfactor(1.5f, "highway=residential");
      t.nodes.get(i).message = m;
    }
    return t;
  }

  private static OsmPathElement makeNode(int x, int y) {
    // ~1 degree ≈ 111km at equator; use small offsets so distances are roughly
    // x and y in meters. CheapRuler is what does the actual scaling.
    return makeNodeRaw(180000000 + x * 14, 50000000 + y * 9);
  }

  private static OsmPathElement makeNodeRaw(int ilon, int ilat) {
    return OsmPathElement.create(ilon, ilat, (short) 0, null);
  }

  private static MessageData msgWayTags(String tags) {
    MessageData m = new MessageData();
    m.wayKeyValues = tags;
    m.costfactor = 1.5f;
    return m;
  }

  private static MessageData msgCostfactor(float cf, String tags) {
    MessageData m = new MessageData();
    m.wayKeyValues = tags;
    m.costfactor = cf;
    return m;
  }

  /** Missing-metadata placeholder: wayKeyValues=null treated as suspect. */
  private static MessageData msgNoMetadata() {
    MessageData m = new MessageData();
    m.wayKeyValues = null;
    m.costfactor = 0;
    return m;
  }

  private static MessageData clone(MessageData src) {
    MessageData m = new MessageData();
    m.wayKeyValues = src.wayKeyValues;
    m.costfactor = src.costfactor;
    return m;
  }
}
