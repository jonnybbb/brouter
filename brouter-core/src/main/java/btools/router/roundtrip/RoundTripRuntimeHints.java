package btools.router.roundtrip;

import btools.router.OsmTrack;

/**
 * Immutable snapshot of the request fields the engine's SEARCH LOOPS read
 * mid-search — the round-trip subsystem's single publication into the engine
 * (see {@link RoundTripRequestState#setRoundTripRuntimeHints}). Everything
 * else about a request stays orchestrator-owned in {@code RoundTripRequest}.
 */
public final class RoundTripRuntimeHints {

  /** Active search radius (matching hooks); 0 outside round-trip requests. */
  public final double searchRadius;

  /** Wall-clock deadline (epoch ms) for the whole request; 0 = unbounded. */
  public final long requestDeadline;

  /** True while an explicit-via round trip is being generated. */
  public final boolean explicitViaRoundTrip;

  /** Per-leg guide tracks from a greedy adoption; consulted by doRouting. */
  public final OsmTrack[] greedyLegTracks;

  public RoundTripRuntimeHints(double searchRadius, long requestDeadline,
                               boolean explicitViaRoundTrip, OsmTrack[] greedyLegTracks) {
    this.searchRadius = searchRadius;
    this.requestDeadline = requestDeadline;
    this.explicitViaRoundTrip = explicitViaRoundTrip;
    // Defensive copy: the snapshot must not change when the caller reuses its
    // array (the tracks themselves stay shared — the engine reads them only
    // as per-leg cost-cutting guides).
    this.greedyLegTracks = greedyLegTracks == null ? null : greedyLegTracks.clone();
  }
}
