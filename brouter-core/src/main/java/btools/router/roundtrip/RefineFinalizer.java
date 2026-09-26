package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;

/**
 * Pipeline for full candidate finalization (§4.7).
 */
public final class RefineFinalizer {

  /** Finalization steps where failure can be simulated in tests. */
  public enum FinalizationStep {
    NONE,
    RETRACK,
    MERGE,
    CLEANUP,
    GATE,
    PRICING,
    RCS
  }

  /** Hook for failure injection in unit/integration tests. */
  static FinalizationStep failureInjectionStep = FinalizationStep.NONE;
  static boolean failureInjectionTimeout = false;

  public static void setFailureInjection(FinalizationStep step, boolean timeout) {
    failureInjectionStep = step;
    failureInjectionTimeout = timeout;
  }

  public static void resetFailureInjection() {
    failureInjectionStep = FinalizationStep.NONE;
    failureInjectionTimeout = false;
  }

  private RefineFinalizer() {
  }

  /**
   * Finalize a raw candidate into a fully detailed, cleaned, gated, and priced loop.
   */
  public static FinishedCandidate finalizeCandidate(
      List<OsmTrack> rawLegs,
      RefineSkeleton skeleton,
      RoundTripEngineOps ops,
      RoundTripTrackCleanup cleanup,
      double searchRadius,
      String profileName,
      double requestedDirection,
      double requestedDistance,
      long deadlineMs) {

    return finalizeCandidate(rawLegs, skeleton, ops, cleanup, searchRadius, profileName,
      requestedDirection, requestedDistance, deadlineMs, null);
  }

  public static FinishedCandidate finalizeCandidate(List<OsmTrack> rawLegs, RefineSkeleton skeleton,
      RoundTripEngineOps ops, RoundTripTrackCleanup cleanup, double searchRadius, String profileName,
      double requestedDirection, double requestedDistance, long deadlineMs, RefineDiagnostics diag) {
    if (ops != null && ops.isTerminated()) {
      return new FinishedCandidate(FinalizationOutcome.CANCELLED, "terminated", null, null, null, -1.0, -1.0);
    }

    long now = System.currentTimeMillis();
    if (deadlineMs > 0 && now >= deadlineMs) {
      return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "timeout_before_start", null, null, null, -1.0, -1.0);
    }

    if (rawLegs == null || rawLegs.isEmpty() || skeleton == null) {
      return new FinishedCandidate(FinalizationOutcome.FAILURE, "invalid_arguments", null, null, null, -1.0, -1.0);
    }

    List<MatchedWaypoint> waypoints = skeleton.getWaypoints();
    if (waypoints.size() < 2 || rawLegs.size() != waypoints.size() - 1) {
      return new FinishedCandidate(FinalizationOutcome.FAILURE, "leg_waypoint_mismatch", null, null, null, -1.0, -1.0);
    }

    // Isolate engine state: save original matched waypoints before any speculative cleanup mutation
    List<MatchedWaypoint> originalWps = (ops != null && ops.matchedWaypoints() != null)
        ? new ArrayList<>(ops.matchedWaypoints())
        : null;

    try (RefineBudget budget = new RefineBudget(ops, deadlineMs)) {
      budget.check();
      // Step 1: Retrack for detail
      if (failureInjectionStep == FinalizationStep.RETRACK) {
        if (failureInjectionTimeout) {
          return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "injected_retrack_timeout", null, null, null, -1.0, -1.0);
        }
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_retrack_failure", null, null, null, -1.0, -1.0);
      }

      List<OsmTrack> detailedLegs = new ArrayList<>(rawLegs.size());
      for (int l = 0; l < rawLegs.size(); l++) {
        if (ops != null && ops.isTerminated()) {
          return new FinishedCandidate(FinalizationOutcome.CANCELLED, "terminated", null, null, null, -1.0, -1.0);
        }
        if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
          return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "retrack_timeout", null, null, null, -1.0, -1.0);
        }
        OsmTrack raw = rawLegs.get(l);
        if (raw == null || raw.nodes == null || raw.nodes.size() < 2) {
          return new FinishedCandidate(FinalizationOutcome.FAILURE, "invalid_raw_leg", null, null, null, -1.0, -1.0);
        }
        OsmTrack rawCopy = copyTrack(raw);
        MatchedWaypoint from = waypoints.get(l);
        MatchedWaypoint to = waypoints.get(l + 1);

        OsmTrack det;
        try (RefineHeading heading = new RefineHeading(ops.routingContext(), l == 0)) {
          long legBudget = (deadlineMs > 0) ? (deadlineMs - System.currentTimeMillis()) : -1L;
          long routeStart = System.currentTimeMillis();
          try {
            det = ops.retrackForDetail(rawCopy, from, to, null, legBudget);
          } finally {
            if (diag != null) diag.routingMs += System.currentTimeMillis() - routeStart;
          }
        } catch (RefineBudget.Exceeded e) {
          throw e;
        } catch (IllegalArgumentException e) {
          if (ops != null && ops.isTerminated()) {
            throw e;
          }
          if (e.getMessage() != null && e.getMessage().contains("thread-priority-watchdog")) {
            throw e; // propagation of watchdog cancellation
          }
          if (e.getMessage() != null && e.getMessage().contains("timeout")) {
            return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "retrack_timeout", null, null, null, -1.0, -1.0);
          }
          return new FinishedCandidate(FinalizationOutcome.FAILURE, "retrack_exception: " + e.getMessage(), null, null, null, -1.0, -1.0);
        }

        if (ops != null && ops.isTerminated()) {
          return new FinishedCandidate(FinalizationOutcome.CANCELLED, "terminated", null, null, null, -1.0, -1.0);
        }
        if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
          return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "retrack_timeout", null, null, null, -1.0, -1.0);
        }

        if (det == null) {
          return new FinishedCandidate(FinalizationOutcome.FAILURE, "retrack_null", null, null, null, -1.0, -1.0);
        }
        // §4.5 / M0.3: "A raw-geometry fallback from retrackForDetail is not a successful finalization."
        if (det == rawCopy) {
          return new FinishedCandidate(FinalizationOutcome.FAILURE, "retrack_raw_fallback", null, null, null, -1.0, -1.0);
        }
        detailedLegs.add(det);
      }

      // Step 2: Merge legs preserving time/energy offsets, detours, and voice hints
      if (failureInjectionStep == FinalizationStep.MERGE) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_merge_failure", null, null, null, -1.0, -1.0);
      }
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "merge_timeout", null, null, null, -1.0, -1.0);
      }

      OsmTrack merged = new OsmTrack();
      for (int l = 0; l < detailedLegs.size(); l++) {
        merged.appendTrack(detailedLegs.get(l));
      }

      // Step 3: Post-routing cleanup
      if (failureInjectionStep == FinalizationStep.CLEANUP) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_cleanup_failure", null, null, null, -1.0, -1.0);
      }
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "cleanup_timeout", null, null, null, -1.0, -1.0);
      }

      List<MatchedWaypoint> waypointsCopy = new ArrayList<>(waypoints.size());
      for (MatchedWaypoint wp : waypoints) {
        waypointsCopy.add(RefineSkeleton.copyWaypoint(wp));
      }

      try {
        long cleanupStart = System.currentTimeMillis();
        try {
          cleanup.finalizeAdoptedRoundTripTrack(merged, waypointsCopy, budget::check);
        } finally {
          if (diag != null) diag.cleanupMs += System.currentTimeMillis() - cleanupStart;
        }
      } catch (RefineBudget.Exceeded e) {
        throw e;
      } catch (IllegalArgumentException e) {
        if (ops != null && ops.isTerminated()) {
          throw e;
        }
        if (e.getMessage() != null && e.getMessage().contains("thread-priority-watchdog")) {
          throw e;
        }
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "cleanup_exception: " + e.getMessage(), null, null, null, -1.0, -1.0);
      }

      // Step 4: Quality gate with exact orchestrator context (§4.5, §4.6)
      if (failureInjectionStep == FinalizationStep.GATE) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_gate_failure", null, null, null, -1.0, -1.0);
      }
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "gate_timeout", null, null, null, -1.0, -1.0);
      }

      RoundTripQualityResult quality;
      try {
        boolean allowSamewayback = ops.routingContext().allowSamewayback;
        boolean explicitViaMode = ops.explicitViaRoundTrip();
        boolean pavedProfile = RoundTripQualityGate.classifyPavedProfile(ops.routingContext().expctxWay);
        boolean ferriesAllowed = ops.roundTripFerriesAllowed();
        double targetDistance = 2 * Math.PI * searchRadius;
        quality = RoundTripQualityGate.evaluate(merged, targetDistance,
          pavedProfile, allowSamewayback, explicitViaMode, ferriesAllowed);
      } catch (RefineBudget.Exceeded e) {
        throw e;
      } catch (Exception e) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "gate_exception: " + e.getMessage(), null, null, null, -1.0, -1.0);
      }

      budget.check();
      if (quality == null || !quality.isAccepted()) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE,
          "gate_rejected: " + (quality != null ? quality.getRejectionReason() : "null"),
          merged, waypointsCopy, quality, -1.0, -1.0);
      }

      // Step 5: Pricing & RCS
      if (failureInjectionStep == FinalizationStep.PRICING) {
        if (failureInjectionTimeout) {
          return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "injected_pricing_timeout", null, null, null, -1.0, -1.0);
        }
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_pricing_failure", null, null, null, -1.0, -1.0);
      }
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "pricing_timeout", null, null, null, -1.0, -1.0);
      }

      long priceStart = System.currentTimeMillis();
      LoopPrice price = LoopCostOracle.evaluate(ops, merged, waypointsCopy, deadlineMs);
      if (diag != null) diag.pricingMs += System.currentTimeMillis() - priceStart;
      double oracleCost = price.costPerMeter();

      if (ops != null && ops.isTerminated()) {
        return new FinishedCandidate(FinalizationOutcome.CANCELLED, "terminated", null, null, null, -1.0, -1.0);
      }
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        return new FinishedCandidate(FinalizationOutcome.TIMEOUT, "pricing_timeout", null, null, null, -1.0, -1.0);
      }

      if (oracleCost <= 0 || Double.isNaN(oracleCost)) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "pricing_failed", merged, waypointsCopy, quality, -1.0, -1.0);
      }

      if (failureInjectionStep == FinalizationStep.RCS) {
        return new FinishedCandidate(FinalizationOutcome.FAILURE, "injected_rcs_failure", null, null, null, -1.0, -1.0);
      }

      double rcsScore = 0.0;
      try {
        RouteChoiceScore.Verdict rcsVerdict = RouteChoiceScore.score(merged, requestedDistance,
          profileName, quality, requestedDirection);
        if (rcsVerdict != null) {
          rcsScore = rcsVerdict.score();
        }
      } catch (RefineBudget.Exceeded e) {
        throw e;
      } catch (Exception e) {
        rcsScore = 0.0;
      }
      if (Double.isNaN(rcsScore)) {
        rcsScore = 0.0;
      }

      budget.check();
      String pricingMethod = price.method();
      return new FinishedCandidate(FinalizationOutcome.SUCCESS, "ok", merged, waypointsCopy,
        quality, oracleCost, rcsScore, pricingMethod);
    } catch (RefineBudget.Exceeded e) {
      return new FinishedCandidate(e.outcome, e.getMessage(), null, null, null, -1.0, -1.0);
    } finally {
      // Restore engine's original matched waypoints: speculative candidate evaluation never mutates engine state!
      if (ops != null) {
        ops.setMatchedWaypoints(originalWps);
      }
    }
  }

  private static OsmTrack copyTrack(OsmTrack src) {
    OsmTrack copy = new OsmTrack();
    copy.cost = src.cost;
    copy.distance = src.distance;
    if (src.nodes != null) {
      for (OsmPathElement pe : src.nodes) {
        copy.nodes.add(OsmPathElement.create(pe.getILon(), pe.getILat(), pe.getSElev(), null));
      }
    }
    return copy;
  }
}
