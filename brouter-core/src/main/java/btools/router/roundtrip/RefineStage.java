package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;

/**
 * Orchestrator adapter for the round-trip refinement stage (§4.6, §4.7).
 */
public final class RefineStage {

  private RefineStage() {
  }

  /**
   * Main refinement hook invoked from RoundTripOrchestrator (§4.6).
   */
  public static void refine(RoundTripEngineOps ops,
                            RoundTripOrchestrator orchestrator,
                            RoundTripRequest request,
                            RoundTripQualityResult baselineQuality,
                            double searchRadius,
                            double direction,
                            RoundTripAlgorithm requestedAlgorithm) {
    if (ops == null || orchestrator == null || request == null) {
      return;
    }
    RefineDiagnostics diag = new RefineDiagnostics();
    long stageStart = System.currentTimeMillis();

    // 1. Eligibility & skip reason checks (§4.6)
    RefineConfig config = RefineConfig.fromContext(ops.routingContext());
    if (config.mode == RefineConfig.Mode.NONE) {
      diag.refineReason = "refine_off";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (ops.routingContext().roundTripSuppressDecoration) {
      diag.refineReason = "auto_child";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (request.isExplicitVia()) {
      diag.refineReason = "explicit_vias";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    boolean requestedAllowed = requestedAlgorithm == RoundTripAlgorithm.GREEDY
        || requestedAlgorithm == RoundTripAlgorithm.ISO_GREEDY
        || requestedAlgorithm == RoundTripAlgorithm.AUTO
        || requestedAlgorithm == RoundTripAlgorithm.QUALITY;
    boolean producingAllowed = request.producingTier == RoundTripAlgorithm.GREEDY
        || request.producingTier == RoundTripAlgorithm.ISO_GREEDY;
    if (!requestedAllowed || !producingAllowed) {
      diag.refineReason = "tier_not_supported";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (request.effortPolicy != null && !request.effortPolicy.refineAllowed) {
      diag.refineReason = "bounded_preset";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (ops.routingContext().allowSamewayback) {
      diag.refineReason = "samewayback";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (request.forcedCorridorAccepted) {
      diag.refineReason = "forced_corridor";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (baselineQuality == null || !baselineQuality.isAccepted()) {
      diag.refineReason = "gate_rejected";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    if (request.track == null || request.track.nodes == null || request.track.nodes.size() < 2) {
      diag.refineReason = "no_route";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    List<MatchedWaypoint> baselineWps = (request.track.getMatchedWaypoints() != null)
        ? request.track.getMatchedWaypoints()
        : ops.matchedWaypoints();
    RefineSkeleton skeleton = RefineInitializer.extractSkeleton(request.track, baselineWps);
    if (skeleton == null || skeleton.getVias().size() < 2) {
      diag.refineReason = "too_few_vias";
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      return;
    }

    // Catch any unexpected exception to preserve baseline route integrity
    try {
      long requestDeadline = request.requestDeadline();
      long stageDeadline = (requestDeadline > 0)
          ? Math.min(requestDeadline, stageStart + config.maxMs)
          : stageStart + config.maxMs;

      // 1. Initialize: re-route n raw legs (no refTrack)
      String profileName = ops.routingContext().getProfileName();
      double requestedDistance = 2 * Math.PI * searchRadius;
      LegEvaluator legEvaluator = new DefaultLegEvaluator(ops, "refine-leg");
      RefineInitResult initResult = RefineInitializer.initialize(
        ops, legEvaluator, skeleton, orchestrator.cleanup, request.track,
        searchRadius, profileName, direction, requestedDistance, stageDeadline);

      if (!initResult.isSuccess()) {
        String reason = initResult.getFailureReason();
        diag.refineReason = "init_failed: " + reason;
        if (reason != null && reason.contains("timeout")) {
          diag.refineTruncated = true;
          diag.timeoutOperation = "init";
        }
        diag.elapsedMs = System.currentTimeMillis() - stageStart;
        publishDiagnostics(ops, request, diag);
        return;
      }

      // Price baseline under oracle & calculate baseline RCS
      double baseOracleCost = initResult.getBaselineOracleCostPerMeter();
      if (baseOracleCost <= 0 || Double.isNaN(baseOracleCost)) {
        baseOracleCost = LoopCostOracle.price(ops, request.track, skeleton.getWaypoints());
      }

      RouteChoiceScore.Verdict baseRcsVerdict = RouteChoiceScore.score(
        request.track, requestedDistance, profileName, baselineQuality, direction);
      double baseRcs = (baseRcsVerdict != null) ? baseRcsVerdict.score() : 0.0;

      diag.oracleCostPerMeterBefore = baseOracleCost;
      diag.rcsBefore = baseRcs;
      diag.chains = config.chains;

      if (baseOracleCost <= 0 || Double.isNaN(baseOracleCost)) {
        diag.refineReason = "baseline_unpriceable";
        diag.elapsedMs = System.currentTimeMillis() - stageStart;
        publishDiagnostics(ops, request, diag);
        return;
      }

      FinishedCandidate baselineFinished = new FinishedCandidate(
        FinalizationOutcome.SUCCESS, "baseline", request.track,
        skeleton.getWaypoints(), baselineQuality, baseOracleCost, baseRcs);

      // 3. Search proposals on raw legs
      MoveProposalOperator moveOp = new MoveProposalOperator(ops, config);
      int varietySeed = Math.max(0, ops.routingContext().alternativeIdx);
      double rawBaselineEnergy = LoopCostOracle.price(ops, initResult.getRawLegs(), skeleton.getWaypoints());
      if (rawBaselineEnergy <= 0 || Double.isNaN(rawBaselineEnergy)) {
        rawBaselineEnergy = baseOracleCost;
      }
      RefineSearch search = new RefineSearch(
        ops, legEvaluator, initResult.getLegCache(), config, skeleton,
        initResult.getRawLegs(), rawBaselineEnergy, moveOp,
        searchRadius, requestedDistance, varietySeed, stageDeadline);

      RefineSearch.SearchResult searchResult = search.search(diag);
      List<RefineSearch.SearchCandidate> finalists = searchResult.getFinalists();

      if (finalists.isEmpty()) {
        if (diag.refineReason == null) {
          diag.refineReason = "no_feasible_evaluations";
        }
        diag.elapsedMs = System.currentTimeMillis() - stageStart;
        publishDiagnostics(ops, request, diag);
        return;
      }

      // 4. Finalize top-k candidates and evaluate ship predicate
      FinishedCandidate bestPassingCandidate = null;
      for (int i = 0; i < finalists.size(); i++) {
        if (ops != null && ops.isTerminated()) {
          diag.refineApplied = false;
          diag.refineReason = "cancelled";
          diag.elapsedMs = System.currentTimeMillis() - stageStart;
          publishDiagnostics(ops, request, diag);
          throw new IllegalArgumentException("operation killed by thread-priority-watchdog");
        }
        if (stageDeadline > 0 && System.currentTimeMillis() >= stageDeadline) {
          diag.refineTruncated = true;
          diag.timeoutOperation = "finalization";
          break;
        }

        RefineSearch.SearchCandidate rawCandidate = finalists.get(i);
        diag.finalizations++;

        FinishedCandidate candidate = RefineFinalizer.finalizeCandidate(
          rawCandidate.getRawLegs(), rawCandidate.getSkeleton(), ops,
          orchestrator.cleanup, searchRadius, profileName, direction,
          requestedDistance, stageDeadline);

        if (candidate.getOutcome() == FinalizationOutcome.SUCCESS) {
          ShipPredicate.Result shipResult = ShipPredicate.evaluate(
            candidate, baselineFinished, config, requestedDistance);
          if (shipResult.isAccepted()) {
            if (bestPassingCandidate == null
                || candidate.getOracleCostPerMeter() < bestPassingCandidate.getOracleCostPerMeter()) {
              bestPassingCandidate = candidate;
            }
          } else {
            if (diag.refineReason == null) {
              diag.refineReason = shipResult.getReason();
            }
          }
        } else if (candidate.getOutcome() == FinalizationOutcome.CANCELLED) {
          diag.refineApplied = false;
          diag.refineReason = "cancelled";
          diag.elapsedMs = System.currentTimeMillis() - stageStart;
          publishDiagnostics(ops, request, diag);
          throw new IllegalArgumentException("operation killed by thread-priority-watchdog");
        } else if (candidate.getOutcome() == FinalizationOutcome.TIMEOUT) {
          diag.refineTruncated = true;
          diag.timeoutOperation = "finalization";
        }
      }

      // 5. Publish winner atomically or leave baseline intact
      if (bestPassingCandidate != null) {
        diag.refineApplied = true;
        diag.refineReason = "accepted";
        diag.oracleCostPerMeterAfter = bestPassingCandidate.getOracleCostPerMeter();
        diag.rcsAfter = bestPassingCandidate.getRcs();
        diag.elapsedMs = System.currentTimeMillis() - stageStart;
        publish(ops, request, bestPassingCandidate, diag);
      } else {
        diag.refineApplied = false;
        if (diag.refineReason == null) {
          diag.refineReason = "predicate_rejected";
        }
        diag.elapsedMs = System.currentTimeMillis() - stageStart;
        publishDiagnostics(ops, request, diag);
      }
    } catch (RuntimeException e) {
      if ((ops != null && ops.isTerminated())
          || (e.getMessage() != null && e.getMessage().contains("thread-priority-watchdog"))) {
        throw e;
      }
      diag.refineApplied = false;
      diag.refineReason = "exception: " + e.getMessage();
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      if (ops != null) {
        ops.logInfo("RefineStage caught exception, shipping baseline unchanged: " + e.getMessage());
      }
    } catch (Exception e) {
      diag.refineApplied = false;
      diag.refineReason = "exception: " + e.getMessage();
      diag.elapsedMs = System.currentTimeMillis() - stageStart;
      publishDiagnostics(ops, request, diag);
      if (ops != null) {
        ops.logInfo("RefineStage caught exception, shipping baseline unchanged: " + e.getMessage());
      }
    }
  }

  /**
   * Atomically publish accepted candidate to engine state (§4.5).
   */
  public static void publish(RoundTripEngineOps ops, FinishedCandidate winner, RefineDiagnostics diag) {
    publish(ops, null, winner, diag);
  }

  /**
   * Atomically publish accepted candidate to engine and request state (§4.5).
   */
  public static void publish(RoundTripEngineOps ops, RoundTripRequest request,
                             FinishedCandidate winner, RefineDiagnostics diag) {
    if (ops != null) {
      ops.setLastRefineDiagnostics(diag);
    }
    if (winner == null || winner.getTrack() == null) {
      publishDiagnostics(ops, request, diag);
      return;
    }

    OsmTrack track = winner.getTrack();
    List<MatchedWaypoint> waypointsCopy = winner.getMatchedWaypoints() != null
        ? new ArrayList<>(winner.getMatchedWaypoints())
        : null;

    if (request != null) {
      request.track = track;
      request.qualityVerdict = winner.getQualityVerdict();
      request.deferredOutputWrite = true;
      if (request.lastResult != null) {
        request.lastResult.setTrack(track);
        if (waypointsCopy != null) {
          request.lastResult.setMatchedWaypoints(waypointsCopy);
        }
        request.lastResult.setTotalDistanceMeters((int) track.distance);
        request.lastResult.setRefineDiagnostics(diag);
      }
    }

    if (ops != null) {
      ops.setFoundTrack(track);
      ops.setLastRoundTripQuality(winner.getQualityVerdict());
      if (waypointsCopy != null) {
        ops.setMatchedWaypoints(new ArrayList<>(waypointsCopy));
        track.setMatchedWaypoints(new ArrayList<>(waypointsCopy));
      }
    }
  }

  /**
   * Publish diagnostics on rejection, skip, or failure leaving route state untouched (§4.5).
   */
  public static void publishDiagnostics(RoundTripRequest request, RefineDiagnostics diag) {
    publishDiagnostics(null, request, diag);
  }

  /**
   * Publish diagnostics to both ops and request on rejection, skip, or failure (§4.5).
   */
  public static void publishDiagnostics(RoundTripEngineOps ops, RoundTripRequest request, RefineDiagnostics diag) {
    if (ops != null) {
      ops.setLastRefineDiagnostics(diag);
    }
    if (request != null && request.lastResult != null) {
      request.lastResult.setRefineDiagnostics(diag);
    }
  }
}
