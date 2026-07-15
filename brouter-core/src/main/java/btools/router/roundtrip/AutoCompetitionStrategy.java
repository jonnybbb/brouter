package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import btools.mapaccess.OsmNode;
import btools.router.OsmNodeNamed;
import btools.router.OsmTrack;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

/**
 * AUTO candidate competition tier: runs candidate algorithms in isolated
 * child engines, scores the gated results, and adopts the winner (or the
 * least-bad best-effort track in lenient mode). Self-finalizing — candidates
 * are gated inside the competition and the winner is decorated on adoption,
 * so the outcome does NOT pass the orchestrator's shared gate again. QUALITY
 * is this strategy pinned to the MAX preset (see the ladder resolution).
 */
final class AutoCompetitionStrategy implements RoundTripStrategy {

  private final RoundTripOrchestrator orchestrator;
  private final RoundTripEngineOps ops;

  AutoCompetitionStrategy(RoundTripOrchestrator orchestrator) {
    this.orchestrator = orchestrator;
    this.ops = orchestrator.ops;
  }

  @Override
  public boolean attempt(RoundTripRequest request, TierSlice slice) {
    request.effortPolicy = slice.effortPolicy;
    runAutoCandidateCompetition(slice.searchRadius, slice.direction);
    return false;
  }

  private static final long DEFAULT_AUTO_BUDGET_MS = 60_000;

  /**
   * Grace (ms) the request thread waits past a parallel AUTO GREEDY child's own
   * budget before terminating it — bounds the join so a wedged or overshooting
   * child can never hang the request thread.
   */
  private static final long AUTO_CHILD_JOIN_UNWIND_MS = 3_000;

  /**
   * When false (default), do not start plain GREEDY speculatively before
   * ISO_GREEDY proves it is needed — avoids duplicate runs on strong or
   * graph-native-absorbed ISO_GREEDY results. Opt back into the old lower-latency
   * tradeoff with {@code -DroundTripSpeculativeAutoGreedy=true}.
   */
  private static final boolean SPECULATIVE_AUTO_GREEDY =
    Boolean.getBoolean("roundTripSpeculativeAutoGreedy");

  /**
   * JVM-wide permit pool capping how many AUTO requests run their speculative
   * GREEDY child in parallel when {@link #SPECULATIVE_AUTO_GREEDY} is on — routing
   * is CPU-bound, so this bounds the extra threads. Tune via
   * {@code -DroundTripParallelAutoPermits}; 0 forces fully-sequential AUTO.
   */
  private static final java.util.concurrent.Semaphore PARALLEL_AUTO_SEMAPHORE =
    new java.util.concurrent.Semaphore(Math.max(0,
      Integer.getInteger("roundTripParallelAutoPermits",
        Runtime.getRuntime().availableProcessors() - 1)));

  /**
   * AUTO's plain-GREEDY entitlement check: a below-threshold ISO_GREEDY does not
   * imply a useful second GREEDY run — if ISO_GREEDY already used graph-native
   * candidates (provider fallback or internal graph-native compare), GREEDY would
   * just duplicate the same source truth.
   */
  static boolean autoNeedsPlainGreedy(RoundTripCandidateResult isoGreedyR,
                                      long now, long deadline) {
    return autoPlainGreedyDiscardReason(isoGreedyR, now, deadline) == null;
  }

  static String autoPlainGreedyDiscardReason(RoundTripCandidateResult isoGreedyR,
                                             long now, long deadline) {
    if (now >= deadline) {
      return "past deadline at decision point";
    }
    if (isoGreedyR == null || !isoGreedyR.accepted()) {
      return null;
    }
    if (isoGreedyR.scoreValue() >= RoundTripOrchestrator.CLEAR_ACCEPT_THRESHOLD) {
      return "ISO_GREEDY strong";
    }
    if (isoGreedyR.internalGraphNativeCompared()) {
      return "ISO_GREEDY already compared graph-native branch";
    }
    if (isoGreedyAbsorbedGraphNativeTruth(isoGreedyR)) {
      return "ISO_GREEDY absorbed graph-native truth";
    }
    return null;
  }

  /**
   * Budget (ms) for the next sequential AUTO candidate: time left to the shared
   * competition deadline, floored at {@link #MIN_CHILD_BUDGET_MS} so a spawned
   * candidate gets a usable slice rather than ~0.
   */
  static long childCandidateBudgetMs(long deadline, long now) {
    return Math.max(MIN_CHILD_BUDGET_MS, deadline - now);
  }

  /**
   * AUTO candidate competition for generated round trips (no user vias). Runs
   * greedy candidates first, the legacy probe/WAYPOINT generator only as fallback:
   * <ol>
   *   <li>ISO_GREEDY — iso-derived candidates fed to the greedy planner.</li>
   *   <li>GREEDY — plain graph-native planner, if ISO_GREEDY fails or is weak.</li>
   *   <li>WAYPOINT/probe — only if greedy produced no accepted route.</li>
   * </ol>
   *
   * <p>Each candidate runs in an isolated child {@link RoutingEngine} built from a
   * request-fields-only copy of the parent {@link RoutingContext} (no parsed/runtime
   * state shared, output suppressed). The highest-scoring accepted candidate's
   * {@link OsmTrack} is adopted; its disclosures are surfaced. If none pass strict
   * validation, the lenient default adopts the least-bad best-effort track (see
   * {@link #selectBestEffortCandidate}); strict mode leaves the track null and sets
   * an error.
   */
  private void runAutoCandidateCompetition(double searchRadius, double direction) {
    long t0 = System.currentTimeMillis();
    // One wall-clock budget shared across the sequentially-run candidates, so
    // the competition cannot run ~Nx the requested timeout. Each child gets the
    // remaining slice (see runChildCandidate); once it is exhausted we stop
    // spawning further candidates.
    long deadline = t0 + (ops.maxRunningTime() > 0 ? ops.maxRunningTime() : DEFAULT_AUTO_BUDGET_MS);
    List<RoundTripCandidateResult> results = new ArrayList<>(3);

    // 1+2. Run ISO_GREEDY first, then plain GREEDY only when the ISO result
    // proves the comparison is still useful. This is the issue-#26 default:
    // avoid duplicate production algorithm runs when ISO_GREEDY is strong or
    // has already absorbed the graph-native provider fallback. An opt-in
    // speculative mode can still start GREEDY in parallel for deployments that
    // prefer lower single-request latency over duplicate CPU work.
    RoundTripCandidateResult[] parallel = new RoundTripCandidateResult[2];
    java.util.concurrent.atomic.AtomicReference<RoutingEngine> greedyEngineOut =
      new java.util.concurrent.atomic.AtomicReference<>();
    Thread greedyThread = null;
    // Optional load-aware parallelism: routing is CPU-bound, so speculative
    // GREEDY is opt-in and also gated on a NON-BLOCKING permit. If the permit
    // is unavailable, or speculation is disabled, GREEDY runs sequentially only
    // if the ISO result needs it.
    boolean parallelPermit = SPECULATIVE_AUTO_GREEDY
      && System.currentTimeMillis() < deadline
      && PARALLEL_AUTO_SEMAPHORE.tryAcquire();
    if (parallelPermit) {
      greedyThread = new Thread(() -> {
        try {
          parallel[1] = runChildCandidate(RoundTripAlgorithm.GREEDY, searchRadius, direction,
            deadline, greedyEngineOut);
        } finally {
          PARALLEL_AUTO_SEMAPHORE.release();
        }
      }, "roundtrip-auto-greedy");
      // Daemon: a discarded speculative child must never delay JVM exit (CLI).
      greedyThread.setDaemon(true);
      greedyThread.start();
    }
    parallel[0] = runChildCandidate(RoundTripAlgorithm.ISO_GREEDY, searchRadius, direction, deadline);
    RoundTripCandidateResult isoGreedyR = parallel[0] != null
      ? parallel[0] : new RoundTripCandidateResult(RoundTripAlgorithm.ISO_GREEDY);
    // Whether GREEDY will be consulted is fully decidable BEFORE the join:
    // the spec calls for GREEDY when iso pool is not viable OR ISO_GREEDY is
    // weak (same single threshold for both signals), and the sequential
    // competition decided whether to START GREEDY right after ISO_GREEDY
    // completed — recording the entitlement instant here keeps the budget
    // accounting identical (a tiny budget still runs/counts exactly one
    // candidate). Deciding now means a STRONG ISO_GREEDY never waits out the
    // speculative child: it is terminated instead, so AUTO latency on the
    // good path stays that of ISO_GREEDY alone.
    long greedyDecisionTime = System.currentTimeMillis();
    // MAX effort (QUALITY tier): the plain-GREEDY competitor always runs — the
    // caller asked for the best loop and accepts the cost; the health-gated
    // skip is a latency optimization the tier explicitly opts out of. Still
    // bounded by the shared deadline.
    boolean greedyNeeded = orchestrator.request.effortPolicy.runGreedyAlways
      && System.currentTimeMillis() < deadline
      || autoNeedsPlainGreedy(isoGreedyR, greedyDecisionTime, deadline);
    String greedyDiscardReason = autoPlainGreedyDiscardReason(isoGreedyR, greedyDecisionTime, deadline);
    boolean greedyResultIgnored = false;
    if (greedyThread != null) {
      RoutingEngine greedyChild = greedyEngineOut.get();
      if (!greedyNeeded && greedyChild != null) {
        // The speculative child's result will not be consulted — kill it so
        // the bounded join below returns promptly (the volatile flag aborts
        // its searches/expansions within ~one heap pop).
        greedyChild.terminate();
      }
      // ALWAYS bound the join. Even a needed child must not hang the request
      // thread: its own budget ends at the shared deadline, so wait only up to
      // the remaining budget plus an unwind margin. If it overstays that
      // (overshot its budget, or wedged in a path slow to honor termination),
      // terminate it and give it a final short window — never block forever.
      // A discarded child gets only the unwind margin.
      long joinBudgetMs = greedyNeeded
        ? Math.max(0L, deadline - System.currentTimeMillis()) + AUTO_CHILD_JOIN_UNWIND_MS
        : AUTO_CHILD_JOIN_UNWIND_MS;
      try {
        greedyThread.join(joinBudgetMs);
        if (greedyThread.isAlive()) {
          ops.logInfo("AUTO: GREEDY child overstayed its budget; terminating");
          if (greedyChild != null) {
            greedyChild.terminate();
          }
          greedyThread.join(AUTO_CHILD_JOIN_UNWIND_MS);
        }
      } catch (InterruptedException ie) {
        Thread.currentThread().interrupt();
      }
      // If the (needed) child is STILL alive its result slot is not safely
      // published — treat it as no candidate rather than reading a
      // half-written result. The daemon thread cannot block JVM exit.
      if (greedyThread.isAlive()) {
        greedyNeeded = false;
        greedyResultIgnored = true;
        ops.logInfo("AUTO: GREEDY child did not stop in time; ignoring its result");
      }
    }
    results.add(isoGreedyR);
    ops.logInfo("AUTO candidate: " + isoGreedyR);

    // Sequential fallback: no spare-CPU permit was available (busy box) or the
    // budget was already spent at spawn time, so GREEDY was not started in
    // parallel. Run it now on this thread iff it is actually needed — exactly
    // the pre-parallel competition's behaviour (GREEDY only when ISO_GREEDY is
    // weak). No oversubscription: this reuses the request's own core.
    if (greedyThread == null && greedyNeeded) {
      parallel[1] = runChildCandidate(RoundTripAlgorithm.GREEDY, searchRadius, direction, deadline);
    }

    if (greedyNeeded && parallel[1] != null) {
      results.add(parallel[1]);
      ops.logInfo("AUTO candidate: " + parallel[1]);
    } else if (greedyThread != null && !greedyResultIgnored) {
      ops.logInfo("AUTO: speculative GREEDY child discarded ("
        + (greedyDiscardReason == null ? "not needed" : greedyDiscardReason)
        + ") — policy parity with the sequential competition");
    }

    // 3. Compare accepted greedy candidates; pick highest score.
    RoundTripCandidateResult winner = null;
    for (RoundTripCandidateResult r : results) {
      if (!r.accepted()) continue;
      if (winner == null || r.scoreValue() > winner.scoreValue()) {
        winner = r;
      }
    }

    // 4. Legacy fallback only if both greedy variants failed hard validation
    //    and budget remains.
    if (winner == null && System.currentTimeMillis() < deadline) {
      RoundTripCandidateResult waypointR = runChildCandidate(
        RoundTripAlgorithm.WAYPOINT, searchRadius, direction, deadline);
      results.add(waypointR);
      ops.logInfo("AUTO candidate: " + waypointR);
      if (waypointR.accepted()) {
        winner = waypointR;
      }
    }

    // 5. Last-resort ISOCHRONE fallback. The direct isochrone-frontier
    //    placement reaches loops the greedy radial candidates miss in
    //    constrained terrain (e.g. a valley where the radial probe can't
    //    form a loop in the requested direction, or only finds a chaotic
    //    one). Purely additive: only runs when ISO_GREEDY, GREEDY and
    //    WAYPOINT have all already failed, so it cannot displace a winner.
    if (winner == null && System.currentTimeMillis() < deadline) {
      RoundTripCandidateResult isochroneR = runChildCandidate(
        RoundTripAlgorithm.ISOCHRONE, searchRadius, direction, deadline);
      results.add(isochroneR);
      ops.logInfo("AUTO candidate: " + isochroneR);
      if (isochroneR.accepted()) {
        winner = isochroneR;
      }
    }
    long totalMs = System.currentTimeMillis() - t0;

    // Lenient default: if no candidate passed strict validation but one produced
    // a rideable route that failed only a QUALITY check, adopt the best-effort
    // one (the child already attached its "Warning:" advisory) instead of
    // returning nothing — keeping AUTO consistent with direct-dispatch leniency.
    // Candidates are in algorithm-quality order (ISO_GREEDY, GREEDY, WAYPOINT,
    // ISOCHRONE), so the first quality-failed track is the best best-effort.
    // The lenient/strict decision uses the same predicate as the gate path
    // (roundTripQualityHardReject), so strict mode keeps the hard "no acceptable
    // route" and only QUALITY verdicts are adopted leniently.
    if (winner == null) {
      // Among the QUALITY-tier best-effort candidates (STRUCTURAL and, under strict
      // mode, every failure are excluded by roundTripQualityHardReject), pick the
      // LEAST-BAD overall rather than the first by algorithm order. We rank with the
      // same multi-factor RouteChoiceScore used for accepted winners — distance
      // closeness (its largest weight), profile cost/m match, and reuse/shape — so
      // each candidate is penalised on the very axis it failed and the most rideable
      // degraded loop wins. No extra routing: the tracks are already generated.
      List<RoundTripCandidateResult> bestEffort = new ArrayList<>();
      for (RoundTripCandidateResult r : results) {
        if (r.track != null && r.gateVerdict != null
            && !ops.roundTripQualityHardReject(r.gateVerdict)) {
          bestEffort.add(r);
        }
      }
      winner = selectBestEffortCandidate(bestEffort, 2 * Math.PI * searchRadius,
        ops.routingContext().getProfileName(), direction);
      if (winner != null) {
        ops.logInfo("AUTO: no strictly-accepted route; adopting best-effort " + winner.algorithm
          + " (most rideable of " + bestEffort.size()
          + " degraded candidate(s)) with quality warning (lenient mode)");
      }
    }

    if (winner == null) {
      // All candidates failed. Surface the most recent (richest) error.
      String err = null;
      for (int i = results.size() - 1; i >= 0; i--) {
        if (results.get(i).errorMessage != null) { err = results.get(i).errorMessage; break; }
      }
      // Surface the best-geometry rejected candidate for post-mortem inspection,
      // mirroring the direct-dispatch reject paths. Candidates are in
      // algorithm-quality order, so the first with a track is the best
      // available rejected geometry.
      OsmTrack rejected = null;
      for (RoundTripCandidateResult r : results) {
        if (r.track != null) {
          rejected = r.track;
          break;
        }
      }
      orchestrator.rejectWithError("AUTO competition produced no acceptable route "
        + "(tried " + results.size() + " candidates in " + totalMs + "ms): "
        + (err == null ? "unknown" : err), rejected);
      return;
    }
    adoptCandidateWinner(winner, results, totalMs);
  }

  /**
   * Run one AUTO candidate in an isolated child engine, score it, return the
   * wrapper. Never throws — failures land in the result's {@code errorMessage}.
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline) {
    return runChildCandidate(algo, searchRadius, direction, deadline, null);
  }

  /**
   * Adopt the winning candidate's track as this engine's result and attach a
   * summary diagnostic of what was tried and which won.
   */
  private void adoptCandidateWinner(RoundTripCandidateResult winner,
                                    List<RoundTripCandidateResult> all, long totalMs) {
    orchestrator.setTrack(winner.track);
    orchestrator.setError(null);
    orchestrator.cleanup.finalizeAdoptedRoundTripTrack(orchestrator.request.track, orchestrator.request.track == null ? null : orchestrator.request.track.getMatchedWaypoints());
    // Best-effort (quality-failed) winner adopted under lenient mode: make sure
    // the user-facing quality Warning is present. The child engine usually
    // attaches it, but when the parent's gate re-evaluation in runChildCandidate
    // disagrees with the child's own verdict the child may not have — so attach
    // it here if absent, mirroring the direct-dispatch advisory (and skip when a
    // "Warning:" is already present to avoid a duplicate).
    if (orchestrator.request.track != null && !winner.accepted() && winner.gateVerdict != null
        && (orchestrator.request.track.message == null || !orchestrator.request.track.message.contains("Warning:"))) {
      orchestrator.appendRouteMessage(orchestrator.request.track, "Warning: " + winner.gateVerdict.getRejectionReason()
        + " (shape=" + winner.gateVerdict.getShape() + ") — route returned anyway; ride at your"
        + " discretion, or set roundTripStrictQuality=1 to reject it.");
    }
    // Append a summary message so debugging consumers can see the
    // competition outcome. Score breakdown is in the route-choice verdict.
    StringBuilder summary = new StringBuilder(256);
    summary.append("AUTO selected ").append(winner.algorithm)
      .append(" (score ").append(String.format(Locale.US, "%.3f", winner.scoreValue()))
      .append(") after ").append(all.size()).append(" candidate(s) in ").append(totalMs).append("ms.");
    for (RoundTripCandidateResult r : all) {
      if (r == winner) continue;
      summary.append(" Also tried ").append(r.algorithm).append(": ")
        .append(r.accepted() ? String.format(Locale.US, "score %.3f", r.scoreValue())
                             : (r.errorMessage == null ? "no track" : "rejected"))
        .append('.');
    }
    if (orchestrator.request.track != null) {
      // orchestrator.request.track is nullable here (a best-effort winner can carry no track —
      // see the null-guards above at adoption and the warning block); only
      // attach the AUTO summary when there is a track to annotate.
      if (orchestrator.request.track.message == null || orchestrator.request.track.message.isEmpty()) {
        orchestrator.request.track.message = summary.toString();
      } else {
        orchestrator.request.track.message += " " + summary.toString();
      }
    }
    // Keep messageList.get(0) in sync with the just-extended message so the
    // GPX <brouter:info> / comment block reflects the AUTO summary too.
    orchestrator.cleanup.ensureInfoMessage(orchestrator.request.track);
    ops.logInfo(summary.toString());
    if (winner.score != null) {
      ops.logInfo("AUTO winner score breakdown:\n" + winner.score.describe());
    }
    // Format + persist the adopted track if the caller asked for an
    // output file. The child engines ran with null outfileBase (output
    // suppressed); the parent does the single final write.
    ops.writeAdoptedTrackOutput(orchestrator.request.track);
  }

  /**
   * Rank degraded best-effort candidates, return the most rideable (or {@code null}
   * if none have a track). Uses {@link RouteChoiceScore#scoreBestEffort}, which
   * bypasses the scorer's accepted-only zero-guard (a rejected track is ranked on
   * real geometry, not collapsed to 0) but still applies the gate verdict's shape
   * penalty, so a rejected LOLLIPOP/OUT_AND_BACK cannot outrank a strict loop.
   * Ties keep {@code candidates} order (AUTO algorithm-quality order). Does no routing.
   */
  static RoundTripCandidateResult selectBestEffortCandidate(
      List<RoundTripCandidateResult> candidates, double expectedDistance,
      String profileName, double direction) {
    RoundTripCandidateResult best = null;
    double bestScore = -1.0;
    RouteChoiceScore.Verdict bestVerdict = null;
    for (RoundTripCandidateResult r : candidates) {
      if (r.track == null) {
        continue;
      }
      RouteChoiceScore.Verdict v = RouteChoiceScore.scoreBestEffort(
        r.track, expectedDistance, profileName, r.gateVerdict, direction);
      double s = v.score();
      if (s > bestScore) {
        bestScore = s;
        best = r;
        bestVerdict = v;
      }
    }
    // Surface the computed best-effort score on the winner so the adoption
    // summary logs the real value (and the score breakdown) instead of 0.000;
    // r.score is otherwise only set for strictly-accepted candidates.
    if (best != null && best.score == null) {
      best.score = bestVerdict;
    }
    return best;
  }

  /**
   * As above, but publishes the child engine into {@code engineOut} as soon as it
   * is constructed, so a concurrent coordinator can {@code terminate()} a
   * speculative child no longer needed (the kill flag is honoured per search pop
   * and per expansion pop).
   */
  private RoundTripCandidateResult runChildCandidate(RoundTripAlgorithm algo,
                                                     double searchRadius, double direction,
                                                     long deadline,
                                                     java.util.concurrent.atomic.AtomicReference<RoutingEngine> engineOut) {
    long t0 = System.currentTimeMillis();
    RoundTripCandidateResult r = new RoundTripCandidateResult(algo);
    try {
      RoutingContext childCtx = ops.routingContext().copyRequestFields();
      childCtx.roundTripAlgorithm = algo;
      childCtx.startDirection = (int) direction;
      // Inherit the user's direction intent from copyRequestFields rather than
      // hard-forcing it. forceUseStartDirection makes the first leg leave on a
      // strict bearing; when the user supplied only a soft `direction` (or
      // none) that over-constrains the loop and can shove the opening leg onto
      // a profile-hostile stretch, failing a candidate that the same algorithm
      // accepts when free to pick a nearby bearing. Only an explicit `heading`
      // (which sets forceUseStartDirection on the parent) hard-forces here.
      // Copy waypoint list — child engine mutates its own list.
      List<OsmNodeNamed> childWps = new ArrayList<>(ops.waypoints().size());
      for (OsmNodeNamed wp : ops.waypoints()) {
        OsmNodeNamed copy = new OsmNodeNamed(new OsmNode(wp.ilon, wp.ilat));
        copy.name = wp.name;
        childWps.add(copy);
      }
      // Output suppressed (null outfileBase). Child runs its own pipeline
      // including post-routing checks + quality gate; we just inspect the
      // result.
      RoutingEngine child = new RoutingEngine(null, null, ops.segmentDir(), childWps, childCtx,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      child.quite = true;
      // The child plans with the parent's resolved effort (QUALITY's raised
      // top-K / plan budget must reach the planners it spawns).
      child.roundTripOps().setRoundTripEffortPolicy(orchestrator.request.effortPolicy);
      if (engineOut != null) {
        engineOut.set(child);
      }
      // Give the child only the remaining shared budget (floored so a spawned
      // candidate still gets a usable slice), not the full request timeout.
      long budget = childCandidateBudgetMs(deadline, System.currentTimeMillis());
      child.doRun(budget);
      r.track = child.getFoundTrack();
      r.errorMessage = child.getErrorMessage();
      r.runtimeMillis = System.currentTimeMillis() - t0;
      // Aggregate the child's expansion work into the parent so
      // getLinksProcessed() reports request-level totals (the perf budget
      // suite's work metric). Same-thread for sequential children; the
      // speculative parallel child is joined before its result is read.
      ops.addLinksProcessed(child.getLinksProcessed());
      // All winner-attribution telemetry (incl. the keep-when-forced marker
      // the re-gate below honors) reads through this reference — no
      // field-by-field copy to forget when RoundTripResult grows.
      r.planner = child.getLastRoundTripResult();

      if (r.track != null) {
        // Score against the parent's expected loop distance. This produces
        // a verdict that may differ from the child's internal gate result
        // because the parent's ops.routingContext() is the source of truth (e.g.
        // for profile-name lookup), but in practice both agree.
        double expectedDist = 2 * Math.PI * searchRadius;
        String profileName = ops.routingContext().getProfileName();
        r.gateVerdict = orchestrator.evaluateRoundTripGate(r.track, searchRadius, false,
          r.forcedCorridorAccepted());
        if (r.gateVerdict.isAccepted()) {
          r.score = RouteChoiceScore.score(r.track, expectedDist,
            profileName, r.gateVerdict, direction);
        }
      }
    } catch (RuntimeException e) {
      // Preserve the exception type: e.getMessage() is null for NPE/AIOOBE/CCE,
      // which otherwise surfaces an undiagnosable "threw: null" to the operator.
      // Also log the full stack trace on the parent (which, unlike the child, is
      // not `quite`) so a recurring child failure is diagnosable from logs — the
      // child suppressed its own logging via quite=true + null outfileBase.
      ops.logThrowable(e);
      r.errorMessage = "candidate " + algo + " threw: " + e.getClass().getSimpleName()
        + (e.getMessage() == null ? "" : ": " + e.getMessage());
      r.runtimeMillis = System.currentTimeMillis() - t0;
    }
    return r;
  }

  private static final long MIN_CHILD_BUDGET_MS = 5_000;

  private static boolean isoGreedyAbsorbedGraphNativeTruth(RoundTripCandidateResult isoGreedyR) {
    // The child's explicit start-policy decision: a graph-native-only plan
    // already used the same candidate source as plain GREEDY, so a separate
    // GREEDY child would duplicate it. (This used to be inferred from three
    // telemetry sentinels — no iso legs + some non-iso legs + NaN health —
    // which any telemetry-semantics change could silently flip.)
    return isoGreedyR.algorithm == RoundTripAlgorithm.ISO_GREEDY
      && isoGreedyR.graphNativeOnlyStart();
  }
}
