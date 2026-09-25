package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;

/**
 * Proposal evaluation and local search over raw skeleton legs (§5).
 * Operates purely on raw legs and evaluation budgets (no wall-clock steering).
 */
public final class RefineSearch {

  /**
   * Evaluated candidate state in the search space.
   */
  public static final class SearchCandidate {
    private final RefineSkeleton skeleton;
    private final List<OsmTrack> rawLegs;
    private final double energy;

    public SearchCandidate(RefineSkeleton skeleton, List<OsmTrack> rawLegs, double energy) {
      this.skeleton = skeleton;
      this.rawLegs = rawLegs;
      this.energy = energy;
    }

    public RefineSkeleton getSkeleton() {
      return skeleton;
    }

    public List<OsmTrack> getRawLegs() {
      return rawLegs;
    }

    public double getEnergy() {
      return energy;
    }
  }

  /**
   * Result of the search phase, containing top-k raw finalists and updated diagnostics.
   */
  public static final class SearchResult {
    private final List<SearchCandidate> finalists;
    private final RefineDiagnostics diagnostics;

    public SearchResult(List<SearchCandidate> finalists, RefineDiagnostics diagnostics) {
      this.finalists = finalists != null ? finalists : Collections.<SearchCandidate>emptyList();
      this.diagnostics = diagnostics;
    }

    public List<SearchCandidate> getFinalists() {
      return finalists;
    }

    public RefineDiagnostics getDiagnostics() {
      return diagnostics;
    }
  }

  private final LegRouter router;
  private final LegEvaluator evaluator;
  private final LegCache legCache;
  private final RefineConfig config;
  private final RefineSkeleton originalSkeleton;
  private final List<OsmTrack> baselineRawLegs;
  private final double baselineEnergy;
  private final MoveProposalOperator moveOperator;
  private final double searchRadius;
  private final double requestedDistance;
  private final int varietySeed;
  private final long deadlineMs;

  public RefineSearch(LegRouter router,
                      LegEvaluator evaluator,
                      LegCache legCache,
                      RefineConfig config,
                      RefineSkeleton originalSkeleton,
                      List<OsmTrack> baselineRawLegs,
                      double baselineEnergy,
                      MoveProposalOperator moveOperator,
                      double searchRadius,
                      double requestedDistance,
                      int varietySeed,
                      long deadlineMs) {
    this.router = router;
    this.evaluator = evaluator;
    this.legCache = legCache != null ? legCache : new LegCache();
    this.config = config != null ? config : new RefineConfig();
    this.originalSkeleton = originalSkeleton;
    this.baselineRawLegs = baselineRawLegs != null ? baselineRawLegs : Collections.<OsmTrack>emptyList();
    this.baselineEnergy = baselineEnergy;
    this.moveOperator = moveOperator;
    this.searchRadius = searchRadius;
    this.requestedDistance = requestedDistance;
    this.varietySeed = varietySeed;
    this.deadlineMs = deadlineMs;
  }

  public RefineSearch(LegRouter router,
                      LegEvaluator evaluator,
                      RefineConfig config,
                      RefineSkeleton originalSkeleton,
                      RefineInitResult baselineInit,
                      MoveProposalOperator moveOperator,
                      double searchRadius,
                      double requestedDistance,
                      int varietySeed,
                      long deadlineMs) {
    this(router, evaluator,
         baselineInit != null ? baselineInit.getLegCache() : new LegCache(),
         config, originalSkeleton,
         baselineInit != null ? baselineInit.getRawLegs() : Collections.<OsmTrack>emptyList(),
         baselineInit != null ? baselineInit.getBaselineOracleCostPerMeter() : -1.0,
         moveOperator, searchRadius, requestedDistance, varietySeed, deadlineMs);
  }

  /**
   * Run the search over proposals up to the evaluation budget (§5).
   */
  public SearchResult search(RefineDiagnostics diag) {
    if (diag == null) {
      diag = new RefineDiagnostics();
    }
    if (originalSkeleton == null || baselineRawLegs == null || baselineRawLegs.isEmpty()) {
      return new SearchResult(Collections.<SearchCandidate>emptyList(), diag);
    }

    List<SearchCandidate> allFeasibleCandidates = new ArrayList<>();
    int maxChains = Math.max(1, config.chains);
    int evalBudgetPerChain = Math.max(1, config.evaluations / maxChains);
    int maxProposalsPerChain = Math.max(1, config.getMaxProposals() / maxChains);

    for (int chainIdx = 0; chainIdx < maxChains; chainIdx++) {
      if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
        diag.refineTruncated = true;
        diag.timeoutOperation = "search_evaluation";
        break;
      }

      // Android API 23 compliant pseudo-random generator
      long chainSeed = (varietySeed * 0x9e3779b97f4a7c15L) ^ (chainIdx * 0xbf58476d1ce4e5b9L);
      SplitmixRandom rng = new SplitmixRandom(chainSeed);

      RefineSkeleton currentSkeleton = originalSkeleton;
      List<OsmTrack> currentRawLegs = baselineRawLegs;
      double currentEnergy = baselineEnergy > 0 ? baselineEnergy
          : LoopCostOracle.price(router, currentRawLegs, currentSkeleton.getWaypoints());

      int chainEvaluations = 0;
      int chainProposals = 0;

      while (chainEvaluations < evalBudgetPerChain && chainProposals < maxProposalsPerChain
             && diag.evaluations < config.evaluations && diag.proposals < config.getMaxProposals()) {
        if (deadlineMs > 0 && System.currentTimeMillis() >= deadlineMs) {
          diag.refineTruncated = true;
          diag.timeoutOperation = "search_evaluation";
          break;
        }

        chainProposals++;
        diag.proposals++;

        MoveProposalOperator.MoveProposal prop = moveOperator.propose(
          currentSkeleton, originalSkeleton, rng, searchRadius, requestedDistance);

        if (!prop.isFeasible()) {
          diag.invalidProposals++;
          continue;
        }

        // Proposal passed pre-routing validation -> counts as evaluation
        chainEvaluations++;
        diag.evaluations++;

        RefineSkeleton mutatedSkeleton = prop.getMutatedSkeleton();
        List<MatchedWaypoint> waypoints = mutatedSkeleton.getWaypoints();
        int legCount = waypoints.size() - 1;

        // Assemble candidate raw legs using legCache
        List<OsmTrack> candidateRawLegs = new ArrayList<>(legCount);
        boolean routeFailed = false;

        for (int l = 0; l < legCount; l++) {
          MatchedWaypoint from = waypoints.get(l);
          MatchedWaypoint to = waypoints.get(l + 1);

          OsmTrack leg = legCache.get(from, to);
          if (leg != null) {
            diag.cacheHits++;
          } else {
            long remaining = deadlineMs > 0 ? (deadlineMs - System.currentTimeMillis()) : 5000L;
            if (deadlineMs > 0 && remaining <= 0) {
              diag.refineTruncated = true;
              diag.timeoutOperation = "search_evaluation";
              routeFailed = true;
              break;
            }
            diag.legsRouted++;
            leg = evaluator.route(from, to, remaining);
            if (leg != null) {
              legCache.put(from, to, leg);
            }
          }

          if (leg == null || leg.nodes == null || leg.nodes.size() < 2) {
            routeFailed = true;
            break;
          }
          candidateRawLegs.add(leg);
        }

        if (diag.refineTruncated) {
          break;
        }

        if (routeFailed || candidateRawLegs.size() != legCount) {
          diag.addTrace(diag.evaluations, prop.getOperator(), -1.0, false, false);
          continue;
        }

        // Validate seams between adjacent legs
        boolean seamsOk = true;
        for (int l = 0; l < candidateRawLegs.size() - 1; l++) {
          OsmTrack legA = candidateRawLegs.get(l);
          OsmTrack legB = candidateRawLegs.get(l + 1);
          MatchedWaypoint seamWp = (l + 1 < waypoints.size()) ? waypoints.get(l + 1) : null;
          if (!isSeamValid(legA, legB, seamWp)) {
            legCache.incrementSeamMismatches();
            seamsOk = false;
            break;
          }
        }
        if (!seamsOk) {
          diag.addTrace(diag.evaluations, prop.getOperator(), -1.0, false, false);
          continue;
        }

        // Price continuous loop energy
        double energy = LoopCostOracle.price(router, candidateRawLegs, waypoints);
        if (energy <= 0 || Double.isNaN(energy)) {
          diag.addTrace(diag.evaluations, prop.getOperator(), -1.0, false, false);
          continue;
        }

        SearchCandidate cand = new SearchCandidate(mutatedSkeleton, candidateRawLegs, energy);
        allFeasibleCandidates.add(cand);

        boolean accepted = false;
        // State update based on search strategy
        if (config.mode == RefineConfig.Mode.LOCAL) {
          if (energy < currentEnergy) {
            accepted = true;
            currentSkeleton = mutatedSkeleton;
            currentRawLegs = candidateRawLegs;
            currentEnergy = energy;
          }
        } else if (config.mode == RefineConfig.Mode.ANNEAL) {
          double delta = energy - currentEnergy;
          double progress = (double) diag.evaluations / Math.max(1, config.evaluations);
          double temp = Math.max(config.minTemperature, config.initialTemperature * (1.0 - progress));
          if (delta < 0 || rng.nextDouble() < Math.exp(-delta / temp)) {
            accepted = true;
            currentSkeleton = mutatedSkeleton;
            currentRawLegs = candidateRawLegs;
            currentEnergy = energy;
          }
        } else {
          // BEST_OF_N
          accepted = energy < currentEnergy;
        }
        diag.addTrace(diag.evaluations, prop.getOperator(), energy, accepted, true);
      }
    }

    if (allFeasibleCandidates.isEmpty()) {
      return new SearchResult(Collections.<SearchCandidate>emptyList(), diag);
    }

    // Sort feasible candidates by raw energy ascending (lowest cost first)
    Collections.sort(allFeasibleCandidates, new Comparator<>() {
      @Override
      public int compare(SearchCandidate c1, SearchCandidate c2) {
        return Double.compare(c1.getEnergy(), c2.getEnergy());
      }
    });

    List<SearchCandidate> finalists = new ArrayList<>(config.topKFinalists);
    Set<String> seenSignatures = new HashSet<>();
    for (SearchCandidate cand : allFeasibleCandidates) {
      if (cand.getEnergy() >= baselineEnergy) {
        continue;
      }
      String sig = candidateSignature(cand);
      if (seenSignatures.add(sig)) {
        finalists.add(cand);
        if (finalists.size() >= config.topKFinalists) {
          break;
        }
      }
    }

    return new SearchResult(finalists, diag);
  }

  private static boolean isSeamValid(OsmTrack legA, OsmTrack legB, MatchedWaypoint seamWp) {
    if (legA == null || legB == null || legA.nodes == null || legB.nodes == null
        || legA.nodes.isEmpty() || legB.nodes.isEmpty()) {
      return false;
    }
    OsmPathElement end = legA.nodes.get(legA.nodes.size() - 1);
    OsmPathElement start = legB.nodes.get(0);
    if (end.getILon() == start.getILon() && end.getILat() == start.getILat()) {
      return true;
    }
    if (seamWp == null) {
      return false;
    }
    return isWaypointNode(end, seamWp) && isWaypointNode(start, seamWp);
  }

  private static boolean isWaypointNode(OsmPathElement elem, MatchedWaypoint wp) {
    if (wp.crosspoint != null && elem.getILon() == wp.crosspoint.ilon && elem.getILat() == wp.crosspoint.ilat) {
      return true;
    }
    if (wp.node1 != null && elem.getILon() == wp.node1.ilon && elem.getILat() == wp.node1.ilat) {
      return true;
    }
    if (wp.node2 != null && elem.getILon() == wp.node2.ilon && elem.getILat() == wp.node2.ilat) {
      return true;
    }
    return false;
  }

  private static String candidateSignature(SearchCandidate cand) {
    StringBuilder sb = new StringBuilder();
    for (MatchedWaypoint mwp : cand.getSkeleton().getVias()) {
      if (mwp.crosspoint != null) {
        sb.append(mwp.crosspoint.ilon).append(',').append(mwp.crosspoint.ilat).append(';');
      }
    }
    return sb.toString();
  }
}
