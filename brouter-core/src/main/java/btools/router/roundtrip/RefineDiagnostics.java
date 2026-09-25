package btools.router.roundtrip;

/**
 * Diagnostics and telemetry for the round-trip refinement stage (§7).
 */
public class RefineDiagnostics {
  /** Whether a refined candidate was accepted and published. */
  public boolean refineApplied;

  /** Skip reason or predicate rejection reason or "accepted". */
  public String refineReason;

  /** Baseline oracle cost per meter (-1 if unpriced). */
  public double oracleCostPerMeterBefore = -1.0;

  /** Final replacement oracle cost per meter (-1 if not refined). */
  public double oracleCostPerMeterAfter = -1.0;

  /** Baseline RoadCharacterScore (-1 if uncalculated). */
  public double rcsBefore = -1.0;

  /** Final replacement RoadCharacterScore (-1 if not refined). */
  public double rcsAfter = -1.0;

  /** Total mutation proposals generated. */
  public int proposals;

  /** Proposals rejected by skeleton constraints before routing. */
  public int invalidProposals;

  /** Proposals successfully routed and evaluated. */
  public int evaluations;

  /** Number of raw legs routed through Dijkstra. */
  public int legsRouted;

  /** Number of legs served from leg cache. */
  public int cacheHits;

  /** Number of candidate loops put through full finalization. */
  public int finalizations;

  /** Number of chains configured. */
  public int chains = 1;

  /** True if any expensive operation timed out against request or stage deadline. */
  public boolean refineTruncated;

  /** Name of the operation that timed out, if truncated. */
  public String timeoutOperation;

  /** Total wall-clock time spent in the refinement stage in milliseconds. */
  public long elapsedMs;

  /** Single evaluation trace event for CSV export (§6 M1). */
  public static final class EvaluationTrace {
    public final int evaluation;
    public final String operator;
    public final double energy;
    public final boolean accepted;
    public final boolean feasible;

    public EvaluationTrace(int evaluation, String operator, double energy, boolean accepted, boolean feasible) {
      this.evaluation = evaluation;
      this.operator = operator;
      this.energy = energy;
      this.accepted = accepted;
      this.feasible = feasible;
    }
  }

  public final java.util.List<EvaluationTrace> traces = new java.util.ArrayList<>();

  public void addTrace(int evaluation, String operator, double energy, boolean accepted, boolean feasible) {
    traces.add(new EvaluationTrace(evaluation, operator, energy, accepted, feasible));
  }

  /** Return cache hit rate as a fraction in [0, 1]. */
  public double getCacheHitRate() {
    int total = legsRouted + cacheHits;
    return total > 0 ? (double) cacheHits / total : 0.0;
  }
}
