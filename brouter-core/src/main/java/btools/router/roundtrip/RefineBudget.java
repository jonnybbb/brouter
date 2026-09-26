package btools.router.roundtrip;

/** Scoped, cooperative deadline shared by refinement and the engine operations it calls. */
public final class RefineBudget implements AutoCloseable {
  private final LegRouter router;
  private final long previousDeadline;
  private final long deadline;

  public RefineBudget(LegRouter router, long deadline) {
    this.router = router;
    previousDeadline = router.refinementDeadline();
    this.deadline = earlier(previousDeadline, deadline);
    router.setRefinementDeadline(this.deadline);
  }

  public static long earlier(long first, long second) {
    return first <= 0 ? second : second <= 0 ? first : Math.min(first, second);
  }

  public void check() {
    check(router, deadline);
  }

  public static void check(LegRouter router, long deadline) {
    if (router.isTerminated()) {
      throw new Exceeded(FinalizationOutcome.CANCELLED);
    }
    if (deadline > 0 && System.currentTimeMillis() >= deadline) {
      throw new Exceeded(FinalizationOutcome.TIMEOUT);
    }
  }

  @Override
  public void close() {
    router.setRefinementDeadline(previousDeadline);
  }

  /** Distinguishes budget exhaustion from unroutable or unpriceable geometry. */
  public static final class Exceeded extends IllegalArgumentException {
    private static final long serialVersionUID = 1L;
    public final FinalizationOutcome outcome;

    public Exceeded(FinalizationOutcome outcome) {
      super(outcome == FinalizationOutcome.CANCELLED
        ? "operation killed by thread-priority-watchdog" : "refinement timeout");
      this.outcome = outcome;
    }
  }
}
