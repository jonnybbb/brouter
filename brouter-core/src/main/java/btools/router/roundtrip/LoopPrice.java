package btools.router.roundtrip;

/** A continuous walk's cost and distance, or a distinguishable unsuccessful outcome. */
public final class LoopPrice {
  public final FinalizationOutcome outcome;
  public final int cost;
  public final int distance;
  public final long geometrySignature;

  private LoopPrice(FinalizationOutcome outcome, int cost, int distance, long geometrySignature) {
    this.outcome = outcome;
    this.cost = cost;
    this.distance = distance;
    this.geometrySignature = geometrySignature;
  }

  public static LoopPrice success(int cost, int distance, long geometrySignature) {
    return cost >= 0 && distance > 0
      ? new LoopPrice(FinalizationOutcome.SUCCESS, cost, distance, geometrySignature)
      : failure(FinalizationOutcome.FAILURE);
  }

  public static LoopPrice failure(FinalizationOutcome outcome) {
    return new LoopPrice(outcome, -1, 0, 0);
  }

  public double costPerMeter() {
    if (outcome == FinalizationOutcome.TIMEOUT || outcome == FinalizationOutcome.CANCELLED) {
      throw new RefineBudget.Exceeded(outcome);
    }
    return outcome == FinalizationOutcome.SUCCESS ? (double) cost / distance : -1.0;
  }

  public String method() {
    return outcome == FinalizationOutcome.SUCCESS ? "continuous" : "none";
  }
}
