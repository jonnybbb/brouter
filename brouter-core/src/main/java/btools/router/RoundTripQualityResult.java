package btools.router;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Structured result from {@link RoundTripQualityGate#evaluate}.
 *
 * <p>Carries both the accept/reject verdict and a semantic description of
 * the route's shape, so callers can disclose to the cyclist what they're
 * actually getting (clean loop, lollipop with a scenic spur, out-and-back
 * to a cape) instead of receiving a binary "accepted/rejected" with no
 * context.
 *
 * <p>Field invariants:
 * <ul>
 *   <li>{@link #shape} is always set, even for rejected routes — it captures
 *       what the route IS, not what was accepted.</li>
 *   <li>{@link #rejectionReason} is non-null iff {@link #accepted} is false.</li>
 *   <li>{@link #totalReuseRatio} is in [0, 1].</li>
 *   <li>{@link #maxContiguousReuseMeters} is the longest single contiguous
 *       retraced stretch — the most cyclist-visible reuse, regardless of
 *       where it sits in the route.</li>
 *   <li>{@link #terminalStemReuseMeters} is the length of reuse that the
 *       classifier accepts as a short unavoidable stem near start/end
 *       (typically a hotel-to-village access road).</li>
 *   <li>{@link #scenicSpurReuseMeters} is the length of reuse that the
 *       classifier accepts as a terminal spur (the road to the
 *       cape/pass/dead-end). For STRICT_LOOP this is 0.</li>
 *   <li>{@link #disclosures} is a human-readable list of facts the cyclist
 *       should know about the route (e.g. "contains retraced scenic spur:
 *       4.2km"). Always non-null, possibly empty.</li>
 * </ul>
 *
 * <p>The class is intentionally immutable after construction: callers
 * receive an unambiguous snapshot, no risk of downstream code mutating
 * the verdict.
 */
public final class RoundTripQualityResult {

  private final boolean accepted;
  private final RouteShape shape;
  private final String rejectionReason;
  private final double totalReuseRatio;
  private final int maxContiguousReuseMeters;
  private final int terminalStemReuseMeters;
  private final int scenicSpurReuseMeters;
  private final List<String> disclosures;

  private RoundTripQualityResult(Builder b) {
    this.accepted = b.accepted;
    this.shape = b.shape;
    this.rejectionReason = b.rejectionReason;
    this.totalReuseRatio = b.totalReuseRatio;
    this.maxContiguousReuseMeters = b.maxContiguousReuseMeters;
    this.terminalStemReuseMeters = b.terminalStemReuseMeters;
    this.scenicSpurReuseMeters = b.scenicSpurReuseMeters;
    this.disclosures = (b.disclosures == null || b.disclosures.isEmpty())
      ? Collections.emptyList()
      : Collections.unmodifiableList(new ArrayList<>(b.disclosures));
  }

  public boolean isAccepted() { return accepted; }
  public RouteShape getShape() { return shape; }
  public String getRejectionReason() { return rejectionReason; }
  public double getTotalReuseRatio() { return totalReuseRatio; }
  public int getMaxContiguousReuseMeters() { return maxContiguousReuseMeters; }
  public int getTerminalStemReuseMeters() { return terminalStemReuseMeters; }
  public int getScenicSpurReuseMeters() { return scenicSpurReuseMeters; }
  public List<String> getDisclosures() { return disclosures; }

  @Override
  public String toString() {
    return "RoundTripQualityResult["
      + "accepted=" + accepted
      + ", shape=" + shape
      + (rejectionReason != null ? ", reason=" + rejectionReason : "")
      + ", reuseRatio=" + String.format(java.util.Locale.US, "%.2f", totalReuseRatio)
      + ", maxContiguous=" + maxContiguousReuseMeters + "m"
      + ", stem=" + terminalStemReuseMeters + "m"
      + ", spur=" + scenicSpurReuseMeters + "m"
      + (disclosures.isEmpty() ? "" : ", disclosures=" + disclosures)
      + "]";
  }

  public static Builder builder() { return new Builder(); }

  public static final class Builder {
    private boolean accepted;
    private RouteShape shape = RouteShape.STRICT_LOOP;
    private String rejectionReason;
    private double totalReuseRatio;
    private int maxContiguousReuseMeters;
    private int terminalStemReuseMeters;
    private int scenicSpurReuseMeters;
    private List<String> disclosures;

    public Builder accepted(boolean v) { this.accepted = v; return this; }
    public Builder shape(RouteShape v) { this.shape = v; return this; }
    public Builder rejectionReason(String v) { this.rejectionReason = v; return this; }
    public Builder totalReuseRatio(double v) { this.totalReuseRatio = v; return this; }
    public Builder maxContiguousReuseMeters(int v) { this.maxContiguousReuseMeters = v; return this; }
    public Builder terminalStemReuseMeters(int v) { this.terminalStemReuseMeters = v; return this; }
    public Builder scenicSpurReuseMeters(int v) { this.scenicSpurReuseMeters = v; return this; }
    public Builder addDisclosure(String d) {
      if (d == null) return this;
      if (disclosures == null) disclosures = new ArrayList<>();
      disclosures.add(d);
      return this;
    }
    public RoundTripQualityResult build() {
      // Defensive: rejected results must carry a reason; accepted results must not.
      if (!accepted && (rejectionReason == null || rejectionReason.isEmpty())) {
        throw new IllegalStateException("rejected result must have a rejectionReason");
      }
      if (accepted && rejectionReason != null) {
        throw new IllegalStateException("accepted result must not have a rejectionReason");
      }
      return new RoundTripQualityResult(this);
    }
  }
}
