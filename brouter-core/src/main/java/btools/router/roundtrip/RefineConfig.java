package btools.router.roundtrip;

/**
 * Configuration for post-tier round-trip refinement stage (§9).
 */
public class RefineConfig {
  /** Maximum number of candidate evaluations to perform. */
  public int evaluations = 4;

  /** Maximum candidate proposals to generate (default 4 × evaluations). */
  public int maxProposals = -1;

  /** Return the effective maximum proposal limit (4 × evaluations if unset). */
  public int getMaxProposals() {
    return maxProposals > 0 ? maxProposals : (4 * evaluations);
  }

  /** Number of independent search chains (default 1). */
  public int chains = 1;

  /** Stage wall-clock safety cap in milliseconds (default 3000ms). */
  public int maxMs = 3000;

  /** Number of top raw candidates to put through full finalization. */
  public int topKFinalists = 3;

  /** Relative improvement margin required in oracle cost/m (0.5%). */
  public double costMargin = 0.005;

  /** RoadCharacterScore guard band (replacement must be >= baseline - 0.02). */
  public double rcsGuard = 0.02;

  /** Minimum spacing between adjacent vias after snapping (meters). */
  public int minViaSpacingMeters = 300;

  /** Maximum displacement from original via position as fraction of searchRadius. */
  public double maxDisplacementFraction = 0.25;

  /** Maximum allowed deviation in bearing to farthest point (degrees). */
  public double maxDirectionDeltaDegrees = 15.0;

  /** Distance radius under which farthest point direction check is waived (meters). */
  public double degenerateDirectionRadius = 500.0;

  /** Search mode / strategy (§5). */
  public enum Mode {
    NONE,
    BEST_OF_N,
    LOCAL,
    ANNEAL
  }

  /** Search mode (default NONE). */
  public Mode mode = Mode.NONE;

  /** Maximum radius bound factor from start (§4.4, §9). */
  public double radiusBoundFactor = 0.6;

  /** Simulated annealing initial temperature T0. */
  public double initialTemperature = 0.05;

  /** Simulated annealing temperature floor. */
  public double minTemperature = 0.001;

  public static Mode parseMode(String s) {
    if (s == null) return Mode.NONE;
    String clean = s.trim().toLowerCase();
    if ("best_of_n".equals(clean) || "bestofn".equals(clean) || "bon".equals(clean)) {
      return Mode.BEST_OF_N;
    }
    if ("local".equals(clean)) {
      return Mode.LOCAL;
    }
    if ("anneal".equals(clean)) {
      return Mode.ANNEAL;
    }
    return Mode.NONE;
  }

  /**
   * Create a RefineConfig from RoutingContext request parameters and system properties.
   */
  public static RefineConfig fromContext(btools.router.RoutingContext rc) {
    RefineConfig cfg = new RefineConfig();
    String modeStr = null;
    if (rc != null && rc.roundTripRefine != null) {
      modeStr = rc.roundTripRefine;
    }
    if (modeStr == null || modeStr.isEmpty()) {
      modeStr = System.getProperty("loop.refine.mode", System.getProperty("loop.refine", "none"));
    }
    cfg.mode = parseMode(modeStr);

    if (rc != null && rc.roundTripRefineEvals != null) {
      cfg.evaluations = Math.max(1, Math.min(64, rc.roundTripRefineEvals));
    } else {
      String evalsStr = System.getProperty("loop.refine.evaluations");
      if (evalsStr != null && !evalsStr.isEmpty()) {
        try {
          cfg.evaluations = Math.max(1, Math.min(64, Integer.parseInt(evalsStr.trim())));
        } catch (NumberFormatException ignore) {
        }
      }
    }

    if (rc != null && rc.roundTripRefineMaxMs != null) {
      cfg.maxMs = (int) Math.max(500L, Math.min(8000L, rc.roundTripRefineMaxMs));
    } else {
      String maxMsStr = System.getProperty("loop.refine.maxMs");
      if (maxMsStr != null && !maxMsStr.isEmpty()) {
        try {
          cfg.maxMs = Math.max(500, Math.min(8000, Integer.parseInt(maxMsStr.trim())));
        } catch (NumberFormatException ignore) {
        }
      }
    }
    return cfg;
  }
}
