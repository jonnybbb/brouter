package btools.router;

import java.util.Collections;
import java.util.List;

/**
 * Result of the optimized FAST waypoint placement: a complete, committed-once
 * waypoint skeleton plus which placement path produced it. The caller replaces
 * its waypoint list with {@link #skeleton} in one step (build-then-commit), so
 * a failed or degraded placement can never leave partial waypoints behind.
 */
final class FastPlacementOutcome {

  /**
   * Which placement path produced the skeleton. The first three map to the
   * {@code ENVELOPE_FAST} telemetry counter, {@code CIRCLE_FALLBACK} to
   * {@code CIRCLE} — the 2-value counter is a stable corpus contract.
   */
  enum Path { RING, DIRECTIONAL_LOBE, RING_FALLBACK, CIRCLE_FALLBACK }

  /**
   * Full skeleton: start, intermediate vias, closing start. Fully validated on
   * every path — the optimized paths commit only road-snapped, deduped,
   * reachability-checked vias, and the circle fallback runs the shared
   * matching pass behind {@link FastPlacementOps#circleFallbackValidated}. The
   * caller routes it as-is (plus the start snap it owns).
   */
  final List<OsmNodeNamed> skeleton;

  final Path path;

  /** Reason for a degraded path (e.g. the circle fallback), or {@code null}. */
  final String diagnostic;

  FastPlacementOutcome(List<OsmNodeNamed> skeleton, Path path, String diagnostic) {
    this.skeleton = Collections.unmodifiableList(skeleton);
    this.path = path;
    this.diagnostic = diagnostic;
  }

  /** True for the optimized (probe-snapped) paths — the {@code ENVELOPE_FAST} counter side. */
  boolean optimizedPlacement() {
    return path != Path.CIRCLE_FALLBACK;
  }
}
