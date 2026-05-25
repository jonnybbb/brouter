package btools.router;

import java.util.List;
import java.util.Locale;

import btools.mapaccess.MatchedWaypoint;

/**
 * Production-safety acceptance gate for round-trip routes.
 *
 * <p>A generated round-trip is unsafe to ship to a cyclist if it contains
 * any of:
 * <ul>
 *   <li>a synthetic beeline segment (the engine inserted a straight line
 *       across un-routable terrain);</li>
 *   <li>a closure gap larger than {@link #MAX_CLOSURE_METERS} (the route
 *       did not actually return to the origin);</li>
 *   <li>a distance ratio outside {@code [MIN_DISTANCE_RATIO,
 *       MAX_DISTANCE_RATIO]} (the route is much shorter or much longer
 *       than the cyclist asked for);</li>
 *   <li>road reuse above {@link #MAX_REUSE_FRACTION} (more than half the
 *       distance is on edges traversed multiple times — an out-and-back
 *       dressed up as a loop);</li>
 *   <li>too few nodes to be a real loop;</li>
 *   <li>for a paved-only profile (fastbike/road), a significant share of
 *       distance on path/track/footway/unpaved surfaces — the cyclist
 *       cannot safely ride those on a road bike.</li>
 * </ul>
 *
 * <p>This is a HARD gate applied uniformly to all round-trip algorithms
 * (WAYPOINT, ISOCHRONE, GREEDY, ISO_GREEDY). A failing check sets
 * {@code foundTrack=null} and surfaces the rejection reason; the
 * algorithm-internal "best effort" must NOT silently downgrade to a
 * surprising route.
 *
 * <p>The thresholds intentionally err toward rejection: a clearly-bad
 * route is far worse user experience than "no route available, try a
 * different start or distance".
 */
public final class RoundTripQualityGate {

  /** Minimum acceptable {@code actualDistance / desiredDistance}. */
  public static final double MIN_DISTANCE_RATIO = 0.5;
  /** Maximum acceptable {@code actualDistance / desiredDistance}. */
  public static final double MAX_DISTANCE_RATIO = 1.8;
  /** Maximum acceptable fraction of distance on edges traversed more than once. */
  public static final double MAX_REUSE_FRACTION = 0.5;
  /** Maximum acceptable gap between the route's start and end points. */
  public static final int MAX_CLOSURE_METERS = 400;
  /** Minimum acceptable node count for a real loop (start + intermediate + close ≥ 4). */
  public static final int MIN_NODES = 4;

  /**
   * Maximum acceptable fraction of distance on profile-hostile edges. For a
   * paved profile, hostile means {@code highway=path|track|footway|...} or
   * a high-cost spike. 10% is intentionally tight: 10% of a 50km loop is
   * 5km of dirt/path the cyclist would hit unexpectedly.
   */
  public static final double MAX_HOSTILE_FRACTION = 0.10;

  /**
   * Costfactor above which a single edge is considered profile-hostile,
   * independent of its tags. Paved profiles return {@code costfactor=1.0}
   * for preferred ways (residential/cycleway/tertiary) and >4 only when
   * the way is something the profile actively avoids (track grade5,
   * unpaved primary, etc.).
   */
  public static final double HOSTILE_COSTFACTOR_THRESHOLD = 4.0;

  /**
   * Way-tag fragments that signal profile-hostile terrain for a paved
   * profile. We match by substring against {@code MessageData.wayKeyValues}
   * because the cost lookup may not have populated {@code costfactor} for
   * every edge (e.g. data-error fallbacks).
   */
  private static final String[] PAVED_PROFILE_HOSTILE_TAG_FRAGMENTS = {
    "highway=path",
    "highway=footway",
    "highway=bridleway",
    "highway=track",
    "highway=steps",
    "tracktype=grade3",
    "tracktype=grade4",
    "tracktype=grade5",
    "surface=ground",
    "surface=dirt",
    "surface=earth",
    "surface=grass",
    "surface=sand",
    "surface=mud",
    "surface=gravel",
    "surface=fine_gravel",
    "surface=unpaved",
    "surface=cobblestone",
    "surface=pebblestone"
  };

  private RoundTripQualityGate() { /* static-only */ }

  /**
   * Validate a generated round-trip track against production-safety gates.
   * Returns {@code null} when the track is acceptable; a human-readable
   * rejection reason otherwise. Callers must treat a non-null return as a
   * <em>hard</em> rejection: drop the track, log the reason, surface an
   * error.
   *
   * @param track            the generated round-trip
   * @param desiredDistance  the cyclist's target loop length (meters); pass
   *                         {@code 0} to skip the distance-ratio gate
   *                         (e.g. for non-round-trip routes)
   * @param profileName      the active profile name (used to decide whether
   *                         the paved-only checks apply)
   */
  /**
   * Convenience overload: assumes loop-style routes (no deliberate retracing).
   * Use {@link #validate(OsmTrack, double, String, boolean)} to relax the
   * reuse and ratio checks for same-way-back / out-and-back routes.
   */
  public static String validate(OsmTrack track, double desiredDistance, String profileName) {
    return validate(track, desiredDistance, profileName, false);
  }

  /**
   * Validate with a flag for same-way-back routes ({@code allowSamewayback}).
   * Same-way-back is by definition 100% retraced (out, then back the same
   * route) — the reuse gate must NOT fire on those, and the distance ratio
   * is computed against half-loop length rather than full-circle.
   */
  public static String validate(OsmTrack track, double desiredDistance, String profileName,
                                  boolean allowSamewayback) {
    if (track == null || track.nodes == null) return "no track";
    int n = track.nodes.size();
    if (n < MIN_NODES) {
      return "too few nodes (" + n + ", need ≥ " + MIN_NODES + ")";
    }

    // 1. Closure: a loop must return to its origin.
    int closure = track.nodes.get(0).calcDistance(track.nodes.get(n - 1));
    if (closure > MAX_CLOSURE_METERS) {
      return "closure=" + closure + "m exceeds " + MAX_CLOSURE_METERS + "m";
    }

    // 2. Distance ratio: not 1/3 of the requested length, not 2× either.
    // Same-way-back routes go out half the loop length then come back,
    // so their total distance ≈ desired (out is half, back is half). Use
    // the full-loop band either way; same-way-back doesn't change the
    // expected total distance, only the shape.
    if (desiredDistance > 0 && track.distance > 0) {
      double ratio = track.distance / desiredDistance;
      if (ratio < MIN_DISTANCE_RATIO || ratio > MAX_DISTANCE_RATIO) {
        return String.format(Locale.US, "distance ratio %.2f outside [%.1f, %.1f]",
          ratio, MIN_DISTANCE_RATIO, MAX_DISTANCE_RATIO);
      }
    }

    // 3. Reuse fraction: avoid out-and-backs dressed as loops. Skip this
    // check for explicit same-way-back routes — the cyclist asked for an
    // out-and-back, so 100% retrace is by design and not a failure.
    if (!allowSamewayback) {
      double reuse = GreedyRoundTripPlanner.finalTrackReuseRatio(track);
      if (reuse > MAX_REUSE_FRACTION) {
        return String.format(Locale.US, "reuse %.0f%% exceeds %.0f%%",
          reuse * 100.0, MAX_REUSE_FRACTION * 100.0);
      }
    }

    // 4. Beeline detection: the matcher marks waypoints as DIRECT when it
    // could not snap them to a road and the engine had to insert a
    // straight-line segment.
    List<MatchedWaypoint> mwps = track.matchedWaypoints;
    if (mwps != null) {
      for (MatchedWaypoint mwp : mwps) {
        if (mwp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT) {
          return "track contains beeline (waypoint marked DIRECT)";
        }
      }
    }

    // 5. Profile-hostile segments: only enforced for paved-only profiles.
    // For gravel/MTB any surface is fine; the cost-match score in
    // LoopQualityMetrics already covers profile alignment for those.
    if (isPavedProfile(profileName)) {
      String hostile = checkHostileSegmentsPaved(track);
      if (hostile != null) return hostile;
    }

    return null;
  }

  /**
   * Walk track edges and reject if the share of profile-hostile distance
   * (including edges with missing metadata, which we cannot verify safe)
   * exceeds {@link #MAX_HOSTILE_FRACTION}. Missing metadata is treated as
   * suspect, never as proof of quality.
   */
  private static String checkHostileSegmentsPaved(OsmTrack track) {
    double total = 0;
    double hostile = 0;
    double suspect = 0;

    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double segLen = a.calcDistance(b);
      total += segLen;

      // BRouter stores the per-edge MessageData on the target node b
      // (the edge a→b).
      MessageData m = b.message;
      if (m == null || m.wayKeyValues == null) {
        suspect += segLen;
        continue;
      }

      if (isHostileForPavedProfile(m)) {
        hostile += segLen;
      }
    }

    if (total <= 0) return null;

    double hostileFrac = hostile / total;
    if (hostileFrac > MAX_HOSTILE_FRACTION) {
      return String.format(Locale.US,
        "%.0f%% of distance on profile-hostile ways (max %.0f%%) — route uses path/track/unpaved that a road bike should avoid",
        hostileFrac * 100.0, MAX_HOSTILE_FRACTION * 100.0);
    }

    // Missing metadata is allowed in small doses (router fallbacks for
    // corrupt edges happen) but a paved-profile route mostly on edges we
    // can't verify is not safe to ship.
    double suspectFrac = suspect / total;
    if (suspectFrac > MAX_HOSTILE_FRACTION) {
      return String.format(Locale.US,
        "%.0f%% of distance on edges with missing/unknown metadata — cannot verify paved-ness for road-bike profile",
        suspectFrac * 100.0);
    }

    return null;
  }

  static boolean isHostileForPavedProfile(MessageData m) {
    if (m.costfactor > HOSTILE_COSTFACTOR_THRESHOLD) return true;
    String tags = m.wayKeyValues;
    if (tags == null) return false;
    for (String fragment : PAVED_PROFILE_HOSTILE_TAG_FRAGMENTS) {
      if (tags.contains(fragment)) return true;
    }
    return false;
  }

  /**
   * A profile is "paved-only" if its name matches the known road-bike
   * profile family. These profiles assume the cyclist is on a road bike
   * with narrow tyres and cannot safely ride dirt, gravel, or singletrack.
   * Gravel and MTB profiles deliberately use unpaved surfaces — the gate
   * is bypassed for them.
   */
  public static boolean isPavedProfile(String profileName) {
    if (profileName == null) return false;
    String n = profileName.toLowerCase();
    return n.contains("fastbike") || n.contains("road") || n.contains("racing");
  }
}
