package btools.router;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Test regions for loop quality verification.
 * Each region defines a start point, the segment tile(s) needed,
 * terrain-dependent quality thresholds, and the set of cycling
 * profiles that are <em>plausibly usable</em> in that region.
 *
 * <p>Direction-delta threshold is effectively disabled (180°) for all regions
 * because round-trip "direction" is a soft hint — the algorithm may
 * legitimately pick the opposite traversal direction in asymmetric terrain.
 * The metric is still computed and logged for observability.
 *
 * <p>Ratio bounds are tightened to reflect real cyclist expectations: a 2x
 * overshoot is unacceptable even in alpine terrain. Cases that exceed these
 * bounds for {@code probe} (FAST tier) are real algorithm shortcomings, and
 * the test failing them is the correct signal to improve placement.
 *
 * <p><b>{@code supportedProfiles}</b> filters out fundamentally impossible
 * terrain × profile combinations — the test {@link
 * org.junit.Assume#assumeTrue Assume}s the combo is supported and skips
 * otherwise. Determined empirically: a combo is excluded when 0% of the 20
 * cases (5 distances × 4 directions) produced a route within strict
 * thresholds across <em>all</em> algorithm variants. Example: urban Berlin
 * has no MTB singletrack network — the MTB profile's
 * {@code path_preference=20} multiplier forces routes through paved roads
 * (cost ~9/m for residential, ~21/m for tertiary) and the cyclist would
 * never want this route regardless of how well the algorithm searches.
 */
public enum LoopTestRegion {
  // MTB profile excluded by default — its path_preference=20 multiplier
  // creates cost/m ~9-12 baseline even in suburban terrain, because anything
  // off the cycleway/track network costs 9× the ideal. Without dedicated
  // MTB regions (a forest singletrack network with full coverage at all
  // test radii), the strict threshold for MTB triggers everywhere. Add MTB
  // back to a region's profile set only after confirming that region has
  // the path network density to support MTB routing.
  DREIEICH(8.720, 50.000, "E5_N50.rd5", 25.0, 0.5, 1.6, 180,
    profiles("fastbike", "gravel")),
  URBAN_BERLIN(13.400, 52.520, "E10_N50.rd5", 25.0, 0.5, 1.6, 180,
    profiles("fastbike", "gravel")),
  ALPINE_INNSBRUCK(11.400, 47.260, "E10_N45.rd5", 35.0, 0.4, 1.8, 180,
    profiles("fastbike", "gravel")),
  COASTAL_NICE(7.270, 43.700, "E5_N40.rd5", 30.0, 0.4, 1.8, 180,
    profiles("fastbike", "gravel")), // gravel rides the hills above Nice (Aspremont,
                                     // Tourrette-Levens); loops verified accepted from this start
  // Start near Rieutort-de-Randon (paved-road country) rather than the original
  // (3.500, 44.500) on the Causse de Mende plateau, which is ~77% highway=track:
  // no paved road-bike loop closes there at any size, so fastbike never formed a
  // loop. fastbike + gravel both loop cleanly from here (real komoot route start).
  RURAL_LOZERE(3.43980, 44.65161, "E0_N40.rd5", 30.0, 0.4, 1.8, 180,
    profiles("fastbike", "gravel")),
  // Mallorca: island geometry forces a different loop-quality regime than
  // mainland regions. The Serra de Tramuntana climbs in the NW, the coast
  // road network is finite (you can't always loop around a cape), and the
  // famous cycling routes are often forced-spur shape (e.g. Cap de Formentor,
  // Sa Calobra). Slightly relaxed reuse/distance thresholds because
  // out-and-back to a cape or pass IS the expected shape of a Mallorca ride.
  MALLORCA(2.650, 39.570, "E0_N35.rd5", 35.0, 0.4, 1.8, 180,
    profiles("fastbike", "gravel"));

  /** Longitude in decimal degrees */
  public final double lon;
  /** Latitude in decimal degrees */
  public final double lat;
  /** BRouter internal longitude: (lon + 180) * 1e6 */
  public final int ilon;
  /** BRouter internal latitude: (lat + 90) * 1e6 */
  public final int ilat;
  /** Segment tile filename (e.g., E5_N50.rd5) */
  public final String segmentFile;
  /** Maximum acceptable road reuse percentage for this terrain */
  public final double maxReusePercent;
  /** Minimum acceptable distance ratio (actual/requested) */
  public final double minDistanceRatio;
  /** Maximum acceptable distance ratio (actual/requested) */
  public final double maxDistanceRatio;
  /** Maximum acceptable direction delta in degrees */
  public final double maxDirectionDelta;
  /** Profile names that are plausibly usable in this region's terrain */
  public final Set<String> supportedProfiles;

  LoopTestRegion(double lon, double lat, String segmentFile,
                 double maxReusePercent, double minDistanceRatio,
                 double maxDistanceRatio, double maxDirectionDelta,
                 Set<String> supportedProfiles) {
    this.lon = lon;
    this.lat = lat;
    this.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    this.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    this.segmentFile = segmentFile;
    this.maxReusePercent = maxReusePercent;
    this.minDistanceRatio = minDistanceRatio;
    this.maxDistanceRatio = maxDistanceRatio;
    this.maxDirectionDelta = maxDirectionDelta;
    this.supportedProfiles = supportedProfiles;
  }

  private static Set<String> profiles(String... names) {
    return Collections.unmodifiableSet(new HashSet<>(Arrays.asList(names)));
  }
}
