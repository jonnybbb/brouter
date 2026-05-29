package btools.router;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

import org.junit.Assume;

/**
 * On-demand fetcher for the {@code .rd5} routing segment tiles the opt-in
 * loop-quality suites depend on. Replaces the old
 * {@code download-loop-test-segments.sh}: instead of a manual pre-step, each
 * suite calls {@link #ensureAvailable(File, String)} in its setup and the tile
 * is downloaded on first run, then reused (the file is cached on disk and never
 * re-fetched while present).
 *
 * <p>Design notes:
 * <ul>
 *   <li>Idempotent — a present tile is left untouched, so repeat runs are free.</li>
 *   <li>Fork-safe — downloads to a unique {@code .part} file and atomically
 *       renames into place, so parallel Gradle test JVMs can't read a
 *       half-written tile.</li>
 *   <li>Skip, don't fail, when the network is unavailable — offline developers
 *       get a JUnit {@code Assume} skip rather than a red build.</li>
 *   <li>Escape hatch — {@code -Dloop.segments.nodownload=true} disables the
 *       fetch and falls back to a plain existence check, for hermetic CI that
 *       provisions tiles out-of-band.</li>
 * </ul>
 */
final class LoopTestSegments {

  private static final String BASE_URL = "http://brouter.de/brouter/segments4/";
  private static final int CONNECT_TIMEOUT_MS = 30_000;
  private static final int READ_TIMEOUT_MS = 120_000;

  /**
   * Half-width (degrees) of the start neighbourhood a loop may reach. A 100 km
   * loop extends at most ~40-50 km from the start; 0.7° (~78 km) is a safe
   * bound. Used to pull in adjacent 5°×5° tiles when the start sits near a tile
   * boundary (e.g. Dreieich at lat 50.0, Lozère/Mallorca loops running north).
   */
  private static final double MARGIN_DEG = 0.7;

  private LoopTestSegments() {
  }

  /**
   * Ensure every {@code .rd5} tile a loop from {@code region} could touch is
   * present. The home tile is a hard requirement ({@link Assume}-skip if it
   * cannot be fetched); boundary neighbours are best-effort (a loop that never
   * crosses the boundary does not need them, so a failed neighbour fetch must
   * not skip the test).
   */
  static void ensureRegion(File segDir, LoopTestRegion region) {
    for (String tile : tilesFor(region.lon, region.lat, MARGIN_DEG)) {
      if (!tile.equals(region.segmentFile)) {
        fetch(segDir, tile); // best-effort neighbour
      }
    }
    ensureAvailable(segDir, region.segmentFile);
  }

  /** Tiles overlapping the [lon±margin] × [lat±margin] box around a start. */
  static java.util.Set<String> tilesFor(double lon, double lat, double margin) {
    java.util.LinkedHashSet<String> tiles = new java.util.LinkedHashSet<>();
    for (double dLon : new double[] {-margin, 0, margin}) {
      for (double dLat : new double[] {-margin, 0, margin}) {
        tiles.add(tileName(lon + dLon, lat + dLat));
      }
    }
    return tiles;
  }

  /** Name of the 5°×5° segment tile containing (lon, lat), e.g. {@code E10_N45.rd5}. */
  static String tileName(double lon, double lat) {
    int lonBase = (int) Math.floor(lon / 5.0) * 5;
    int latBase = (int) Math.floor(lat / 5.0) * 5;
    String lonPart = lonBase >= 0 ? "E" + lonBase : "W" + (-lonBase);
    String latPart = latBase >= 0 ? "N" + latBase : "S" + (-latBase);
    return lonPart + "_" + latPart + ".rd5";
  }

  /**
   * Ensure {@code tile} is present in {@code segDir}, downloading it if needed.
   * {@link Assume}-skips the calling test when the tile is absent and cannot be
   * fetched (offline, fetch disabled, or download error).
   */
  static void ensureAvailable(File segDir, String tile) {
    Assume.assumeTrue(
      "Segment tile " + tile + " unavailable in " + segDir.getAbsolutePath()
        + " and could not be downloaded — check network, or pre-provision it"
        + " (see LoopTestSegments).",
      fetch(segDir, tile));
  }

  /**
   * Ensure {@code tile} is present, downloading if needed. Returns {@code true}
   * if the tile is available afterwards, {@code false} otherwise. Never throws.
   */
  static boolean fetch(File segDir, String tile) {
    File target = new File(segDir, tile);
    if (target.isFile() && target.length() > 0) {
      return true;
    }
    if (Boolean.getBoolean("loop.segments.nodownload")) {
      return false;
    }
    if (!segDir.exists() && !segDir.mkdirs()) {
      System.out.println("[segments] cannot create " + segDir.getAbsolutePath());
      return false;
    }

    String url = BASE_URL + tile;
    Path part = new File(segDir, tile + ".part." + ProcessHandle.current().pid()).toPath();
    System.out.println("[segments] fetching " + url);
    try {
      HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
      conn.setConnectTimeout(CONNECT_TIMEOUT_MS);
      conn.setReadTimeout(READ_TIMEOUT_MS);
      conn.setInstanceFollowRedirects(true);
      int code = conn.getResponseCode();
      if (code != HttpURLConnection.HTTP_OK) {
        System.out.println("[segments] HTTP " + code + " for " + url);
        conn.disconnect();
        return false;
      }
      try (InputStream in = conn.getInputStream()) {
        Files.copy(in, part, StandardCopyOption.REPLACE_EXISTING);
      }
      conn.disconnect();
      if (Files.size(part) <= 0) {
        Files.deleteIfExists(part);
        return false;
      }
      // Atomic publish; if a sibling fork beat us, its copy is equivalent.
      Files.move(part, target.toPath(),
        StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
      System.out.println("[segments] ready " + target.getName() + " (" + target.length() + " bytes)");
      return true;
    } catch (IOException e) {
      System.out.println("[segments] download failed for " + tile + ": " + e.getMessage());
      try {
        Files.deleteIfExists(part);
      } catch (IOException ignore) {
        // best-effort cleanup
      }
      return target.isFile() && target.length() > 0;
    }
  }
}
