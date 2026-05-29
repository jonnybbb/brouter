package btools.router;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.file.Files;
import java.util.Set;

import org.junit.Assume;
import org.junit.Test;

/**
 * Unit tests for {@link LoopTestSegments}. Tile-name/coverage logic runs
 * offline on every build; the live download is opt-in (network) via
 * {@code -Dloop.tests=true} and fetches the smallest tile to a temp dir.
 */
public class LoopTestSegmentsTest {

  @Test
  public void tileNameMapsCoordinatesToTile() {
    assertEquals("E10_N45.rd5", LoopTestSegments.tileName(11.40, 47.26)); // Innsbruck
    assertEquals("E0_N35.rd5", LoopTestSegments.tileName(2.65, 39.57));   // Mallorca
    assertEquals("E5_N50.rd5", LoopTestSegments.tileName(8.72, 50.00));   // Dreieich
    assertEquals("E0_N40.rd5", LoopTestSegments.tileName(3.50, 44.50));   // Lozère
    assertEquals("W5_N45.rd5", LoopTestSegments.tileName(-3.0, 45.0));    // west of Greenwich
  }

  @Test
  public void boundaryStartPullsInNeighbourTile() {
    // Dreieich sits on the lat-50 tile boundary: a southbound loop needs N45 too.
    Set<String> dreieich = LoopTestSegments.tilesFor(8.72, 50.00, 0.7);
    assertTrue(dreieich.contains("E5_N50.rd5"));
    assertTrue(dreieich.contains("E5_N45.rd5"));

    // Mid-tile start needs exactly one tile.
    Set<String> innsbruck = LoopTestSegments.tilesFor(11.40, 47.26, 0.7);
    assertEquals(1, innsbruck.size());
    assertTrue(innsbruck.contains("E10_N45.rd5"));
  }

  @Test
  public void fetchDownloadsMissingTile() throws Exception {
    Assume.assumeTrue("network download is opt-in — run with -Dloop.tests=true",
      Boolean.getBoolean("loop.tests"));
    File tmp = Files.createTempDirectory("seg-fetch-test").toFile();
    tmp.deleteOnExit();
    String smallest = "E0_N35.rd5"; // ~16 MB, the lightest test tile
    assertTrue("fetch should report success", LoopTestSegments.fetch(tmp, smallest));
    File got = new File(tmp, smallest);
    assertTrue("tile should exist after fetch", got.isFile());
    assertTrue("tile should be non-trivial", got.length() > 1_000_000);
    // Second call is a no-op (already present).
    assertTrue(LoopTestSegments.fetch(tmp, smallest));
    got.delete();
  }
}
