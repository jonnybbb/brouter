package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class ElevationCoverageTest {
  @Rule
  public TemporaryFolder tmp = new TemporaryFolder();

  private Path convert(boolean absent, boolean invalid, int height) throws IOException {
    File directory = tmp.newFolder();
    PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(6, 6);
    for (int x = 30; x <= 34; x++) {
      for (int y = 20; y <= 24; y++) {
        if (absent && x == 32 && y == 22) {
          continue;
        }
        builder.put(6, x, y, TerrainTiles.constantPng(64,
          invalid && x == 32 && y == 22 ? -32768 : height));
      }
    }
    try (PmTilesArchive archive = PmTilesArchive.open(builder.asByteSource())) {
      ConvertMapterhornTile.convertOne(archive, 6, null, directory,
        0, 45, 4, null, 1, false, 64);
    }
    return directory.toPath().resolve(ElevationCells.cellFileName(0, 45));
  }

  @Test
  public void actualConversionBindsSourceIdentityGeometryAndValidNegativeTerrain() throws IOException {
    Path file = convert(false, false, -430);
    ElevationCoverage.Evidence evidence = ElevationCoverage.verify(file);
    assertEquals("full", evidence.sampling);
    assertEquals(21, evidence.geometry.ncols);
    assertEquals(6, evidence.zoom);
    assertEquals(64, evidence.tileSize);
    assertEquals(64, evidence.archiveId.length());
    assertFalse(evidence.sourceTiles.isEmpty());
    for (int row = 0; row < 21; row++) {
      for (int col = 0; col < 21; col++) {
        assertEquals(ElevationCoverage.VALID, evidence.flags(row, col));
      }
    }
  }

  @Test
  public void absenceAndInvalidSourceRemainDifferentCauses() throws IOException {
    ElevationCoverage.Evidence absent = ElevationCoverage.verify(convert(true, false, 100));
    ElevationCoverage.Evidence invalid = ElevationCoverage.verify(convert(false, true, 100));
    assertEquals("absent", absent.sourceTiles.get("6/32/22"));
    assertTrue(invalid.sourceTiles.get("6/32/22").endsWith("invalid=4096"));
    boolean found = false;
    for (int row = 0; row < 21; row++) {
      for (int col = 0; col < 21; col++) {
        if ((absent.flags(row, col) & ElevationCoverage.VALID) == 0) {
          assertEquals(ElevationCoverage.ABSENT_SOURCE, absent.flags(row, col));
          assertEquals(ElevationCoverage.INVALID_SOURCE, invalid.flags(row, col));
          found = true;
        }
      }
    }
    assertTrue("fixture must contain missing support", found);
  }

  @Test
  public void plantedHolesFailIncludingEdgesDiagonalsTailsAndWholeTiles() throws IOException {
    for (String shape : new String[]{"pixel", "3x3", "diagonal", "tail", "edge", "whole"}) {
      Path file = convert(false, false, 100);
      ElevationRaster raster = read(file);
      for (int row = 0; row < 21; row++) {
        for (int col = 0; col < 21; col++) {
          boolean hole = shape.equals("pixel") && row == 10 && col == 10
            || shape.equals("3x3") && row >= 8 && row <= 10 && col >= 8 && col <= 10
            || shape.equals("diagonal") && row == col
            || shape.equals("tail") && (row == 10 || col == 10 && row > 10)
            || shape.equals("edge") && row == 0
            || shape.equals("whole");
          if (hole) {
            raster.eval_array[row * 21 + col] = Short.MIN_VALUE;
          }
        }
      }
      write(file, raster);
      reject(file, "UNEXPECTED_NODATA");
    }
  }

  @Test
  public void malformedConverterProvenanceCannotPassCoverageValidation() throws IOException {
    Path file = convert(false, false, 100);
    Path metadata = Path.of(file + MapterhornOutputMetadata.SUFFIX);
    String text = Files.readString(metadata);
    Files.writeString(metadata, text.replaceAll("(?m)^modified=.*$", "modified=invalid"));
    reject(file, "COVERAGE_PROVENANCE");
  }

  @Test
  public void finiteDamageFailsTheByteBinding() throws IOException {
    Path file = convert(false, false, 100);
    ElevationRaster raster = read(file);
    raster.eval_array[100] = -900; // finite, above the old source cutoff
    write(file, raster);
    reject(file, "COVERAGE_HASH");
  }

  @Test
  public void missingTruncatedAndStaleEvidenceFailClosed() throws IOException {
    Path first = convert(false, false, 100);
    Path second = convert(false, false, 200);
    Path coverage = Path.of(second + ElevationCoverage.SUFFIX);
    Files.copy(Path.of(first + ElevationCoverage.SUFFIX), coverage,
      java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    reject(second, "COVERAGE_HASH");
    Files.write(coverage, Arrays.copyOf(Files.readAllBytes(coverage), 12));
    try {
      ElevationCoverage.verify(second);
      fail("truncated evidence must fail");
    } catch (IOException expected) {
      // A gzip/EOF failure is fatal, never an empty or successful audit.
    }
    Files.delete(coverage);
    reject(second, "COVERAGE_MISSING");
  }

  @Test
  public void wrongGeometryAndTruncatedBefFailClosed() throws IOException {
    Path file = convert(false, false, 100);
    byte[] original = Files.readAllBytes(file);
    ElevationRaster raster = read(file);
    raster.xllcorner += 5;
    write(file, raster);
    reject(file, "COVERAGE_GEOMETRY");
    Files.write(file, Arrays.copyOf(original, 20));
    try {
      ElevationCoverage.verify(file);
      fail("truncated BEF must fail");
    } catch (IOException expected) {
      // Decoder EOF must escape rather than become a passing result.
    }
  }

  private static ElevationRaster read(Path file) throws IOException {
    try (InputStream input = Files.newInputStream(file)) {
      return new ElevationRasterCoder().decodeRaster(input);
    }
  }

  private static void write(Path file, ElevationRaster raster) throws IOException {
    try (OutputStream output = Files.newOutputStream(file)) {
      new ElevationRasterCoder().encodeRaster(raster, output);
    }
  }

  private static void reject(Path file, String code) throws IOException {
    try {
      ElevationCoverage.verify(file);
      fail("expected " + code);
    } catch (IOException expected) {
      assertTrue(expected.toString(), expected.getMessage().contains(code));
    }
  }
}
