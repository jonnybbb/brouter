package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.io.IOException;
import org.junit.Test;

/** Actual PMTiles-to-raster regressions, independent of the BEF audit geometry. */
public class ConverterCoverageRegressionTest {
  private static final int RESOLUTION = 360;

  private static double latitude(int tileY) {
    return Math.toDegrees(Math.atan(Math.sinh(Math.PI * (1 - 2 * tileY / 4096.0))));
  }

  private static double longitude(int tileX) {
    return tileX / 4096.0 * 360 - 180;
  }

  private static int cellStart(double coordinate) {
    return (int) Math.floor(coordinate / 5) * 5;
  }

  private static PmTilesTestArchive tiles(int tileX, int tileY, boolean corner) throws IOException {
    byte[] valid = TerrainTiles.constantPng(512, 100);
    PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(12, 12);
    for (int x = tileX - 2; x <= tileX + 1; x++) {
      for (int y = tileY - 2; y <= tileY + 1; y++) {
        boolean absent = x == tileX - 1 && y == tileY - 1;
        if (corner) {
          absent |= (x == tileX && y == tileY - 1) || (x == tileX - 1 && y == tileY);
        }
        if (!absent) {
          builder.put(12, x, y, valid);
        }
      }
    }
    return builder;
  }

  private static ElevationRaster convert(PmTilesTestArchive builder, int tileX, int tileY,
                                          int lonStart) throws IOException {
    int latStart = cellStart(latitude(tileY));
    try (PmTilesArchive archive = PmTilesArchive.open(builder.asByteSource());
         ConvertMapterhornTile converter = new ConvertMapterhornTile(archive, 12, null, 512)) {
      ElevationRaster raster = converter.buildRaster(lonStart, latStart, RESOLUTION,
        new double[]{longitude(tileX - 2), latitude(tileY + 2),
          longitude(tileX + 2), latitude(tileY - 2)});
      assertNotNull(raster);
      return raster;
    }
  }

  private static void assertHalfCovered(ElevationRaster raster, int tileX, int tileY, int lonStart) {
    int north = cellStart(latitude(tileY)) + 5;
    int column = (int) Math.round((longitude(tileX) - lonStart) * RESOLUTION);
    int from = (int) Math.ceil((north - latitude(tileY - 1)) * RESOLUTION) + 2;
    int to = (int) Math.floor((north - latitude(tileY)) * RESOLUTION) - 2;
    for (int row = from; row <= to; row++) {
      assertEquals("50 percent coverage x=" + tileX + " y=" + tileY + " row=" + row,
        100, raster.eval_array[row * raster.ncols + column]);
    }
  }

  @Test
  public void exactlyHalfCoveredPixelsKeepElevationInBothHemispheres() throws IOException {
    for (int tileY : new int[]{1568, 2528}) {
      for (int tileX : new int[]{128, 256, 1024, 2048, 2176}) {
        int start = cellStart(longitude(tileX));
        assertHalfCovered(convert(tiles(tileX, tileY, false), tileX, tileY, start),
          tileX, tileY, start);
      }
    }
  }

  @Test
  public void genuinelyLessThanHalfAtMissingTileCornerStaysNodata() throws IOException {
    int tileX = 2176;
    int tileY = 1568;
    int start = cellStart(longitude(tileX));
    ElevationRaster raster = convert(tiles(tileX, tileY, true), tileX, tileY, start);
    int column = (int) Math.round((longitude(tileX) - start) * RESOLUTION);
    int row = (int) Math.ceil((cellStart(latitude(tileY)) + 5 - latitude(tileY)) * RESOLUTION);
    assertEquals(Short.MIN_VALUE, raster.eval_array[row * raster.ncols + column]);
    assertEquals(100, raster.eval_array[(row + 5) * raster.ncols + column + 5]);
  }

  @Test
  public void clippingArchiveToThePresentHalfRetainsElevation() throws IOException {
    int tileX = 256;
    int tileY = 1568;
    int start = cellStart(longitude(tileX));
    PmTilesTestArchive builder = tiles(tileX, tileY, false)
      .bounds(longitude(tileX), -85, 180, 85);
    assertHalfCovered(convert(builder, tileX, tileY, start), tileX, tileY, start);
  }

  @Test
  public void adjacentCellsAgreeOnTheirSharedMeridian() throws IOException {
    int tileX = 2048; // Greenwich: both a source-tile and an output-cell seam.
    int tileY = 1568;
    PmTilesTestArchive builder = tiles(tileX, tileY, false);
    ElevationRaster west = convert(builder, tileX, tileY, -5);
    ElevationRaster east = convert(builder, tileX, tileY, 0);
    for (int row = 0; row < east.nrows; row++) {
      assertEquals("shared seam row=" + row,
        west.eval_array[row * west.ncols + west.ncols - 1], east.eval_array[row * east.ncols]);
    }
    assertHalfCovered(east, tileX, tileY, 0);
  }
  @Test
  public void allMissingTileCornerPatternsMatchAnIndependentAreaReference() throws IOException {
    int tileX = 2176;
    byte[] valid = TerrainTiles.constantPng(512, 100);
    for (int tileY : new int[]{1568, 2528}) {
      for (int pattern = 0; pattern < 16; pattern++) {
        PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(12, 12);
        for (int x = tileX - 2; x <= tileX + 1; x++) {
          for (int y = tileY - 2; y <= tileY + 1; y++) {
            int quadrant = (y == tileY ? 2 : 0) + (x == tileX ? 1 : 0);
            boolean central = (x == tileX - 1 || x == tileX) && (y == tileY - 1 || y == tileY);
            if (!central || (pattern & (1 << quadrant)) != 0) {
              builder.put(12, x, y, valid);
            }
          }
        }
        int west = cellStart(longitude(tileX));
        int north = cellStart(latitude(tileY)) + 5;
        ElevationRaster raster = convert(builder, tileX, tileY, west);
        int column = (int) Math.round((longitude(tileX) - west) * RESOLUTION);
        int nearest = (int) Math.floor((north - latitude(tileY)) * RESOLUTION);
        for (int row = nearest; row <= nearest + 1; row++) {
          // Analytic intersection of four constant tile rectangles. This reference
          // never uses converter weights, coverage flags or the BEF hole heuristic.
          double top = north - (row - 0.5) / RESOLUTION;
          double bottom = north - (row + 0.5) / RESOLUTION;
          double yTop = Math.log(Math.tan(Math.PI / 4 + Math.toRadians(top) / 2));
          double yBottom = Math.log(Math.tan(Math.PI / 4 + Math.toRadians(bottom) / 2));
          double boundary = Math.PI * (1 - 2 * tileY / 4096.0);
          double northFraction = Math.max(0, Math.min(1, (yTop - boundary) / (yTop - yBottom)));
          int northTiles = Integer.bitCount(pattern & 3);
          int southTiles = Integer.bitCount(pattern & 12);
          double coverage = northTiles == southTiles ? northTiles / 2.0
            : (northTiles * northFraction + southTiles * (1 - northFraction)) / 2;
          int expected = coverage >= 0.5 ? 100 : Short.MIN_VALUE;
          assertEquals("pattern=" + pattern + " y=" + tileY + " row=" + row,
            expected, raster.eval_array[row * raster.ncols + column]);
        }
      }
    }
  }

  @Test
  public void equatorialSampleWithOneHemisphereOfSupportKeepsElevation() throws IOException {
    int resolution = 1200;
    int tileX = 2176;
    byte[] valid = TerrainTiles.constantPng(512, 100);
    for (boolean northOnly : new boolean[]{true, false}) {
      PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(12, 12);
      for (int x = tileX - 2; x <= tileX + 1; x++) {
        for (int y = 2046; y <= 2049; y++) {
          if ((y < 2048) == northOnly) {
            builder.put(12, x, y, valid);
          }
        }
      }
      double[] bbox = {longitude(tileX - 2), latitude(2050), longitude(tileX + 2), latitude(2046)};
      int west = cellStart(longitude(tileX));
      int column = (int) Math.round((longitude(tileX) - west) * resolution);
      try (PmTilesArchive archive = PmTilesArchive.open(builder.asByteSource());
           ConvertMapterhornTile converter = new ConvertMapterhornTile(archive, 12, null, 512)) {
        // The equator is the bottom row of the northern cell and the top row of the
        // southern one; both footprints are split exactly in half by the tile seam.
        ElevationRaster north = converter.buildRaster(west, 0, resolution, bbox);
        assertNotNull(north);
        assertEquals("north cell, northOnly=" + northOnly,
          100, north.eval_array[(north.nrows - 1) * north.ncols + column]);
        ElevationRaster south = converter.buildRaster(west, -5, resolution, bbox);
        assertNotNull(south);
        assertEquals("south cell, northOnly=" + northOnly,
          100, south.eval_array[column]);
      }
    }
  }

}
