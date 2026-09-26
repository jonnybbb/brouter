package btools.mapcreator;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import org.junit.Test;

public class ElevationRasterCoderTest {

  /**
   * The encoder pads its final flush, so a decoder legitimately reads a few bytes
   * past the payload — the round trip must stay exact under the EOF bound.
   */
  @Test
  public void roundTripSurvivesTheEofBound() throws IOException {
    ElevationRaster raster = denseRaster(60);
    ByteArrayOutputStream encoded = new ByteArrayOutputStream();
    new ElevationRasterCoder().encodeRaster(raster, encoded);

    ElevationRaster decoded = new ElevationRasterCoder()
      .decodeRaster(new ByteArrayInputStream(encoded.toByteArray()));
    assertArrayEquals(raster.eval_array, decoded.eval_array);
  }

  /**
   * A truncated body used to hang forever: EOF became unlimited zero bits and
   * decodeVarBits2 never terminated. It must throw instead.
   */
  @Test(timeout = 10000L)
  public void truncatedBodyFailsInsteadOfHanging() throws IOException {
    ElevationRaster raster = denseRaster(60);
    ByteArrayOutputStream encoded = new ByteArrayOutputStream();
    new ElevationRasterCoder().encodeRaster(raster, encoded);
    byte[] bytes = encoded.toByteArray();
    byte[] truncated = java.util.Arrays.copyOf(bytes, bytes.length / 2);

    try {
      new ElevationRasterCoder().decodeRaster(new ByteArrayInputStream(truncated));
      fail("expected truncated raster body to fail");
    } catch (IOException expected) {
      assertTrue(expected.getMessage(),
        expected.getMessage().contains("truncated"));
    }
  }

  /**
   * A corrupt or hostile header must not drive a multi-gigabyte allocation: the
   * plausibility bound rejects it before the pixel array is created.
   */
  @Test
  public void implausibleHeaderDimensionsAreRejected() throws IOException {
    ByteArrayOutputStream bos = new ByteArrayOutputStream();
    DataOutputStream dos = new DataOutputStream(bos);
    dos.writeInt(40000);
    dos.writeInt(40000); // 1.6e9 pixels: fits an int, far beyond any real cell
    dos.writeBoolean(false);
    dos.writeDouble(0.0);
    dos.writeDouble(0.0);
    dos.writeDouble(1.0);
    dos.writeShort(Short.MIN_VALUE);

    try {
      new ElevationRasterCoder().decodeRaster(new ByteArrayInputStream(bos.toByteArray()));
      fail("expected implausible raster dimensions to be rejected");
    } catch (IOException expected) {
      assertTrue(expected.getMessage(), expected.getMessage().contains("too large"));
    }
  }

  /** Dense, run-free values, so truncation always lands mid-value. */
  private static ElevationRaster denseRaster(int n) {
    ElevationRaster raster = new ElevationRaster();
    raster.ncols = n;
    raster.nrows = n;
    raster.halfcol = false;
    raster.noDataValue = ElevationCells.NODATA;
    raster.cellsize = 1.0 / n;
    raster.xllcorner = 0.0;
    raster.yllcorner = 0.0;
    raster.eval_array = new short[n * n];
    for (int i = 0; i < raster.eval_array.length; i++) {
      raster.eval_array[i] = (short) ((i * 131) % 2000 - 1000);
    }
    return raster;
  }
}
