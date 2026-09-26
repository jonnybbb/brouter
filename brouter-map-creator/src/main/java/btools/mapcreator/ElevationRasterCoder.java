package btools.mapcreator;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import btools.util.MixCoderDataInputStream;
import btools.util.MixCoderDataOutputStream;

//
// Encode/decode a raster
//

public class ElevationRasterCoder {
  public void encodeRaster(ElevationRaster raster, OutputStream os) throws IOException {
    DataOutputStream dos = new DataOutputStream(os);

    long t0 = System.currentTimeMillis();

    dos.writeInt(raster.ncols);
    dos.writeInt(raster.nrows);
    dos.writeBoolean(raster.halfcol);
    dos.writeDouble(raster.xllcorner);
    dos.writeDouble(raster.yllcorner);
    dos.writeDouble(raster.cellsize);
    dos.writeShort(raster.noDataValue);

    _encodeRaster(raster, os);
    long t1 = System.currentTimeMillis();

    System.out.println("finished encoding in " + (t1 - t0) + " ms");
  }

  public ElevationRaster decodeRaster(InputStream is) throws IOException {
    long t0 = System.currentTimeMillis();

    ElevationRaster raster = decodeHeader(is);
    int pixelCount = checkedPixelCount(raster);
    raster.eval_array = new short[pixelCount];

    decodeRasterRows(raster, is, (row, rowPixels) ->
      System.arraycopy(rowPixels, 0, raster.eval_array, row * raster.ncols, raster.ncols));

    raster.usingWeights = false; // raster.ncols > 6001;

    long t1 = System.currentTimeMillis();
    System.out.println("finished decoding in " + (t1 - t0) + " ms ncols=" + raster.ncols + " nrows=" + raster.nrows);
    return raster;
  }

  /** One decoded raster row, valid only for the duration of the callback. */
  @FunctionalInterface
  public interface RowConsumer {
    void row(int rowIndex, short[] rowPixels) throws IOException;
  }

  /**
   * Decode the raster body row by row into a single reused row buffer, so a consumer
   * that only inspects the values (e.g. read-back verification) never has to hold a
   * second full raster in memory. {@code header} must come from
   * {@link #decodeHeader} on the same stream.
   */
  public void decodeRasterRows(ElevationRaster header, InputStream is, RowConsumer consumer)
      throws IOException {
    checkedPixelCount(header);
    MixCoderDataInputStream mci = new MixCoderDataInputStream(is);
    int nrows = header.nrows;
    int ncols = header.ncols;
    int colstep = header.halfcol ? 2 : 1;
    short[] rowPixels = new short[ncols];

    for (int row = 0; row < nrows; row++) {
      java.util.Arrays.fill(rowPixels, (short) 0);
      for (int col = 0; col < ncols; col += colstep) {
        int code = mci.readMixed();

        // remap nodata
        int v30 = code == -1 ? ElevationCells.NODATA : (code < 0 ? code + 1 : code);
        if (header.usingWeights && v30 > ElevationCells.BORDER_SKIP) {
          v30 *= 2;
        }
        rowPixels[col] = (short) (v30);
      }
      if (header.halfcol) {
        for (int col = 1; col < ncols - 1; col += colstep) {
          int l = (int) rowPixels[col - 1];
          int r = (int) rowPixels[col + 1];
          short v30 = ElevationCells.NODATA;
          if (l > ElevationCells.BORDER_SKIP && r > ElevationCells.BORDER_SKIP) {
            v30 = (short) ((l + r) / 2);
          }
          rowPixels[col] = v30;
        }
      }
      consumer.row(row, rowPixels);
    }
  }

  /**
   * The largest plausible raster: a 1-arcsecond 5x5-degree cell is 18001x18001
   * (~324 M pixels). Anything bigger is a corrupt or hostile header, and would
   * otherwise drive a multi-gigabyte allocation before the body is even read.
   */
  private static final long MAX_RASTER_PIXELS = 400_000_000L;

  private static int checkedPixelCount(ElevationRaster raster) throws IOException {
    try {
      if (raster.ncols <= 0 || raster.nrows <= 0) {
        throw new ArithmeticException("non-positive raster dimensions");
      }
      int pixelCount = Math.multiplyExact(raster.ncols, raster.nrows);
      if (pixelCount > MAX_RASTER_PIXELS) {
        throw new ArithmeticException("implausible raster size");
      }
      return pixelCount;
    } catch (ArithmeticException e) {
      throw new IOException("raster dimensions are too large: " + raster.ncols
        + " x " + raster.nrows, e);
    }
  }

  public ElevationRaster decodeHeader(InputStream is) throws IOException {
    DataInputStream dis = new DataInputStream(is);
    ElevationRaster raster = new ElevationRaster();
    raster.ncols = dis.readInt();
    raster.nrows = dis.readInt();
    raster.halfcol = dis.readBoolean();
    raster.xllcorner = dis.readDouble();
    raster.yllcorner = dis.readDouble();
    raster.cellsize = dis.readDouble();
    raster.noDataValue = dis.readShort();
    return raster;
  }


  private void _encodeRaster(ElevationRaster raster, OutputStream os) throws IOException {
    MixCoderDataOutputStream mco = new MixCoderDataOutputStream(os);
    int nrows = raster.nrows;
    int ncols = raster.ncols;
    short[] pixels = raster.eval_array;
    int colstep = raster.halfcol ? 2 : 1;

    for (int row = 0; row < nrows; row++) {
      short lastval = ElevationCells.NODATA;
      for (int col = 0; col < ncols; col += colstep) {
        short val = pixels[row * ncols + col];
        if (val == ElevationCells.BORDER_SKIP) {
          val = lastval; // replace remaining (border) skips
        } else {
          lastval = val;
        }

        // remap nodata
        int code = val == ElevationCells.NODATA ? -1 : (val < 0 ? val - 1 : val);
        mco.writeMixed(code);
      }
    }
    mco.flush();
  }
}
