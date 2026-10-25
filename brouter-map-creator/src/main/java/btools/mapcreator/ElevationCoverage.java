package btools.mapcreator;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.channels.Channels;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

/** Source-derived validity evidence. Absence is a cause, never a claim of water. */
public final class ElevationCoverage {
  public static final String SUFFIX = ".coverage.gz";
  public static final int VALID = 1;
  public static final int ABSENT_SOURCE = 2;
  public static final int INVALID_SOURCE = 4;
  public static final int OUTSIDE = 8;
  private static final String MAGIC = "BEF-COVERAGE-1";
  private static final int MAX_PIXELS = 400_000_000;
  private static final int MAX_SOURCE_TILES = 1_000_000;

  private ElevationCoverage() {
  }

  /** Bulk integrity check for release publication; any unreadable or invalid cell is fatal. */
  public static void main(String[] args) throws IOException {
    if (args.length == 0) {
      throw new IllegalArgumentException("usage: ElevationCoverage cell.bef [cell.bef ...]");
    }
    for (String argument : args) {
      Evidence evidence = verify(Path.of(argument));
      long valid = 0;
      long absent = 0;
      long invalid = 0;
      for (byte flag : evidence.flags) {
        if ((flag & VALID) != 0) valid++;
        if ((flag & ABSENT_SOURCE) != 0) absent++;
        if ((flag & INVALID_SOURCE) != 0) invalid++;
      }
      System.out.println("elevation-coverage PASS " + argument + " archive=" + evidence.archiveId
        + " valid=" + valid + " absentSource=" + absent + " invalidSource=" + invalid);
    }
  }

  /** Immutable evidence for the exact decoded BEF; flags use north-first raster order. */
  public static final class Evidence {
    public final String archiveId;
    public final String befSha256;
    public final String sampling;
    public final int zoom;
    public final int tileSize;
    public final ElevationRaster geometry;
    public final Map<String, String> sourceTiles;
    private final byte[] flags;

    private Evidence(String archiveId, String befSha256, String sampling, int zoom,
                     int tileSize, ElevationRaster geometry, Map<String, String> sourceTiles,
                     byte[] flags) {
      this.archiveId = archiveId;
      this.befSha256 = befSha256;
      this.sampling = sampling;
      this.zoom = zoom;
      this.tileSize = tileSize;
      this.geometry = geometry;
      this.sourceTiles = Collections.unmodifiableMap(sourceTiles);
      this.flags = flags;
    }

    /** All four samples used by ElevationRaster.getElevation, including zero-weight corners. */
    public int supportFlags(int ilon, int ilat) {
      double lon = ilon / 1000000.0 - 180.0;
      double lat = ilat / 1000000.0 - 90.0;
      int col = (int) ((lon - geometry.xllcorner) / geometry.cellsize - 0.5);
      int row = (int) ((lat - geometry.yllcorner) / geometry.cellsize - 0.5);
      col = Math.max(0, Math.min(geometry.ncols - 2, col));
      row = Math.max(0, Math.min(geometry.nrows - 2, row));
      int north = geometry.nrows - 2 - row;
      return flags(north, col) | flags(north, col + 1)
        | flags(north + 1, col) | flags(north + 1, col + 1);
    }

    public int flags(int row, int column) {
      return flags[Math.addExact(Math.multiplyExact(row, geometry.ncols), column)];
    }
  }

  static void write(Path bef, MapterhornOutputMetadata.Config config, String befSha256,
                    ElevationRaster raster, byte[] flags, Map<String, String> sourceTiles)
      throws IOException {
    if (flags.length != checkedPixels(raster)) {
      throw new IOException("COVERAGE_GEOMETRY: source mask size differs from BEF");
    }
    Path target = Path.of(bef + SUFFIX);
    MapterhornFiles.rejectSymbolicLink(target, MapterhornFiles.attributesIfPresent(target), "coverage");
    Path temporary = Files.createTempFile(bef.toAbsolutePath().getParent(), "coverage-", ".tmp");
    try {
      try (FileChannel channel = FileChannel.open(temporary, StandardOpenOption.WRITE);
           GZIPOutputStream gzip = new GZIPOutputStream(new BufferedOutputStream(Channels.newOutputStream(channel)));
           DataOutputStream out = new DataOutputStream(gzip)) {
        out.writeUTF(MAGIC);
        out.writeInt(MapterhornOutputMetadata.CONVERTER_VERSION);
        out.writeUTF(config.archiveId);
        out.writeUTF(befSha256);
        out.writeUTF(config.coverage);
        out.writeInt(config.zoom);
        out.writeInt(config.tileSize);
        out.writeInt(raster.ncols);
        out.writeInt(raster.nrows);
        out.writeDouble(raster.xllcorner);
        out.writeDouble(raster.yllcorner);
        out.writeDouble(raster.cellsize);
        out.writeInt(sourceTiles.size());
        for (Map.Entry<String, String> tile : sourceTiles.entrySet()) {
          out.writeUTF(tile.getKey());
          out.writeUTF(tile.getValue());
        }
        out.write(flags);
        out.flush();
        gzip.finish();
        gzip.flush();
        channel.force(false);
      }
      MapterhornFiles.atomicMove(temporary, target);
    } finally {
      Files.deleteIfExists(temporary);
    }
  }

  /** Decode every output sample against recorded source validity and verify every BEF byte. */
  public static Evidence verify(Path bef) throws IOException {
    Path coverage = Path.of(bef + SUFFIX);
    try (DataInputStream in = new DataInputStream(new GZIPInputStream(open(coverage)))) {
      if (!MAGIC.equals(in.readUTF()) || in.readInt() != MapterhornOutputMetadata.CONVERTER_VERSION) {
        throw new IOException("COVERAGE_VERSION: reconvert " + bef);
      }
      String archive = requireSha256(in.readUTF(), "archive");
      String expectedHash = requireSha256(in.readUTF(), "BEF");
      String sampling = in.readUTF();
      if (!(sampling.equals("full") || sampling.startsWith("bbox="))) {
        throw new IOException("COVERAGE_GEOMETRY: invalid sampling descriptor");
      }
      int zoom = in.readInt();
      int tileSize = in.readInt();
      if (zoom < 0 || zoom > 30 || tileSize <= 0 || tileSize > 4096) {
        throw new IOException("COVERAGE_GEOMETRY: invalid source grid");
      }
      ElevationRaster geometry = new ElevationRaster();
      geometry.ncols = in.readInt();
      geometry.nrows = in.readInt();
      geometry.xllcorner = in.readDouble();
      geometry.yllcorner = in.readDouble();
      geometry.cellsize = in.readDouble();
      geometry.noDataValue = Short.MIN_VALUE;
      int pixelCount = checkedPixels(geometry);
      int count = in.readInt();
      if (count <= 0 || count > MAX_SOURCE_TILES) {
        throw new IOException("COVERAGE_SOURCE: invalid source tile count " + count);
      }
      Map<String, String> sources = new LinkedHashMap<>();
      for (int i = 0; i < count; i++) {
        String key = in.readUTF();
        String value = in.readUTF();
        if (!key.matches("[0-9]+/[0-9]+/[0-9]+")
          || !(value.equals("absent") || value.matches("[0-9a-f]{64} invalid=[0-9]+"))
          || sources.put(key, value) != null) {
          throw new IOException("COVERAGE_SOURCE: invalid or duplicate source tile " + key);
        }
      }
      byte[] flags = new byte[pixelCount];
      in.readFully(flags);
      if (in.read() != -1) {
        throw new IOException("COVERAGE_FORMAT: trailing coverage bytes");
      }
      MessageDigest digest = MapterhornFiles.sha256Digest();
      try (DigestInputStream data = new DigestInputStream(open(bef), digest)) {
        ElevationRasterCoder coder = new ElevationRasterCoder();
        ElevationRaster actual = coder.decodeHeader(data);
        if (!ElevationCells.headerEquals(geometry, actual)) {
          throw new IOException("COVERAGE_GEOMETRY: header differs for " + bef);
        }
        coder.decodeRasterRows(actual, data, (row, values) -> {
          for (int column = 0; column < values.length; column++) {
            int mask = flags[row * values.length + column];
            if (mask < 1 || mask > 8 || ((mask & OUTSIDE) != 0 && mask != OUTSIDE)) {
              throw new IOException("COVERAGE_FORMAT: invalid source flag " + mask);
            }
            boolean expectedValid = (mask & VALID) != 0;
            if (expectedValid != (values[column] != Short.MIN_VALUE)) {
              throw new IOException("UNEXPECTED_NODATA: source/output validity differs in "
                + bef + " row=" + row + " column=" + column);
            }
          }
        });
        byte[] buffer = new byte[8192];
        while (data.read(buffer) != -1) {
          // Bind padding as well as bytes consumed by the decoder.
        }
      }
      if (!expectedHash.equals(MapterhornFiles.hex(digest.digest()))) {
        throw new IOException("COVERAGE_HASH: BEF bytes changed; reconvert " + bef);
      }
      verifyProvenance(bef, archive, expectedHash, sampling, zoom, tileSize, geometry);
      return new Evidence(archive, expectedHash, sampling, zoom, tileSize, geometry, sources, flags);
    }
  }

  private static void verifyProvenance(Path bef, String archive, String hash, String sampling,
                                       int zoom, int tileSize, ElevationRaster geometry) throws IOException {
    MapterhornOutputMetadata.CompletedMetadata metadata = MapterhornOutputMetadata.readCompleted(bef);
    MapterhornOutputMetadata.Config config = metadata.config;
    if (!archive.equals(config.archiveId) || !hash.equals(metadata.file.sha256)
      || !sampling.equals(config.coverage) || zoom != config.zoom || tileSize != config.tileSize
      || Files.size(bef) != metadata.file.length) {
      throw new IOException("COVERAGE_PROVENANCE: source evidence differs from completed conversion " + bef);
    }
    ElevationRaster expected = new ElevationRaster();
    ElevationCells.configureCellRaster(expected, config.lonStart, config.latStart, config.rowLength);
    if (!ElevationCells.headerEquals(expected, geometry)) {
      throw new IOException("COVERAGE_GEOMETRY: source evidence differs from conversion geometry " + bef);
    }
  }

  private static InputStream open(Path path) throws IOException {
    if (!Files.isRegularFile(path, LinkOption.NOFOLLOW_LINKS)) {
      throw new IOException("COVERAGE_MISSING: expected regular file " + path + "; reconvert the cell");
    }
    return new BufferedInputStream(Channels.newInputStream(Files.newByteChannel(path,
      StandardOpenOption.READ, LinkOption.NOFOLLOW_LINKS)));
  }

  private static String requireSha256(String value, String description) throws IOException {
    if (!value.matches("[0-9a-f]{64}")) {
      throw new IOException("COVERAGE_HASH: invalid " + description + " identity");
    }
    return value;
  }

  private static int checkedPixels(ElevationRaster raster) throws IOException {
    long count = (long) raster.ncols * raster.nrows;
    if (raster.ncols < 2 || raster.nrows != raster.ncols || count > MAX_PIXELS
      || !Double.isFinite(raster.xllcorner) || !Double.isFinite(raster.yllcorner)
      || !Double.isFinite(raster.cellsize) || raster.cellsize <= 0
      || raster.halfcol || raster.noDataValue != Short.MIN_VALUE) {
      throw new IOException("COVERAGE_GEOMETRY: unsupported raster dimensions or geometry");
    }
    return (int) count;
  }
}
