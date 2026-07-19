package btools.mapcreator;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.zip.GZIPInputStream;

/**
 * Reader for PMTiles v3 archives, written against the public specification
 * (https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md).
 * <p>
 * Bytes are pulled through a {@link ByteSource}, so one reader serves both a local archive
 * file and a remote archive addressed with HTTP range requests. The 700+ GB Mapterhorn
 * planet archive is only usable in the latter form.
 *
 * @see <a href="https://mapterhorn.com">Mapterhorn</a>
 */
public final class PmTilesArchive implements Closeable {

  public static final int HEADER_LEN = 127;

  public static final int COMPRESSION_NONE = 1;
  public static final int COMPRESSION_GZIP = 2;

  public static final int TILETYPE_PNG = 2;
  public static final int TILETYPE_WEBP = 4;

  static final long INITIAL_DIRECTORY_END = 16L * 1024L;
  static final int MAX_COMPRESSED_DIRECTORY_BYTES = 16 * 1024 * 1024;
  static final int MAX_INFLATED_BYTES = 64 * 1024 * 1024;
  static final int MAX_DIRECTORY_ENTRIES = 250_000;
  static final int MAX_CACHED_LEAF_ENTRIES = 250_000;

  private static final int MAX_DIR_DEPTH = 4;
  private static final int MAX_TILE_ZOOM = 31;

  /**
   * Random access to the raw archive bytes.
   */
  public interface ByteSource extends Closeable {
    byte[] read(long offset, int length) throws IOException;

    /** The immutable snapshot's total byte length. */
    long size() throws IOException;

    /** A stable identity for the immutable snapshot. */
    String versionId() throws IOException;
  }

  private final ByteSource source;

  private long rootDirOffset;
  private long rootDirLength;
  private long metadataOffset;
  private long metadataLength;
  private long leafDirsOffset;
  private long leafDirsLength;
  private long tileDataOffset;
  private long tileDataLength;
  private String headerId;
  private int internalCompression;
  private int tileCompression;
  private int tileType;
  private int minZoom;
  private int maxZoom;
  private double minLon;
  private double minLat;
  private double maxLon;
  private double maxLat;

  private List<DirEntry> rootDir;
  private boolean closed;

  private final Map<Long, List<DirEntry>> leafCache =
    new LinkedHashMap<>(16, 0.75f, true);
  private int cachedLeafEntries;

  private PmTilesArchive(ByteSource source) {
    this.source = source;
  }

  /**
   * Open an archive given either a local file path or an http(s) URL.
   */
  public static PmTilesArchive open(String location) throws IOException {
    ByteSource src = location.startsWith("http://") || location.startsWith("https://")
      ? new HttpRangeByteSource(location)
      : new FileByteSource(new File(location));
    try {
      return open(src);
    } catch (IOException | RuntimeException e) {
      src.close();
      throw e;
    }
  }

  /**
   * Open an archive over a caller-supplied source. The caller keeps ownership of the
   * source when this throws.
   */
  public static PmTilesArchive open(ByteSource source) throws IOException {
    PmTilesArchive archive = new PmTilesArchive(source);
    byte[] header = archive.readHeader();
    byte[] rawRootDir = source.read(archive.rootDirOffset,
      checkedDirectoryLength(archive.rootDirLength, "root directory"));
    MessageDigest identity = MapterhornFiles.sha256Digest();
    identity.update(source.versionId().getBytes(StandardCharsets.UTF_8));
    identity.update(header);
    identity.update(rawRootDir);
    archive.headerId = MapterhornFiles.hex(identity.digest());
    archive.rootDir = archive.parseDirectory(rawRootDir, "root directory");
    return archive;
  }

  public int minZoom() {
    return minZoom;
  }

  public int maxZoom() {
    return maxZoom;
  }

  public int tileType() {
    return tileType;
  }

  public double minLon() {
    return minLon;
  }

  public double minLat() {
    return minLat;
  }

  public double maxLon() {
    return maxLon;
  }

  public double maxLat() {
    return maxLat;
  }

  /**
   * Byte position of one tile's body within the archive. Distinct tiles may share a
   * location: PMTiles de-duplicates identical tiles into runs.
   */
  public static final class TileLocation {
    public final long offset;
    public final int length;

    TileLocation(long offset, int length) {
      this.offset = offset;
      this.length = length;
    }

    /** Adjacent or identical locations can be served by one contiguous read. */
    public boolean isContiguousWith(TileLocation prev) {
      return offset == prev.offset + prev.length
        || (offset == prev.offset && length == prev.length);
    }
  }

  /**
   * @return the raw (decompressed) bytes of the requested tile, or null when the archive
   * does not contain it. Absent tiles are normal: Mapterhorn omits open water.
   * <p>
   * Safe to call from several threads. Only the directory walk is serialised; the tile
   * body, which is where the round-trip latency lives, is read outside the lock.
   */
  public byte[] getTile(int z, int x, int y) throws IOException {
    TileLocation ref = locateTile(z, x, y);
    if (ref == null) {
      return null;
    }
    return decompressTile(source.read(ref.offset, ref.length));
  }

  /**
   * Resolve a tile to its byte location without reading the body, or null when absent.
   * Callers batching many tiles can group contiguous locations (the archive is usually
   * clustered: Hilbert order == byte order) and fetch each group with one
   * {@link #readRange} call, then slice with {@link TileLocation#offset}.
   */
  public synchronized TileLocation locateTile(int z, int x, int y) throws IOException {
    if (closed) {
      // fail loudly: a null here means "tile absent", and absent tiles quietly become
      // no-data terrain downstream
      throw new IOException("PMTiles archive is closed");
    }
    long[] ref = findEntry(zxyToTileId(z, x, y), rootDir, 0);
    return ref == null ? null : new TileLocation(ref[0], checkedInt(ref[1], "tile length"));
  }

  /**
   * Read raw archive bytes, e.g. one contiguous run of tile bodies. Thread-safe.
   */
  public byte[] readRange(long offset, int length) throws IOException {
    return source.read(offset, length);
  }

  /**
   * Undo the archive's tile compression on one tile body sliced out of a range read.
   */
  public byte[] decompressTile(byte[] raw) throws IOException {
    if (raw.length > MAX_INFLATED_BYTES) {
      throw new IOException("PMTiles stored tile body exceeds " + MAX_INFLATED_BYTES
        + " bytes: " + raw.length);
    }
    return tileCompression == COMPRESSION_GZIP
      ? gunzip(raw, MAX_INFLATED_BYTES, "PMTiles tile body") : raw;
  }

  /**
   * A fingerprint of this archive build, for binding external tile caches to their
   * source: the SHA-256 of the source version, header, and raw root directory.
   */
  public String archiveId() {
    return headerId;
  }

  @Override
  public void close() throws IOException {
    synchronized (this) {
      closed = true;
      leafCache.clear();
      cachedLeafEntries = 0;
      rootDir = null;
    }
    source.close();
  }

  // ---------------------------------------------------------------- header

  private byte[] readHeader() throws IOException {
    byte[] buf = source.read(0L, HEADER_LEN);
    ByteBuffer bb = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN);

    byte[] magic = new byte[7];
    bb.get(magic);
    if (!Arrays.equals(magic, new byte[]{'P', 'M', 'T', 'i', 'l', 'e', 's'})) {
      throw new IOException("not a PMTiles archive (bad magic)");
    }
    int version = bb.get() & 0xff;
    if (version != 3) {
      throw new IOException("only PMTiles v3 is supported, found v" + version);
    }

    rootDirOffset = bb.getLong();
    rootDirLength = bb.getLong();
    metadataOffset = bb.getLong();
    metadataLength = bb.getLong();
    leafDirsOffset = bb.getLong();
    leafDirsLength = bb.getLong();
    tileDataOffset = bb.getLong();
    tileDataLength = bb.getLong();
    bb.getLong(); // num addressed tiles
    bb.getLong(); // num tile entries
    bb.getLong(); // num tile contents

    bb.get(); // clustered
    internalCompression = bb.get() & 0xff;
    tileCompression = bb.get() & 0xff;
    tileType = bb.get() & 0xff;
    minZoom = bb.get() & 0xff;
    maxZoom = bb.get() & 0xff;
    minLon = bb.getInt() / 1e7;
    minLat = bb.getInt() / 1e7;
    maxLon = bb.getInt() / 1e7;
    maxLat = bb.getInt() / 1e7;

    validateHeaderSections();
    if (internalCompression != COMPRESSION_NONE && internalCompression != COMPRESSION_GZIP) {
      throw new IOException("unsupported PMTiles directory compression: " + internalCompression);
    }
    if (tileCompression != COMPRESSION_NONE && tileCompression != COMPRESSION_GZIP) {
      throw new IOException("unsupported PMTiles tile compression: " + tileCompression);
    }
    if (minZoom > MAX_TILE_ZOOM || maxZoom > MAX_TILE_ZOOM || minZoom > maxZoom) {
      throw new IOException("invalid PMTiles zoom range: " + minZoom + ".." + maxZoom);
    }
    if (minLon < -180.0 || maxLon > 180.0 || minLon > maxLon
      || minLat < -90.0 || maxLat > 90.0 || minLat > maxLat) {
      throw new IOException("invalid PMTiles geographic bounds: " + minLon + "," + minLat
        + " to " + maxLon + "," + maxLat);
    }
    return buf;
  }

  private void validateHeaderSections() throws IOException {
    long rootEnd = checkedSectionEnd(rootDirOffset, rootDirLength, "root directory");
    long metadataEnd = checkedSectionEnd(metadataOffset, metadataLength, "metadata");
    long leafEnd = checkedSectionEnd(leafDirsOffset, leafDirsLength, "leaf data");
    long tileEnd = checkedSectionEnd(tileDataOffset, tileDataLength, "tile data");

    if (rootDirOffset < HEADER_LEN) {
      throw new IOException("invalid PMTiles root directory offset: " + rootDirOffset);
    }
    if (rootEnd > INITIAL_DIRECTORY_END) {
      throw new IOException("implausible PMTiles root directory exceeds initial 16 KiB: "
        + rootEnd);
    }

    long sourceSize = source.size();
    requireSectionInsideSource(rootEnd, sourceSize, "root directory");
    requireSectionInsideSource(metadataEnd, sourceSize, "metadata");
    requireSectionInsideSource(leafEnd, sourceSize, "leaf data");
    requireSectionInsideSource(tileEnd, sourceSize, "tile data");

    // The spec fixes only the header position and the root directory's 16 KiB window;
    // every other section may be located anywhere. They must not alias each other or
    // the header, though: overlapping sections would let one region parse as two things.
    String[] names = {"root directory", "metadata", "leaf data", "tile data"};
    long[] offsets = {rootDirOffset, metadataOffset, leafDirsOffset, tileDataOffset};
    long[] lengths = {rootDirLength, metadataLength, leafDirsLength, tileDataLength};
    long[] ends = {rootEnd, metadataEnd, leafEnd, tileEnd};
    for (int i = 0; i < names.length; i++) {
      if (lengths[i] == 0L) {
        continue;
      }
      if (offsets[i] < HEADER_LEN) {
        throw new IOException("PMTiles " + names[i] + " section overlaps the header: "
          + offsets[i]);
      }
      for (int j = i + 1; j < names.length; j++) {
        if (lengths[j] == 0L) {
          continue;
        }
        if (offsets[i] < ends[j] && offsets[j] < ends[i]) {
          throw new IOException("PMTiles sections overlap: " + names[i] + " and "
            + names[j]);
        }
      }
    }
  }

  private static long checkedSectionEnd(long offset, long length, String label)
      throws IOException {
    if (offset < 0L || length < 0L) {
      throw new IOException("invalid PMTiles " + label + " section: offset=" + offset
        + ", length=" + length);
    }
    return checkedAdd(offset, length, label + " section end");
  }

  private static void requireSectionInsideSource(long end, long sourceSize, String label)
      throws IOException {
    if (end > sourceSize) {
      throw new IOException("PMTiles " + label + " section exceeds source length "
        + sourceSize + ": " + end);
    }
  }

  // ------------------------------------------------------------- directories

  private List<DirEntry> parseDirectory(byte[] raw, String label) throws IOException {
    if (internalCompression == COMPRESSION_GZIP) {
      raw = gunzip(raw, MAX_INFLATED_BYTES, label);
    }
    return deserializeDirectory(raw);
  }

  private List<DirEntry> readDirectory(long offset, long length, String label) throws IOException {
    int readLength = checkedDirectoryLength(length, label);
    return parseDirectory(source.read(offset, readLength), label);
  }

  private List<DirEntry> readLeafDirectory(long offset, long length) throws IOException {
    requireSubrange(offset, length, leafDirsLength, "leaf data");
    List<DirEntry> cached = leafCache.get(offset);
    if (cached != null) {
      return cached;
    }
    long absoluteOffset = checkedAdd(leafDirsOffset, offset, "leaf directory base offset");
    List<DirEntry> entries = readDirectory(absoluteOffset, length,
      "compressed leaf directory");
    leafCache.put(offset, entries);
    try {
      cachedLeafEntries = Math.addExact(cachedLeafEntries, entries.size());
    } catch (ArithmeticException e) {
      throw new IOException("PMTiles leaf cache entry count overflow", e);
    }
    Iterator<Map.Entry<Long, List<DirEntry>>> iterator = leafCache.entrySet().iterator();
    while (cachedLeafEntries > MAX_CACHED_LEAF_ENTRIES && iterator.hasNext()) {
      Map.Entry<Long, List<DirEntry>> eldest = iterator.next();
      cachedLeafEntries -= eldest.getValue().size();
      iterator.remove();
    }
    return entries;
  }

  static List<DirEntry> deserializeDirectory(byte[] data) throws IOException {
    int[] pos = {0};
    int numEntries = checkedInt(readVarint(data, pos), "directory entry count");
    if (numEntries <= 0) {
      return Collections.emptyList();
    }
    if (numEntries > MAX_DIRECTORY_ENTRIES
      || 4L * numEntries > data.length - pos[0]) {
      throw new IOException("implausible PMTiles directory entry count: " + numEntries);
    }

    long[] tileIds = new long[numEntries];
    long last = 0L;
    for (int i = 0; i < numEntries; i++) {
      long delta = readVarint(data, pos);
      if (i > 0 && delta <= 0L) {
        throw new IOException("invalid PMTiles directory tile ID delta at entry " + i
          + ": " + delta);
      }
      last = checkedAdd(last, delta, "directory tile ID");
      tileIds[i] = last;
    }
    long[] runLengths = new long[numEntries];
    for (int i = 0; i < numEntries; i++) {
      runLengths[i] = readVarint(data, pos);
    }
    long[] lengths = new long[numEntries];
    for (int i = 0; i < numEntries; i++) {
      lengths[i] = readVarint(data, pos);
      if (lengths[i] <= 0L) {
        throw new IOException("invalid PMTiles directory entry length at entry " + i
          + ": " + lengths[i]);
      }
    }
    // an offset of zero means "directly after the previous entry"
    long[] offsets = new long[numEntries];
    for (int i = 0; i < numEntries; i++) {
      long v = readVarint(data, pos);
      if (v == 0L) {
        if (i == 0) {
          throw new IOException("invalid PMTiles directory: leading zero offset");
        }
        offsets[i] = checkedAdd(offsets[i - 1], lengths[i - 1],
          "directory offset");
      } else {
        offsets[i] = v - 1L;
      }
    }

    List<DirEntry> entries = new ArrayList<>(numEntries);
    for (int i = 0; i < numEntries; i++) {
      entries.add(new DirEntry(tileIds[i], runLengths[i], offsets[i], lengths[i]));
    }
    return entries;
  }

  private long[] findEntry(long tileId, List<DirEntry> dir, int depth) throws IOException {
    if (depth > MAX_DIR_DEPTH) {
      // an absent tile is normal, but a directory tree this deep is not: fail loudly
      // instead of quietly reporting terrain as missing
      throw new IOException("PMTiles directory nesting exceeds depth " + MAX_DIR_DEPTH
        + " (corrupt or cyclic archive)");
    }
    if (dir == null || dir.isEmpty()) {
      return null;
    }
    // last entry with entry.tileId <= tileId
    int lo = 0;
    int hi = dir.size() - 1;
    int found = -1;
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      if (dir.get(mid).tileId <= tileId) {
        found = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    if (found < 0) {
      return null;
    }
    DirEntry e = dir.get(found);
    if (e.runLength == 0L) {
      return findEntry(tileId, readLeafDirectory(e.offset, e.length), depth + 1);
    }
    long runEnd = checkedAdd(e.tileId, e.runLength, "tile run");
    if (tileId < runEnd) {
      if (e.length > MAX_INFLATED_BYTES) {
        throw new IOException("PMTiles stored tile body exceeds " + MAX_INFLATED_BYTES
          + " bytes: " + e.length);
      }
      requireSubrange(e.offset, e.length, tileDataLength, "tile data");
      long absoluteOffset = checkedAdd(tileDataOffset, e.offset, "tile data base offset");
      return new long[]{absoluteOffset, e.length};
    }
    return null;
  }

  // ------------------------------------------------------- hilbert tile ids

  /**
   * Number of tiles on all zoom levels below z, i.e. the tile id of (z, 0, 0).
   * Closed form of sum(4^i) for i in [0, z).
   */
  public static long hilbertBase(int z) {
    validateZoom(z);
    return hilbertBaseUnchecked(z);
  }

  private static long hilbertBaseUnchecked(int z) {
    return ((1L << (2 * z)) - 1L) / 3L;
  }

  public static long zxyToTileId(int z, int x, int y) {
    validateTileCoordinates(z, x, y);
    return hilbertBaseUnchecked(z) + xyToHilbertDistanceUnchecked(z, x, y);
  }

  /**
   * Standard Hilbert xy-to-distance, matching the PMTiles reference implementation.
   */
  public static long xyToHilbertDistance(int order, long x, long y) {
    validateTileCoordinates(order, x, y);
    return xyToHilbertDistanceUnchecked(order, x, y);
  }

  private static long xyToHilbertDistanceUnchecked(int order, long x, long y) {
    if (order == 0) {
      return 0L;
    }
    long d = 0L;
    for (long s = 1L << (order - 1); s > 0L; s >>= 1) {
      long rx = (x & s) > 0L ? 1L : 0L;
      long ry = (y & s) > 0L ? 1L : 0L;
      d += s * s * ((3L * rx) ^ ry);
      if (ry == 0L) {
        if (rx == 1L) {
          x = s - 1L - x;
          y = s - 1L - y;
        }
        long t = x;
        x = y;
        y = t;
      }
    }
    return d;
  }

  private static void validateZoom(int z) {
    if (z < 0 || z > MAX_TILE_ZOOM) {
      throw new IllegalArgumentException("PMTiles zoom must be in 0.." + MAX_TILE_ZOOM
        + ": " + z);
    }
  }

  private static void validateTileCoordinates(int z, long x, long y) {
    validateZoom(z);
    long tilesPerAxis = 1L << z;
    if (x < 0L || y < 0L || x >= tilesPerAxis || y >= tilesPerAxis) {
      throw new IllegalArgumentException("PMTiles tile coordinates out of range at zoom "
        + z + ": x=" + x + ", y=" + y + " (expected 0.." + (tilesPerAxis - 1L)
        + ")");
    }
  }

  // -------------------------------------------------------------- utilities

  /**
   * Narrow a length/count from the archive with a range check. An unchecked cast could
   * truncate a corrupt 64-bit value to a small positive int and silently parse garbage.
   */
  static int checkedInt(long v, String what) throws IOException {
    if (v < 0L || v > Integer.MAX_VALUE) {
      throw new IOException("implausible PMTiles " + what + ": " + v);
    }
    return (int) v;
  }

  private static int checkedDirectoryLength(long length, String label) throws IOException {
    if (length <= 0L || length > MAX_COMPRESSED_DIRECTORY_BYTES) {
      throw new IOException("implausible PMTiles " + label + " length: " + length
        + " (maximum " + MAX_COMPRESSED_DIRECTORY_BYTES + ")");
    }
    return (int) length;
  }

  private static long checkedAdd(long left, long right, String what) throws IOException {
    if (left < 0L || right < 0L) {
      throw new IOException("invalid PMTiles " + what + ": " + left + " + " + right);
    }
    try {
      return Math.addExact(left, right);
    } catch (ArithmeticException e) {
      throw new IOException("PMTiles " + what + " overflow: " + left + " + " + right, e);
    }
  }

  private static void requireSubrange(long offset, long length, long sectionLength,
                                      String label) throws IOException {
    long end = checkedAdd(offset, length, label + " range");
    if (end > sectionLength) {
      throw new IOException("PMTiles " + label + " range exceeds section length "
        + sectionLength + ": offset=" + offset + ", length=" + length);
    }
  }

  static long checkedReadEnd(long offset, int length) throws IOException {
    if (offset < 0L || length <= 0) {
      throw new IOException("invalid byte range: offset=" + offset + ", length=" + length);
    }
    try {
      return Math.addExact(offset, length);
    } catch (ArithmeticException e) {
      throw new IOException("invalid byte range overflow: offset=" + offset + ", length=" + length,
        e);
    }
  }

  static String sha256Hex(byte[] data) {
    return MapterhornFiles.sha256Hex(data);
  }

  static long readVarint(byte[] data, int[] pos) throws IOException {
    long result = 0L;
    int shift = 0;
    while (true) {
      if (pos[0] >= data.length) {
        throw new IOException("truncated varint in PMTiles directory");
      }
      int b = data[pos[0]++] & 0xff;
      if (shift == 63 && (b & 0x7f) != 0) {
        throw new IOException("varint exceeds signed 64-bit range in PMTiles directory");
      }
      result |= (long) (b & 0x7f) << shift;
      if ((b & 0x80) == 0) {
        return result;
      }
      shift += 7;
      if (shift > 63) {
        throw new IOException("varint too long in PMTiles directory");
      }
    }
  }

  static byte[] gunzip(byte[] data, int maxBytes, String label) throws IOException {
    if (maxBytes < 0) {
      throw new IOException("invalid gzip output limit for " + label + ": " + maxBytes);
    }
    try (GZIPInputStream gis = new GZIPInputStream(new ByteArrayInputStream(data));
         ByteArrayOutputStream bos = new ByteArrayOutputStream(Math.min(maxBytes, 8192))) {
      byte[] buf = new byte[8192];
      long total = 0L;
      int n;
      while ((n = gis.read(buf)) >= 0) {
        try {
          total = Math.addExact(total, n);
        } catch (ArithmeticException e) {
          throw new IOException(label + " gzip output length overflow", e);
        }
        if (total > maxBytes) {
          throw new IOException(label + " gzip output exceeds " + maxBytes + " bytes");
        }
        bos.write(buf, 0, n);
      }
      return bos.toByteArray();
    }
  }

  static final class DirEntry {
    final long tileId;
    final long runLength;
    final long offset;
    final long length;

    DirEntry(long tileId, long runLength, long offset, long length) {
      this.tileId = tileId;
      this.runLength = runLength;
      this.offset = offset;
      this.length = length;
    }
  }

}
