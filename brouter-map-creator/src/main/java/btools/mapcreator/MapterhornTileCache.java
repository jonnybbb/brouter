package btools.mapcreator;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.channels.Channels;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.DirectoryStream;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.HashSet;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;

/**
 * Persistent cache of decompressed Mapterhorn tiles, keyed by their PMTiles byte range.
 * One cache instance owns byte accounting and an exclusive root lock for its lifetime;
 * completed entries are never removed, while writes stop once the configured budget
 * would be exceeded.
 */
public final class MapterhornTileCache implements AutoCloseable {

  private static final String ID_FILE = "archive.id";
  private static final String LOCK_SUFFIX = ".mapterhorn-cache.lock";
  private static final String TILES_DIRECTORY = "tiles";
  private static final String CACHE_SCHEMA = "2";
  private static final AtomicInteger TMP_SEQUENCE = new AtomicInteger();
  private static final Pattern ID_TEMPORARY =
    Pattern.compile("archive\\.id\\.tmp[0-9]+");
  private static final Pattern TILE_TEMPORARY =
    Pattern.compile("[0-9a-f]+-[0-9a-f]+\\.tile\\.tmp[0-9]+");
  private static final Pattern SHARD_DIRECTORY = Pattern.compile("[0-9a-f]{2}");

  private final Path root;
  private final Path tilesDirectory;
  private final long maxBytes;
  private final MapterhornFiles.AtomicMoveOperation atomicMoveOperation;
  private final SiblingDirectoryLock rootLock;
  private final Set<Path> reservations = new HashSet<>();

  private long usedBytes;
  private boolean writeEnabled = true;
  private boolean warned;
  private boolean closed;
  private int activeUses;

  public MapterhornTileCache(File root, String archiveId, long maxBytes) throws IOException {
    this(root, archiveId, maxBytes, MapterhornFiles::atomicMove);
  }

  MapterhornTileCache(File root, String archiveId, long maxBytes,
                      MapterhornFiles.AtomicMoveOperation atomicMoveOperation) throws IOException {
    this(root, archiveId, maxBytes, atomicMoveOperation,
      MapterhornTileCache::ignoreRootLock);
  }

  MapterhornTileCache(File root, String archiveId, long maxBytes,
                      MapterhornFiles.AtomicMoveOperation atomicMoveOperation,
                      SiblingDirectoryLock.AfterLock rootLockOperation) throws IOException {
    if (root == null) {
      throw new IllegalArgumentException("cache directory is required");
    }
    if (archiveId == null || archiveId.isEmpty()) {
      throw new IllegalArgumentException("archive ID is required");
    }
    if (maxBytes <= 0L) {
      throw new IllegalArgumentException("cache byte budget must be positive");
    }
    if (atomicMoveOperation == null) {
      throw new IllegalArgumentException("atomic move operation is required");
    }
    if (rootLockOperation == null) {
      throw new IllegalArgumentException("root lock operation is required");
    }
    SiblingDirectoryLock acquiredLock = SiblingDirectoryLock.acquire(
      root, LOCK_SUFFIX, "tile cache", "cache", rootLockOperation);
    this.root = acquiredLock.directoryPath();
    this.tilesDirectory = this.root.resolve(TILES_DIRECTORY);
    this.maxBytes = maxBytes;
    this.atomicMoveOperation = atomicMoveOperation;
    this.rootLock = acquiredLock;

    try {
      cleanStaleIdentityTemporaryFiles();
      initializeIdentity(archiveId);
      usedBytes = cleanTemporaryFilesAndCountTiles();
      if (usedBytes > maxBytes) {
        disableWrites();
      }
    } catch (IOException | RuntimeException | Error e) {
      try {
        rootLock.close();
      } catch (IOException closeFailure) {
        e.addSuppressed(closeFailure);
      }
      throw e;
    }
  }

  /** @return cached decompressed tile bytes, or null when this location is not cached. */
  public byte[] read(PmTilesArchive.TileLocation location) throws IOException {
    beginUse();
    try {
      Path path = file(location);
      if (!requireDirectoryIfPresent(tilesDirectory)
        || !requireDirectoryIfPresent(path.getParent())) {
        return null;
      }
      BasicFileAttributes attributes = attributesIfPresent(path);
      if (attributes == null) {
        return null;
      }
      requireRegularFile(path, attributes, "tile cache entry");
      if (attributes.size() > PmTilesArchive.MAX_INFLATED_BYTES) {
        // no legitimate write can produce this (decompressTile bounds every entry);
        // drop the corrupt file instead of feeding it to readAllBytes
        invalidate(location);
        return null;
      }
      try (InputStream in = Channels.newInputStream(Files.newByteChannel(
          path, StandardOpenOption.READ, LinkOption.NOFOLLOW_LINKS))) {
        return in.readAllBytes();
      }
    } finally {
      endUse();
    }
  }

  /**
   * Store decompressed bytes if this key is new and the byte budget has room. Equal
   * locations are idempotent, including while another cache worker is writing the key.
   */
  public void write(PmTilesArchive.TileLocation location, byte[] bytes) throws IOException {
    if (bytes == null) {
      throw new IllegalArgumentException("tile bytes are required");
    }
    beginUse();
    try {
      writeEntry(location, bytes);
    } finally {
      endUse();
    }
  }

  /**
   * Remove one entry, e.g. after its bytes failed to decode: a torn or corrupted entry
   * must not permanently fail its cell on every rerun. An absent entry is a no-op.
   */
  public void invalidate(PmTilesArchive.TileLocation location) throws IOException {
    beginUse();
    try {
      Path target = file(location);
      synchronized (this) {
        if (reservations.contains(target)) {
          return; // a concurrent writer owns this key right now
        }
        BasicFileAttributes attributes = MapterhornFiles.attributesIfPresent(target);
        if (attributes == null) {
          return;
        }
        MapterhornFiles.requireRegularFile(target, attributes, "tile cache entry");
        Files.delete(target);
        usedBytes = Math.max(0L, usedBytes - attributes.size());
      }
    } finally {
      endUse();
    }
  }

  public synchronized long usedBytes() {
    return usedBytes;
  }

  public synchronized boolean isWriteEnabled() {
    return writeEnabled;
  }

  @Override
  public void close() throws IOException {
    boolean interrupted = false;
    synchronized (this) {
      if (closed) {
        return;
      }
      closed = true;
      while (activeUses > 0) {
        try {
          wait();
        } catch (InterruptedException e) {
          interrupted = true;
        }
      }
    }
    try {
      rootLock.close();
    } finally {
      if (interrupted) {
        Thread.currentThread().interrupt();
      }
    }
  }

  private void writeEntry(PmTilesArchive.TileLocation location, byte[] bytes)
    throws IOException {
    Path target = file(location);
    ensureDirectory(tilesDirectory, "tile cache tiles directory");
    ensureDirectory(target.getParent(), "tile cache shard directory");
    synchronized (this) {
      BasicFileAttributes attributes = attributesIfPresent(target);
      if (attributes != null) {
        requireRegularFile(target, attributes, "tile cache entry");
        return;
      }
      if (reservations.contains(target)) {
        return;
      }
      if (!writeEnabled) {
        return;
      }
      if (bytes.length > maxBytes - usedBytes) {
        disableWrites();
        return;
      }
      usedBytes += bytes.length;
      reservations.add(target);
    }

    Path temporary = target.resolveSibling(
      target.getFileName() + ".tmp" + TMP_SEQUENCE.incrementAndGet());
    boolean complete = false;
    try {
      writeNewRegularFile(temporary, bytes);
      ensureDirectory(tilesDirectory, "tile cache tiles directory");
      ensureDirectory(target.getParent(), "tile cache shard directory");
      BasicFileAttributes targetAttributes = attributesIfPresent(target);
      if (targetAttributes != null) {
        rejectSymbolicLink(target, targetAttributes);
        throw new IOException("tile cache entry appeared while it was being written: "
          + target);
      }
      atomicMoveOperation.move(temporary, target);
      complete = true;
    } finally {
      try {
        deleteTemporaryIfPresent(temporary);
      } finally {
        synchronized (this) {
          reservations.remove(target);
          if (!complete) {
            usedBytes -= bytes.length;
          }
        }
      }
    }
  }

  private Path file(PmTilesArchive.TileLocation location) {
    if (location == null) {
      throw new IllegalArgumentException("tile location is required");
    }
    String offset = Long.toUnsignedString(location.offset, 16);
    String shard = offset.length() < 2 ? "00" : offset.substring(0, 2);
    Path path = tilesDirectory.resolve(shard)
      .resolve(offset + "-" + Integer.toHexString(location.length) + ".tile")
      .normalize();
    if (!path.startsWith(root)) {
      throw new IllegalArgumentException("tile location resolves outside the cache root");
    }
    return path;
  }

  private void initializeIdentity(String archiveId) throws IOException {
    Path idPath = root.resolve(ID_FILE);
    if (isUninitializedDirectory()) {
      writeIdentity(idPath, archiveId);
      return;
    }
    BasicFileAttributes idAttributes = attributesIfPresent(idPath);
    if (idAttributes == null) {
      throw unidentifiedCache();
    }
    requireRegularFile(idPath, idAttributes, "tile cache archive identity");

    Properties identity = new Properties();
    try (InputStream in = Channels.newInputStream(Files.newByteChannel(
        idPath, StandardOpenOption.READ, LinkOption.NOFOLLOW_LINKS))) {
      identity.load(in);
    } catch (IllegalArgumentException e) {
      throw unidentifiedCache();
    }
    String schema = identity.getProperty("schema");
    String existingArchiveId = identity.getProperty("archiveId");
    if (!CACHE_SCHEMA.equals(schema) || existingArchiveId == null
      || existingArchiveId.isEmpty()) {
      throw unidentifiedCache();
    }
    if (!archiveId.equals(existingArchiveId)) {
      throw new IOException("tile cache " + root + " was filled from a different archive"
        + " (cache id " + existingArchiveId + ", archive id " + archiveId + ");"
        + " use an empty cache directory or delete this one");
    }
  }

  private void cleanStaleIdentityTemporaryFiles() throws IOException {
    try (DirectoryStream<Path> entries = Files.newDirectoryStream(root)) {
      for (Path entry : entries) {
        if (!ID_TEMPORARY.matcher(entry.getFileName().toString()).matches()) {
          continue;
        }
        BasicFileAttributes attributes = attributesIfPresent(entry);
        if (attributes == null) {
          continue;
        }
        requireRegularFile(entry, attributes, "tile cache identity temporary file");
        Files.delete(entry);
      }
    }
  }

  private void writeIdentity(Path idPath, String archiveId) throws IOException {
    Properties identity = new Properties();
    identity.setProperty("schema", CACHE_SCHEMA);
    identity.setProperty("archiveId", archiveId);
    Path temporary = idPath.resolveSibling(
      idPath.getFileName() + ".tmp" + TMP_SEQUENCE.incrementAndGet());
    try {
      BasicFileAttributes temporaryAttributes = attributesIfPresent(temporary);
      if (temporaryAttributes != null) {
        rejectSymbolicLink(temporary, temporaryAttributes);
        throw new IOException("tile cache identity temporary file already exists: "
          + temporary);
      }
      try (FileChannel channel = FileChannel.open(temporary, StandardOpenOption.CREATE_NEW,
          StandardOpenOption.WRITE, LinkOption.NOFOLLOW_LINKS)) {
        OutputStream out = Channels.newOutputStream(channel);
        identity.store(out, "Mapterhorn tile cache");
        out.flush();
        channel.force(false);
      }
      atomicMoveOperation.move(temporary, idPath);
    } finally {
      deleteTemporaryIfPresent(temporary);
    }
  }

  private boolean isUninitializedDirectory() throws IOException {
    boolean uninitialized = true;
    try (DirectoryStream<Path> entries = Files.newDirectoryStream(root)) {
      for (Path entry : entries) {
        BasicFileAttributes attributes = Files.readAttributes(entry,
          BasicFileAttributes.class, LinkOption.NOFOLLOW_LINKS);
        rejectSymbolicLink(entry, attributes);
        uninitialized = false;
      }
      return uninitialized;
    }
  }

  private IOException unidentifiedCache() {
    return new IOException("tile cache " + root + " has no valid schema 2 archive ID;"
      + " use an empty cache directory or delete this one");
  }

  private long cleanTemporaryFilesAndCountTiles() throws IOException {
    CacheStartupVisitor visitor = new CacheStartupVisitor(root);
    Files.walkFileTree(root, visitor);
    return visitor.tileBytes;
  }

  private synchronized void disableWrites() {
    writeEnabled = false;
    if (!warned) {
      warned = true;
      System.err.println("mapterhorn: tile cache budget of " + maxBytes
        + " bytes reached; existing entries remain readable, new writes are disabled");
    }
  }

  private synchronized void beginUse() {
    if (closed) {
      throw new IllegalStateException("tile cache is closed: " + root);
    }
    activeUses++;
  }

  private synchronized void endUse() {
    activeUses--;
    if (activeUses == 0) {
      notifyAll();
    }
  }

  private static boolean requireDirectoryIfPresent(Path path) throws IOException {
    BasicFileAttributes attributes = attributesIfPresent(path);
    if (attributes == null) {
      return false;
    }
    rejectSymbolicLink(path, attributes);
    requireDirectory(path, attributes, "tile cache directory");
    return true;
  }

  private static void ensureDirectory(Path path, String description) throws IOException {
    BasicFileAttributes attributes = attributesIfPresent(path);
    if (attributes == null) {
      try {
        Files.createDirectory(path);
      } catch (FileAlreadyExistsException e) {
        // A concurrent cache worker may have created it. Validate that entry below.
      }
      attributes = attributesIfPresent(path);
    }
    if (attributes == null) {
      throw new IOException("cannot create " + description + ": " + path);
    }
    rejectSymbolicLink(path, attributes);
    requireDirectory(path, attributes, description);
  }

  private static void writeNewRegularFile(Path path, byte[] bytes) throws IOException {
    BasicFileAttributes attributes = attributesIfPresent(path);
    if (attributes != null) {
      rejectSymbolicLink(path, attributes);
      throw new IOException("tile cache temporary file already exists: " + path);
    }
    try (FileChannel channel = FileChannel.open(path, StandardOpenOption.CREATE_NEW,
        StandardOpenOption.WRITE, LinkOption.NOFOLLOW_LINKS)) {
      ByteBuffer buffer = ByteBuffer.wrap(bytes);
      while (buffer.hasRemaining()) {
        channel.write(buffer);
      }
      // flush the data before the atomic rename: a torn entry would otherwise survive
      // a power loss and fail its cell on every rerun
      channel.force(false);
    }
  }

  private static void deleteTemporaryIfPresent(Path path) throws IOException {
    BasicFileAttributes attributes = attributesIfPresent(path);
    if (attributes == null) {
      return;
    }
    rejectSymbolicLink(path, attributes);
    requireRegularFile(path, attributes, "tile cache temporary file");
    Files.delete(path);
  }

  private static BasicFileAttributes attributesIfPresent(Path path) throws IOException {
    return MapterhornFiles.attributesIfPresent(path);
  }

  private static void rejectSymbolicLink(Path path, BasicFileAttributes attributes)
    throws IOException {
    MapterhornFiles.rejectSymbolicLink(path, attributes, "tile cache");
  }

  private static void requireDirectory(Path path, BasicFileAttributes attributes,
                                       String description) throws IOException {
    MapterhornFiles.requireDirectory(path, attributes, description);
  }

  private static void requireRegularFile(Path path, BasicFileAttributes attributes,
                                         String description) throws IOException {
    MapterhornFiles.requireRegularFile(path, attributes, description);
  }

  private static boolean isGeneratedTemporary(Path root, Path path) {
    Path relative = root.relativize(path);
    String name = path.getFileName().toString();
    if (relative.getNameCount() == 1) {
      return ID_TEMPORARY.matcher(name).matches();
    }
    return relative.getNameCount() == 3
      && TILES_DIRECTORY.equals(relative.getName(0).toString())
      && SHARD_DIRECTORY.matcher(relative.getName(1).toString()).matches()
      && TILE_TEMPORARY.matcher(name).matches();
  }

  private static void ignoreRootLock(Path path) {
    // Production lock acquisition needs no callback.
  }

  private static final class CacheStartupVisitor extends SimpleFileVisitor<Path> {
    private final Path root;
    private long tileBytes;

    CacheStartupVisitor(Path root) {
      this.root = root;
    }

    @Override
    public FileVisitResult preVisitDirectory(Path directory,
                                             BasicFileAttributes attributes)
      throws IOException {
      rejectSymbolicLink(directory, attributes);
      requireDirectory(directory, attributes, "tile cache entry");
      return FileVisitResult.CONTINUE;
    }

    @Override
    public FileVisitResult visitFile(Path file, BasicFileAttributes attributes)
      throws IOException {
      rejectSymbolicLink(file, attributes);
      requireRegularFile(file, attributes, "tile cache entry");
      if (isGeneratedTemporary(root, file)) {
        Files.delete(file);
      } else if (file.getFileName().toString().endsWith(".tile")) {
        try {
          tileBytes = Math.addExact(tileBytes, attributes.size());
        } catch (ArithmeticException e) {
          throw new IOException("tile cache size exceeds the supported range", e);
        }
      }
      return FileVisitResult.CONTINUE;
    }
  }

}
