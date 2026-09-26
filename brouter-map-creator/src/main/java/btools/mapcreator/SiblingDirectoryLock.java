package btools.mapcreator;

import java.io.File;
import java.io.IOException;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.channels.OverlappingFileLockException;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.BasicFileAttributes;

/**
 * Exclusive advisory lock on a directory, held through a hidden sibling lock file
 * ({@code .<name><suffix>} next to the directory) so the locked directory itself stays
 * free of foreign files. Acquisition resolves the parent to its real path with symlink
 * checks, takes the file lock, then creates and validates the directory while the lock
 * is already held. One implementation serves both the Mapterhorn output directory and
 * the tile cache root, so locking fixes apply to both.
 */
final class SiblingDirectoryLock implements AutoCloseable {

  /** Test hook invoked between taking the file lock and preparing the directory. */
  @FunctionalInterface
  interface AfterLock {
    void afterLock(Path lockFile) throws IOException;
  }

  private final Path directory;
  private final FileChannel channel;
  private final FileLock lock;
  private boolean closed;

  private SiblingDirectoryLock(Path directory, FileChannel channel, FileLock lock) {
    this.directory = directory;
    this.channel = channel;
    this.lock = lock;
  }

  /**
   * @param what      noun for error messages, e.g. "Mapterhorn output" or "tile cache"
   * @param holder    noun for the competing owner in the already-in-use message
   * @param afterLock optional test hook, may be null
   */
  static SiblingDirectoryLock acquire(File requestedDirectory, String lockSuffix,
                                      String what, String holder, AfterLock afterLock)
      throws IOException {
    if (requestedDirectory == null) {
      throw new IllegalArgumentException(what + " directory is required");
    }
    Path requested = requestedDirectory.toPath().toAbsolutePath().normalize();
    Path requestedParent = requested.getParent();
    Path directoryName = requested.getFileName();
    if (requestedParent == null || directoryName == null) {
      throw new IOException("a filesystem root cannot be used as " + what + ": "
        + requested);
    }
    Files.createDirectories(requestedParent);
    Path realParent = requestedParent.toRealPath();
    BasicFileAttributes parentAttributes = Files.readAttributes(realParent,
      BasicFileAttributes.class, LinkOption.NOFOLLOW_LINKS);
    if (parentAttributes.isSymbolicLink() || !parentAttributes.isDirectory()) {
      throw new IOException(what + " parent is not a real directory: " + realParent);
    }
    Path directory = realParent.resolve(directoryName.toString()).normalize();
    Path lockFile = realParent.resolve("." + directoryName + lockSuffix).normalize();

    BasicFileAttributes lockAttributes = MapterhornFiles.attributesIfPresent(lockFile);
    if (lockAttributes != null) {
      MapterhornFiles.requireRegularFile(lockFile, lockAttributes,
        what + " sibling lock file");
    }
    FileChannel channel = FileChannel.open(lockFile, StandardOpenOption.CREATE,
      StandardOpenOption.WRITE, LinkOption.NOFOLLOW_LINKS);
    try {
      lockAttributes = Files.readAttributes(lockFile, BasicFileAttributes.class,
        LinkOption.NOFOLLOW_LINKS);
      MapterhornFiles.requireRegularFile(lockFile, lockAttributes,
        what + " sibling lock file");
      Object openedKey = lockAttributes.fileKey();
      FileLock lock;
      try {
        lock = channel.tryLock();
      } catch (OverlappingFileLockException e) {
        throw alreadyLocked(directory, what, holder, e);
      }
      if (lock == null) {
        throw alreadyLocked(directory, what, holder, null);
      }
      // The lock binds to the inode we opened; if the pathname was unlinked and
      // recreated in between, another process could lock the NEW inode concurrently.
      // Verify the path still names our inode while we hold the lock. (This closes
      // the open-vs-lock race; like any advisory path lock it cannot survive
      // deliberate external unlinking afterwards.)
      BasicFileAttributes lockedAttributes = MapterhornFiles.attributesIfPresent(lockFile);
      if (lockedAttributes == null
        || !java.util.Objects.equals(openedKey, lockedAttributes.fileKey())) {
        throw new IOException(what + " sibling lock file was replaced while locking: "
          + lockFile);
      }
      if (afterLock != null) {
        afterLock.afterLock(lockFile);
      }
      Path prepared = prepareDirectory(directory, what);
      return new SiblingDirectoryLock(prepared, channel, lock);
    } catch (IOException | RuntimeException | Error e) {
      try {
        channel.close();
      } catch (IOException closeFailure) {
        e.addSuppressed(closeFailure);
      }
      throw e;
    }
  }

  /** The locked directory's real path. */
  Path directoryPath() {
    if (closed) {
      throw new IllegalStateException("directory lock is closed: " + directory);
    }
    return directory;
  }

  @Override
  public synchronized void close() throws IOException {
    if (closed) {
      return;
    }
    closed = true;
    IOException failure = null;
    try {
      lock.release();
    } catch (IOException e) {
      failure = e;
    }
    try {
      channel.close();
    } catch (IOException e) {
      if (failure == null) {
        failure = e;
      } else {
        failure.addSuppressed(e);
      }
    }
    if (failure != null) {
      throw failure;
    }
  }

  private static Path prepareDirectory(Path directory, String what) throws IOException {
    BasicFileAttributes attributes = MapterhornFiles.attributesIfPresent(directory);
    if (attributes == null) {
      try {
        Files.createDirectory(directory);
      } catch (FileAlreadyExistsException e) {
        // Validate a concurrent filesystem change below while holding our lock.
      }
      attributes = MapterhornFiles.attributesIfPresent(directory);
    }
    if (attributes == null) {
      throw new IOException("cannot create " + what + " directory " + directory);
    }
    MapterhornFiles.rejectSymbolicLink(directory, attributes, what);
    if (!attributes.isDirectory()) {
      throw new IOException(what + " path is not a directory: " + directory);
    }
    Path real = directory.toRealPath();
    BasicFileAttributes realAttributes = Files.readAttributes(real,
      BasicFileAttributes.class, LinkOption.NOFOLLOW_LINKS);
    MapterhornFiles.rejectSymbolicLink(real, realAttributes, what);
    if (!realAttributes.isDirectory()) {
      throw new IOException(what + " path is not a directory: " + real);
    }
    return real;
  }

  private static IOException alreadyLocked(Path directory, String what, String holder,
                                           Throwable cause) {
    return new IOException(what + " directory " + directory
      + " is already in use by another process or " + holder + " instance", cause);
  }
}
