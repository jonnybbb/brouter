package btools.mapcreator;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.nio.file.attribute.FileTime;
import java.util.Objects;

/**
 * {@link PmTilesArchive.ByteSource} over a local archive file. The file is
 * fingerprinted at open (real path, file key, length, modification time) and every read
 * re-checks that fingerprint after the bytes are in, so one reader cannot silently mix
 * two versions of a replaced archive.
 */
public final class FileByteSource implements PmTilesArchive.ByteSource {
  private final Path realPath;
  private final Object fileKey;
  private final long sourceSize;
  private final FileTime modifiedTime;
  private final String versionId;
  private final RandomAccessFile raf;
  private final FileChannel channel;

  public FileByteSource(File file) throws IOException {
    Path path = file.toPath().toRealPath();
    BasicFileAttributes before = Files.readAttributes(path, BasicFileAttributes.class);
    RandomAccessFile opened = new RandomAccessFile(path.toFile(), "r");
    BasicFileAttributes after;
    try {
      after = Files.readAttributes(path, BasicFileAttributes.class);
      if (!sameSnapshot(before, after)) {
        throw new IOException("source file changed while opening: " + path);
      }
    } catch (IOException e) {
      opened.close();
      throw e;
    }

    this.realPath = path;
    this.fileKey = after.fileKey();
    this.sourceSize = after.size();
    this.modifiedTime = after.lastModifiedTime();
    String identity = realPath + "\n" + fileKey + "\n" + sourceSize + "\n"
      + modifiedTime;
    this.versionId = "file-sha256:"
      + MapterhornFiles.sha256Hex(identity.getBytes(StandardCharsets.UTF_8));
    this.raf = opened;
    this.channel = opened.getChannel();
  }

  @Override
  public byte[] read(long offset, int length) throws IOException {
    long end = PmTilesArchive.checkedReadEnd(offset, length);
    if (end > sourceSize) {
      throw new IOException("read past end of source file: offset=" + offset + ", length="
        + length + ", size=" + sourceSize);
    }
    ByteBuffer buf = ByteBuffer.allocate(length);
    int read = 0;
    try {
      while (read < length) {
        int n = channel.read(buf, offset + read);
        if (n <= 0) {
          break;
        }
        read += n;
      }
    } finally {
      // one post-read fingerprint check catches everything a pre-check would have:
      // the data is only returned after the source is known unchanged
      checkSnapshot();
    }
    if (read < length) {
      throw new IOException("short read at " + offset + ": wanted " + length + ", got " + read);
    }
    return buf.array();
  }

  @Override
  public long size() {
    return sourceSize;
  }

  @Override
  public String versionId() {
    return versionId;
  }

  private void checkSnapshot() throws IOException {
    BasicFileAttributes current;
    try {
      current = Files.readAttributes(realPath, BasicFileAttributes.class);
    } catch (IOException e) {
      throw new IOException("source file changed or disappeared: " + realPath, e);
    }
    if (!Objects.equals(fileKey, current.fileKey()) || sourceSize != current.size()
        || !modifiedTime.equals(current.lastModifiedTime())) {
      throw new IOException("source file changed: " + realPath);
    }
  }

  private static boolean sameSnapshot(BasicFileAttributes one, BasicFileAttributes two) {
    return Objects.equals(one.fileKey(), two.fileKey()) && one.size() == two.size()
      && one.lastModifiedTime().equals(two.lastModifiedTime());
  }

  @Override
  public void close() throws IOException {
    channel.close();
    raf.close();
  }
}
