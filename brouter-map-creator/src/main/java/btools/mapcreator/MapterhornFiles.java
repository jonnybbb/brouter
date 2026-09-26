package btools.mapcreator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

/**
 * File-safety and provenance helpers shared by the Mapterhorn converter, tile cache,
 * output metadata and PMTiles reader. Keeping them in one place keeps the symlink
 * defenses, atomic-move semantics and digest formats identical across the feature: a
 * hardening fix applied here reaches every path at once.
 */
final class MapterhornFiles {

  /** Injectable atomic rename, so tests can exercise move failures. */
  @FunctionalInterface
  interface AtomicMoveOperation {
    void move(Path from, Path to) throws IOException;
  }

  private MapterhornFiles() {
  }

  static void atomicMove(Path from, Path to) throws IOException {
    Files.move(from, to, StandardCopyOption.ATOMIC_MOVE,
      StandardCopyOption.REPLACE_EXISTING);
  }

  static MessageDigest sha256Digest() {
    try {
      return MessageDigest.getInstance("SHA-256");
    } catch (NoSuchAlgorithmException e) {
      throw new AssertionError("SHA-256 is required by the Java runtime", e);
    }
  }

  static String sha256Hex(byte[] data) {
    return hex(sha256Digest().digest(data));
  }

  static String hex(byte[] bytes) {
    char[] result = new char[bytes.length * 2];
    char[] digits = "0123456789abcdef".toCharArray();
    for (int i = 0; i < bytes.length; i++) {
      int value = bytes[i] & 0xff;
      result[2 * i] = digits[value >>> 4];
      result[2 * i + 1] = digits[value & 0xf];
    }
    return new String(result);
  }

  /** Attributes without following symlinks, or null when the path does not exist. */
  static BasicFileAttributes attributesIfPresent(Path path) throws IOException {
    try {
      return Files.readAttributes(path, BasicFileAttributes.class,
        LinkOption.NOFOLLOW_LINKS);
    } catch (NoSuchFileException e) {
      return null;
    }
  }

  static void rejectSymbolicLink(Path path, BasicFileAttributes attributes,
                                 String what) throws IOException {
    if (attributes != null && attributes.isSymbolicLink()) {
      throw new IOException("refusing symbolic link in " + what + ": " + path);
    }
  }

  static void requireRegularFile(Path path, BasicFileAttributes attributes,
                                 String description) throws IOException {
    rejectSymbolicLink(path, attributes, description);
    if (!attributes.isRegularFile()) {
      throw new IOException(description + " is not a regular file: " + path);
    }
  }

  static void requireDirectory(Path path, BasicFileAttributes attributes,
                               String description) throws IOException {
    if (!attributes.isDirectory()) {
      throw new IOException(description + " is not a directory: " + path);
    }
  }
}
