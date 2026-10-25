package btools.mapcreator;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Raw railcar-ferry support that the unchanged routing dictionary cannot represent. */
final class RailFerrySupport {
  static final String FILENAME = "rail-ferry-support.bin";
  private static final int MAGIC = 0x52465331;
  private static final int MAX_WAYS = 100_000;
  private static final int MAX_NODES = 1_000_000;
  final Map<Long, Entry> ways = new HashMap<>();
  String sourceSha256;
  String sha256;

  static final class Entry {
    final int nodes;
    final String nodeSha256;

    Entry(int nodes, String nodeSha256) {
      this.nodes = nodes;
      this.nodeSha256 = nodeSha256;
    }

    void verify(int count, MessageDigest digest) throws IOException {
      if (count != nodes || !nodeSha256.equals(MapterhornFiles.hex(digest.digest()))) {
        throw new IOException("FERRY_SUPPORT_STALE: accepted way node sequence differs from source");
      }
    }
  }

  static final class Writer implements AutoCloseable {
    private final DataOutputStream output;
    private final Set<Long> seen = new HashSet<>();

    Writer(Path file, Path source) throws IOException {
      String sourceHash = hashFile(source);
      output = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file,
        StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE)));
      output.writeInt(MAGIC);
      output.writeUTF(sourceHash);
    }

    void accepted(WayData way) throws IOException {
      Map<String, String> tags = way.getTagsOrNull();
      if (tags == null || !"ferry".equals(tags.get("route")) || !"ferry".equals(tags.get("railway"))
        || tags.containsKey("highway") || (tags.containsKey("waterway") && !RoutingElevationGate.WATERWAYS.contains(tags.get("waterway")))) {
        return;
      }
      if (way.wid < 0 || seen.size() == MAX_WAYS || !seen.add(way.wid)
        || way.nodes.size() < 1 || way.nodes.size() > MAX_NODES) {
        throw new IOException("FERRY_SUPPORT_FORMAT: duplicate, invalid or excessive source ways");
      }
      MessageDigest digest = MapterhornFiles.sha256Digest();
      for (int i = 0; i < way.nodes.size(); i++) {
        long node = way.nodes.get(i);
        if (node < 0) throw new IOException("FERRY_SUPPORT_FORMAT: negative source node ID");
        addNode(digest, node);
      }
      output.writeLong(way.wid);
      output.writeInt(way.nodes.size());
      output.write(digest.digest());
    }

    void finish() throws IOException {
      output.writeLong(-1);
      output.writeInt(seen.size());
    }

    @Override
    public void close() throws IOException {
      output.close();
    }
  }

  static RailFerrySupport read(Path file, Path source) throws IOException {
    RailFerrySupport result = new RailFerrySupport();
    MessageDigest digest = MapterhornFiles.sha256Digest();
    try (DataInputStream input = new DataInputStream(new DigestInputStream(open(file), digest))) {
      if (input.readInt() != MAGIC) throw new IOException("FERRY_SUPPORT_FORMAT: unsupported schema");
      result.sourceSha256 = input.readUTF();
      if (!result.sourceSha256.matches("[0-9a-f]{64}") || !result.sourceSha256.equals(hashFile(source))) {
        throw new IOException("FERRY_SUPPORT_STALE: source PBF differs from the cutter source");
      }
      for (long id; (id = input.readLong()) != -1;) {
        if (id < 0 || result.ways.size() == MAX_WAYS || result.ways.containsKey(id)) {
          throw new IOException("FERRY_SUPPORT_FORMAT: duplicate, invalid or excessive source ways");
        }
        int nodes = input.readInt();
        if (nodes < 1 || nodes > MAX_NODES) throw new IOException("FERRY_SUPPORT_FORMAT: invalid node count");
        byte[] hash = new byte[32];
        input.readFully(hash);
        result.ways.put(id, new Entry(nodes, MapterhornFiles.hex(hash)));
      }
      if (input.readInt() != result.ways.size() || input.read() != -1) {
        throw new IOException("FERRY_SUPPORT_FORMAT: invalid footer or trailing data");
      }
    }
    result.sha256 = MapterhornFiles.hex(digest.digest());
    return result;
  }

  static void addNode(MessageDigest digest, long node) {
    for (int shift = 56; shift >= 0; shift -= 8) digest.update((byte) (node >>> shift));
  }

  private static InputStream open(Path path) throws IOException {
    if (!Files.isRegularFile(path, LinkOption.NOFOLLOW_LINKS)) {
      throw new IOException("FERRY_SUPPORT_INPUT: expected a regular file: " + path);
    }
    return new BufferedInputStream(Files.newInputStream(path, LinkOption.NOFOLLOW_LINKS));
  }

  private static String hashFile(Path path) throws IOException {
    MessageDigest digest = MapterhornFiles.sha256Digest();
    try (InputStream input = open(path)) {
      byte[] buffer = new byte[65536];
      for (int size; (size = input.read(buffer)) != -1;) digest.update(buffer, 0, size);
    }
    return MapterhornFiles.hex(digest.digest());
  }
}
