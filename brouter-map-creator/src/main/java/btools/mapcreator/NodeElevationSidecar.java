package btools.mapcreator;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.Closeable;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * One elevation per node of one node file, in that file's node order, written by
 * {@link MapterhornNodeElevation} and read by {@link PosUnifier}.
 * <p>
 * Every entry carries its node id as well as its elevation, and the reader refuses an
 * entry whose id is not the node it is asked about. The two files are only ever paired by
 * name, so a sidecar from another cutter run would otherwise put every elevation on the
 * wrong node without any error.
 * <p>
 * Layout: magic, source zoom, entry count, then per node its id (long) and elevation in
 * quarter metres (short, {@link Short#MIN_VALUE} for none).
 */
final class NodeElevationSidecar {

  static final String SUFFIX = ".nel";

  private static final int MAGIC = 0x4e454c31; // "NEL1"

  private NodeElevationSidecar() {
  }

  /** The sidecar belonging to {@code nodeFile} inside {@code dir}. */
  static File forNodeFile(File dir, File nodeFile) {
    return new File(dir, nodeFile.getName() + SUFFIX);
  }

  static void write(File out, int zoom, long[] nids, short[] selev) throws IOException {
    if (nids.length != selev.length) {
      throw new IllegalArgumentException("one elevation per node required");
    }
    File tmp = new File(out.getPath() + ".tmp");
    try (DataOutputStream dos = new DataOutputStream(
        new BufferedOutputStream(new FileOutputStream(tmp)))) {
      dos.writeInt(MAGIC);
      dos.writeInt(zoom);
      dos.writeLong(nids.length);
      for (int i = 0; i < nids.length; i++) {
        dos.writeLong(nids[i]);
        dos.writeShort(selev[i]);
      }
    }
    MapterhornFiles.atomicMove(tmp.toPath(), out.toPath());
  }

  /** Sequential reader, consumed in lockstep with the node file it belongs to. */
  static final class Reader implements Closeable {
    private final File file;
    private final DataInputStream in;
    private final int zoom;
    private final long count;
    private long read;

    Reader(File file) throws IOException {
      this.file = file;
      this.in = new DataInputStream(new BufferedInputStream(new FileInputStream(file)));
      try {
        if (in.readInt() != MAGIC) {
          throw new IOException("not a node elevation sidecar: " + file);
        }
        zoom = in.readInt();
        count = in.readLong();
      } catch (IOException e) {
        in.close();
        throw e;
      }
    }

    int zoom() {
      return zoom;
    }

    /** The elevation of the next node, which must be {@code nid}. */
    short next(long nid) throws IOException {
      if (read == count) {
        throw new IOException(file + " ends after " + count + " nodes, before node " + nid);
      }
      long expected;
      short selev;
      try {
        expected = in.readLong();
        selev = in.readShort();
      } catch (EOFException e) {
        throw new IOException(file + " is truncated at entry " + read, e);
      }
      read++;
      if (expected != nid) {
        throw new IOException(file + " entry " + (read - 1) + " is node " + expected
          + ", the node file has " + nid + ": the sidecar belongs to another cutter run");
      }
      return selev;
    }

    /** Fails unless every entry was consumed, i.e. the node file had no fewer nodes. */
    void finish() throws IOException {
      if (read != count) {
        throw new IOException(file + " holds " + count + " nodes, the node file only " + read);
      }
    }

    @Override
    public void close() throws IOException {
      in.close();
    }
  }
}
