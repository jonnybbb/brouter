package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class NodeElevationSidecarTest {

  @Rule
  public TemporaryFolder tmp = new TemporaryFolder();

  @Test
  public void readsBackTheElevationsInNodeOrder() throws Exception {
    File f = write(new long[]{7, 3, 11}, new short[]{4000, Short.MIN_VALUE, -12});
    try (NodeElevationSidecar.Reader r = new NodeElevationSidecar.Reader(f)) {
      assertEquals(15, r.zoom());
      assertEquals(4000, r.next(7));
      assertEquals(Short.MIN_VALUE, r.next(3));
      assertEquals(-12, r.next(11));
      r.finish();
    }
  }

  @Test
  public void refusesAnEntryForAnotherNode() throws Exception {
    File f = write(new long[]{7, 3}, new short[]{1, 2});
    try (NodeElevationSidecar.Reader r = new NodeElevationSidecar.Reader(f)) {
      r.next(7);
      IOException e = assertThrows(IOException.class, () -> r.next(4));
      assertTrue(e.getMessage(), e.getMessage().contains("another cutter run"));
    }
  }

  @Test
  public void refusesANodeFileWithMoreOrFewerNodes() throws Exception {
    File f = write(new long[]{7}, new short[]{1});
    try (NodeElevationSidecar.Reader r = new NodeElevationSidecar.Reader(f)) {
      r.next(7);
      assertThrows(IOException.class, () -> r.next(8));
    }
    try (NodeElevationSidecar.Reader r = new NodeElevationSidecar.Reader(f)) {
      assertThrows(IOException.class, r::finish);
    }
  }

  private File write(long[] nids, short[] selev) throws IOException {
    File f = new File(tmp.getRoot(), "E5_N45.n5d" + NodeElevationSidecar.SUFFIX);
    NodeElevationSidecar.write(f, 15, nids, selev);
    return f;
  }
}
