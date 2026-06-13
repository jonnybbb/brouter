package btools.router;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

/**
 * Real-geometry regression for the parallel-corridor reuse gate.
 *
 * <p>Fixture {@code basel_corridor_loop.txt} is the original Kleinbasel→NE 50km
 * gravel loop the GREEDY planner produced before this change. Edge-identity
 * reuse certified it a clean {@code STRICT_LOOP} (claiming only 226m of shared
 * stem), but it actually rides ~1km back along a parallel return corridor on a
 * different way. This is the bug that motivated {@link CorridorOverlapIndex}.
 *
 * <p>The test pins the fix on real geometry (synthetic fixtures in
 * {@link CorridorOverlapIndexTest}/{@link ReuseClassifierTest} pin the mechanics):
 * the corridor is now detected and the verdict is no longer a clean loop.
 */
public class BaselCorridorRegressionTest {

  private static OsmTrack loadFixture() throws Exception {
    OsmTrack t = new OsmTrack();
    t.nodes = new ArrayList<>();
    try (InputStream in = BaselCorridorRegressionTest.class
           .getResourceAsStream("/btools/router/basel_corridor_loop.txt");
         BufferedReader br = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (line.isEmpty() || line.startsWith("#")) continue;
        int comma = line.indexOf(',');
        double lat = Double.parseDouble(line.substring(0, comma));
        double lon = Double.parseDouble(line.substring(comma + 1));
        t.nodes.add(OsmPathElement.create(
          (int) Math.round((lon + 180) * 1e6),
          (int) Math.round((lat + 90) * 1e6), (short) 0, null));
      }
    }
    int d = 0;
    for (int i = 1; i < t.nodes.size(); i++) d += t.nodes.get(i - 1).calcDistance(t.nodes.get(i));
    t.distance = d;
    return t;
  }

  @Test
  public void corridorDetectedOnRealBaselLoop() throws Exception {
    OsmTrack t = loadFixture();
    boolean[] ov = CorridorOverlapIndex.computeEdgeOverlap(t);
    double overlapM = 0;
    for (int i = 0; i < ov.length; i++) {
      if (ov[i]) overlapM += t.nodes.get(i).calcDistance(t.nodes.get(i + 1));
    }
    // The parallel return corridor is ~1km (edge identity saw only 226m).
    assertTrue("corridor should be detected (~1km), got " + (int) overlapM + "m",
      overlapM > 600);
  }

  @Test
  public void verdictNoLongerCleanStrictLoop() throws Exception {
    OsmTrack t = loadFixture();
    RoundTripQualityResult r = ReuseClassifier.classify(t, t.distance, false);
    // Was STRICT_LOOP (falsely); the corridor now downgrades it.
    assertNotEquals("parallel-corridor loop must not certify as a clean STRICT_LOOP: " + r,
      RouteShape.STRICT_LOOP, r.getShape());
  }
}
