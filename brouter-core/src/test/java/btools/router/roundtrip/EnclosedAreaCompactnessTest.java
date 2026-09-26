package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import btools.router.OsmPathElement;

/**
 * The loop-vs-thread discriminator: NET enclosed area (signed shoelace). A
 * square loop scores near pi/4; an out-and-back thread cancels to ~0 even
 * though its convex hull (the basis of the display compactness) is large.
 */
public class EnclosedAreaCompactnessTest {

  private static OsmPathElement at(double lonM, double latM) {
    // ~meters around 48N: 1e-6 deg lat ~ 0.11m; use microdegree grid directly
    int ilon = 187852000 + (int) Math.round(lonM / 0.0745);
    int ilat = 138000000 + (int) Math.round(latM / 0.1105);
    return OsmPathElement.create(ilon, ilat, (short) 0, null);
  }

  private static int perimeter(List<OsmPathElement> nodes) {
    int d = 0;
    for (int i = 1; i < nodes.size(); i++) {
      d += nodes.get(i - 1).calcDistance(nodes.get(i));
    }
    return d;
  }

  @Test
  public void squareLoopScoresHighThreadScoresZero() {
    List<OsmPathElement> square = new ArrayList<>();
    int s = 5000;
    for (double[] c : new double[][]{{0, 0}, {s, 0}, {s, s}, {0, s}, {0, 0}}) {
      square.add(at(c[0], c[1]));
    }
    double sq = LoopQualityMetrics.enclosedAreaCompactness(square, perimeter(square));
    Assert.assertTrue("square loop ~pi/4, got " + sq, sq > 0.6 && sq < 0.9);

    // Out-and-back on two parallel roads 60m apart: hull is a long rectangle,
    // net enclosed area is a sliver.
    List<OsmPathElement> thread = new ArrayList<>();
    for (int x = 0; x <= 10000; x += 500) thread.add(at(x, 0));
    for (int x = 10000; x >= 0; x -= 500) thread.add(at(x, 60));
    thread.add(at(0, 0));
    double th = LoopQualityMetrics.enclosedAreaCompactness(thread, perimeter(thread));
    Assert.assertTrue("thread ~0, got " + th, th < 0.05);
  }

  @Test
  public void degenerateInputsScoreZero() {
    Assert.assertEquals(0.0, LoopQualityMetrics.enclosedAreaCompactness(null, 1000), 0.0);
    Assert.assertEquals(0.0, LoopQualityMetrics.enclosedAreaCompactness(new ArrayList<>(), 1000), 0.0);
    List<OsmPathElement> two = new ArrayList<>();
    two.add(at(0, 0));
    two.add(at(1000, 0));
    Assert.assertEquals(0.0, LoopQualityMetrics.enclosedAreaCompactness(two, 0), 0.0);
  }
}
