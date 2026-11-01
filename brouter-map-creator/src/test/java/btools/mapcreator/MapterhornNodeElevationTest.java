package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.Test;

public class MapterhornNodeElevationTest {

  private static final int Z = 15;
  private static final int SIZE = 16;

  // the z15 tile holding Alpe d'Huez, and the z6 archive tile above it
  private static final int TX = tileX(6.07);
  private static final int TY = tileY(45.09);
  private static final String ARCHIVE = "https://example.test/6-" + (TX >> 9) + "-" + (TY >> 9) + ".pmtiles";

  /** A plane in the tile's own pixel coordinates, which bilinear sampling reproduces exactly. */
  private static double plane(double px, double py) {
    return 1000.0 + 0.5 * px + 0.25 * py;
  }

  @Test
  public void samplesBilinearlyBetweenPixelCentres() throws Exception {
    try (MapterhornNodeElevation sampler = sampler(archive(15, 15))) {
      MapterhornNodeElevation.Stats stats = new MapterhornNodeElevation.Stats();
      short[] selev = sample(sampler, stats, node(TX + 0.4, TY + 0.7));

      double expected = plane(0.4 * SIZE - 0.5, 0.7 * SIZE - 0.5);
      assertEquals(expected * 4, selev[0], 1.0);
      assertEquals(1, stats.sampled);
      assertEquals(1, stats.tiles.get());
    }
  }

  @Test
  public void clampsToTheEdgePixelsInsteadOfReadingTheNeighbourTile() throws Exception {
    try (MapterhornNodeElevation sampler = sampler(archive(15, 15))) {
      short[] selev = sample(sampler, new MapterhornNodeElevation.Stats(), node(TX + 0.01, TY + 0.99));

      assertEquals(plane(0, SIZE - 1) * 4, selev[0], 1.0);
    }
  }

  @Test
  public void leavesNodesTheArchivesDoNotCoverWithoutElevation() throws Exception {
    try (MapterhornNodeElevation sampler = sampler(archive(15, 15))) {
      MapterhornNodeElevation.Stats stats = new MapterhornNodeElevation.Stats();
      // a sibling tile in the same archive that the archive does not hold, then a tile
      // under another z6 archive that is not listed
      short[] selev = sample(sampler, stats, node(TX + 1.5, TY + 0.5), node(TX + 600.5, TY + 0.5));

      assertEquals(Short.MIN_VALUE, selev[0]);
      assertEquals(Short.MIN_VALUE, selev[1]);
      assertEquals(1, stats.absentTile);
      assertEquals(1, stats.uncovered);
      assertEquals(0, stats.sampled);
    }
  }

  @Test
  public void skipsAnArchiveThatStopsBelowTheSamplingZoom() throws Exception {
    try (MapterhornNodeElevation sampler = sampler(archive(13, 14))) {
      MapterhornNodeElevation.Stats stats = new MapterhornNodeElevation.Stats();
      short[] selev = sample(sampler, stats, node(TX + 0.5, TY + 0.5));

      assertEquals(Short.MIN_VALUE, selev[0]);
      assertEquals(1, stats.uncovered);
    }
  }

  @Test
  public void treatsNoDataPixelsAsNoElevation() throws Exception {
    byte[] water = TerrainTiles.constantPng(SIZE, -5000.0);
    PmTilesArchive pm = PmTilesArchive.open(new PmTilesTestArchive().zoomRange(13, 15)
      .put(Z, TX, TY, water).asByteSource());
    try (MapterhornNodeElevation sampler = sampler(pm)) {
      MapterhornNodeElevation.Stats stats = new MapterhornNodeElevation.Stats();
      short[] selev = sample(sampler, stats, node(TX + 0.5, TY + 0.5));

      assertEquals(Short.MIN_VALUE, selev[0]);
      assertEquals(1, stats.noData);
    }
  }

  @Test
  public void fetchesAdjacentTilesInOneRequestAndEachTileOnce() throws Exception {
    PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(13, 15);
    for (int dx = 0; dx < 4; dx++) {
      builder.put(Z, TX + dx, TY, planeTile());
    }
    try (MapterhornNodeElevation sampler = sampler(PmTilesArchive.open(builder.asByteSource()))) {
      MapterhornNodeElevation.Stats stats = new MapterhornNodeElevation.Stats();
      double[][] nodes = new double[12][];
      for (int k = 0; k < nodes.length; k++) {
        nodes[k] = node(TX + (k % 4) + 0.3, TY + 0.6);
      }
      sample(sampler, stats, nodes);

      assertEquals(12, stats.sampled);
      assertEquals(4, stats.tiles.get());
      assertTrue("requests " + stats.requests.get(), stats.requests.get() <= 2);
    }
  }

  // --- helpers ---------------------------------------------------------------

  private static byte[] planeTile() throws IOException {
    return TerrainTiles.rgbPng(SIZE, (x, y) -> TerrainTiles.encode(plane(x, y)));
  }

  private static PmTilesArchive archive(int minZoom, int maxZoom) throws IOException {
    PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(minZoom, maxZoom);
    if (maxZoom >= Z) {
      builder.put(Z, TX, TY, planeTile());
    }
    return PmTilesArchive.open(builder.asByteSource());
  }

  private static MapterhornNodeElevation sampler(PmTilesArchive archive) throws IOException {
    Map<String, PmTilesArchive> byLocation = new HashMap<>();
    byLocation.put(ARCHIVE, archive);
    return new MapterhornNodeElevation(List.of(ARCHIVE), Z, 2, byLocation::get);
  }

  /** A node at fractional z15 tile coordinates, as {lon, lat}. */
  private static double[] node(double tx, double ty) {
    double n = 1 << Z;
    double lon = tx / n * 360.0 - 180.0;
    double lat = Math.toDegrees(Math.atan(Math.sinh(Math.PI * (1 - 2 * ty / n))));
    return new double[]{lon, lat};
  }

  private static short[] sample(MapterhornNodeElevation sampler, MapterhornNodeElevation.Stats stats,
                                double[]... nodes) throws IOException {
    int[] ilon = new int[nodes.length];
    int[] ilat = new int[nodes.length];
    for (int i = 0; i < nodes.length; i++) {
      NodeData n = new NodeData(i, nodes[i][0], nodes[i][1]);
      ilon[i] = n.ilon;
      ilat[i] = n.ilat;
    }
    return sampler.sample(ilon, ilat, nodes.length, stats);
  }

  private static int tileX(double lon) {
    return (int) Math.floor((lon + 180.0) / 360.0 * (1 << Z));
  }

  private static int tileY(double lat) {
    double r = Math.toRadians(lat);
    return (int) Math.floor((1.0 - Math.log(Math.tan(r) + 1.0 / Math.cos(r)) / Math.PI) / 2.0 * (1 << Z));
  }
}
