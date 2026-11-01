package btools.mapcreator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicLong;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Samples every node of a cutter's node files from Mapterhorn's high-zoom regional
 * archives, and writes one {@link NodeElevationSidecar} per node file for
 * {@link PosUnifier} to prefer over the .bef raster.
 * <p>
 * <b>Why per node and not a finer raster.</b> The .bef raster is resampled from zoom 12,
 * whose ~13 m pixels blend a hairpin road with the hillside above and below it: the
 * road's profile then carries steps of several metres over a few metres of road, which
 * read as 30-40 % ramps on climbs that are 11 % (Alpe d'Huez). Zoom 15 pixels are ~1.7 m
 * at 45 deg N, fine enough to stay on the road, but a raster at that resolution would
 * be ~60x the size of today's. Only the nodes need an elevation, so each node is sampled
 * once from the tile that holds it and the tile is then dropped.
 * <p>
 * <b>One pass, no cache.</b> Nodes are visited in the archives' own tile order (Hilbert
 * order is byte order in a clustered PMTiles archive), so each tile body is fetched
 * exactly once, adjacent bodies share one range request, and nothing is kept between
 * runs. A node the archives do not cover -- no archive at that zoom, tile absent, or a
 * no-data pixel -- gets no sidecar elevation and keeps the raster's value.
 * <p>
 * Sampling is bilinear between pixel centres (pixel i covers [i, i+1)). A node within
 * half a pixel of its tile's edge is clamped to the edge pixels rather than read across
 * into the neighbour tile: at most ~0.85 m of horizontal shift, against a raster today
 * whose cells are ~30 m.
 * <p>
 * Usage:
 * <pre>
 * MapterhornNodeElevation &lt;node-tiles-in&gt; &lt;sidecar-out&gt; &lt;archive-list&gt; [-zoom 15] [-threads 4]
 *   &lt;archive-list&gt;  text file, one regional archive per line (an https:// URL or a
 *                   local path) named z-x-y.pmtiles after the tile it covers, e.g.
 *                   https://download.mapterhorn.com/6-33-22.pmtiles. Archives whose
 *                   zoom range does not include -zoom are skipped.
 * </pre>
 */
public final class MapterhornNodeElevation implements AutoCloseable {

  static final int DEFAULT_ZOOM = 15;

  /** Tiles decoded per batch; bounds memory to about this many decoded tiles per thread. */
  private static final int TILE_BATCH = 64;

  /** Upper bound on one range request, so a long contiguous run is still split. */
  private static final int MAX_RUN_BYTES = 8 * 1024 * 1024;

  private static final Pattern ARCHIVE_NAME = Pattern.compile("(\\d+)-(\\d+)-(\\d+)\\.pmtiles$");

  @FunctionalInterface
  interface ArchiveOpener {
    PmTilesArchive open(String location) throws IOException;
  }

  private static final class Archive {
    /** Position in the archive list, to keep one archive's tiles together in a batch. */
    final int index;
    final PmTilesArchive pmtiles;

    Archive(int index, PmTilesArchive pmtiles) {
      this.index = index;
      this.pmtiles = pmtiles;
    }
  }

  /** One source tile and the run of sorted node keys it holds. */
  private static final class TileJob {
    final int x;
    final int y;
    final Archive archive;
    final int from;
    final int to;
    PmTilesArchive.TileLocation location;

    TileJob(int x, int y, Archive archive, int from, int to) {
      this.x = x;
      this.y = y;
      this.archive = archive;
      this.from = from;
      this.to = to;
    }
  }

  /** Counters of one node file, for the log line and the tests. */
  static final class Stats {
    long nodes;
    long sampled;
    long uncovered;
    long absentTile;
    long noData;
    final AtomicLong tiles = new AtomicLong();
    final AtomicLong requests = new AtomicLong();
    final AtomicLong bytes = new AtomicLong();

    @Override
    public String toString() {
      return nodes + " nodes: " + sampled + " sampled, " + uncovered + " outside the archives, "
        + absentTile + " in absent tiles, " + noData + " on no-data pixels; "
        + tiles.get() + " tiles in " + requests.get() + " requests, "
        + (bytes.get() / (1024 * 1024)) + " MiB";
    }
  }

  private final int zoom;
  private final int threads;
  /** Regional archives by the tile they cover, keyed as {@link #key}. */
  private final Map<Long, Archive> archives = new HashMap<>();
  private final List<Integer> archiveZooms = new ArrayList<>();

  MapterhornNodeElevation(List<String> locations, int zoom, int threads, ArchiveOpener opener)
    throws IOException {
    this.zoom = zoom;
    this.threads = threads;
    try {
      for (String location : locations) {
        Matcher m = ARCHIVE_NAME.matcher(location);
        if (!m.find()) {
          throw new IllegalArgumentException("archive name is not z-x-y.pmtiles: " + location);
        }
        int az = Integer.parseInt(m.group(1));
        if (az > zoom) {
          throw new IllegalArgumentException("archive tile zoom " + az + " is finer than -zoom "
            + zoom + ": " + location);
        }
        PmTilesArchive pmtiles = opener.open(location);
        if (pmtiles.minZoom() > zoom || pmtiles.maxZoom() < zoom) {
          System.out.println("*** skipping " + location + ": zoom " + pmtiles.minZoom() + "-"
            + pmtiles.maxZoom() + " does not include " + zoom);
          pmtiles.close();
          continue;
        }
        Archive previous = archives.put(
          key(az, Integer.parseInt(m.group(2)), Integer.parseInt(m.group(3))),
          new Archive(archives.size(), pmtiles));
        if (previous != null) {
          previous.pmtiles.close();
          throw new IllegalArgumentException("archive listed twice: " + location);
        }
        if (!archiveZooms.contains(az)) {
          archiveZooms.add(az);
        }
      }
    } catch (IOException | RuntimeException e) {
      close();
      throw e;
    }
  }

  public static void main(String[] args) throws Exception {
    System.out.println("*** MapterhornNodeElevation: sample node elevations from Mapterhorn");
    if (args.length < 3) {
      System.out.println("usage: java MapterhornNodeElevation <node-tiles-in> <sidecar-out> <archive-list> [-zoom 15] [-threads 4]");
      System.exit(2);
    }
    int zoom = DEFAULT_ZOOM;
    int threads = 4;
    for (int i = 3; i < args.length; i += 2) {
      if (i + 1 >= args.length) {
        throw new IllegalArgumentException("missing value for " + args[i]);
      }
      if ("-zoom".equals(args[i])) {
        zoom = Integer.parseInt(args[i + 1]);
      } else if ("-threads".equals(args[i])) {
        threads = Integer.parseInt(args[i + 1]);
      } else {
        throw new IllegalArgumentException("unknown option " + args[i]);
      }
    }
    TerrariumTileDecoder.disableImageIoDiskCache();
    try (MapterhornNodeElevation sampler = new MapterhornNodeElevation(
        readArchiveList(new File(args[2])), zoom, threads, PmTilesArchive::open)) {
      sampler.processDir(new File(args[0]), new File(args[1]));
    }
  }

  static List<String> readArchiveList(File list) throws IOException {
    List<String> locations = new ArrayList<>();
    try (BufferedReader r = new BufferedReader(new FileReader(list))) {
      for (String line; (line = r.readLine()) != null; ) {
        line = line.trim();
        if (!line.isEmpty() && !line.startsWith("#")) {
          locations.add(line);
        }
      }
    }
    return locations;
  }

  void processDir(File nodeTilesIn, File outDir) throws Exception {
    if (!nodeTilesIn.isDirectory()) {
      throw new IllegalArgumentException("not a directory: " + nodeTilesIn);
    }
    File[] files = nodeTilesIn.listFiles((dir, name) -> name.endsWith(".n5d"));
    Arrays.sort(files);
    for (File nodeFile : files) {
      long start = System.currentTimeMillis();
      Stats stats = processFile(nodeFile, NodeElevationSidecar.forNodeFile(outDir, nodeFile));
      System.out.println("*** " + nodeFile.getName() + ": " + stats + " ("
        + (System.currentTimeMillis() - start) / 1000 + " s)");
    }
  }

  Stats processFile(File nodeFile, File sidecar) throws Exception {
    NodePositions nodes = new NodePositions();
    new NodeIterator(nodes, false).processFile(nodeFile);
    Stats stats = new Stats();
    short[] selev = sample(nodes.ilon, nodes.ilat, nodes.size, stats);
    NodeElevationSidecar.write(sidecar, zoom, Arrays.copyOf(nodes.nid, nodes.size), selev);
    return stats;
  }

  /**
   * Elevations in quarter metres for nodes at BRouter's integer positions (micro-degrees,
   * offset by 180/90), {@link Short#MIN_VALUE} where the archives have none.
   */
  short[] sample(int[] ilon, int[] ilat, int n, Stats stats) throws IOException {
    short[] selev = new short[n];
    Arrays.fill(selev, Short.MIN_VALUE);
    stats.nodes = n;

    // node index in the low half, the tile's Hilbert id above it: sorting groups nodes by
    // tile and visits the tiles in archive byte order
    long[] keys = new long[n];
    int keyed = 0;
    for (int i = 0; i < n; i++) {
      double fx = tileX(ilon[i]);
      double fy = tileY(ilat[i]);
      if (Double.isNaN(fy)) {
        continue;
      }
      long id = PmTilesArchive.zxyToTileId(zoom, (int) Math.floor(fx), (int) Math.floor(fy));
      keys[keyed++] = id << 32 | i;
    }
    stats.uncovered = n - keyed;
    Arrays.sort(keys, 0, keyed);

    List<TileJob> jobs = new ArrayList<>();
    for (int from = 0; from < keyed; ) {
      int to = from + 1;
      while (to < keyed && keys[to] >>> 32 == keys[from] >>> 32) {
        to++;
      }
      int i = (int) keys[from];
      int x = (int) Math.floor(tileX(ilon[i]));
      int y = (int) Math.floor(tileY(ilat[i]));
      Archive archive = archiveFor(x, y);
      if (archive == null) {
        stats.uncovered += to - from;
      } else {
        jobs.add(new TileJob(x, y, archive, from, to));
      }
      from = to;
    }

    ExecutorService pool = Executors.newFixedThreadPool(threads);
    try {
      for (int b = 0; b < jobs.size(); b += TILE_BATCH) {
        List<TileJob> batch = jobs.subList(b, Math.min(jobs.size(), b + TILE_BATCH));
        List<TileJob> present = new ArrayList<>();
        for (TileJob job : batch) {
          job.location = job.archive.pmtiles.locateTile(zoom, job.x, job.y);
          if (job.location == null) {
            stats.absentTile += job.to - job.from;
          } else {
            present.add(job);
          }
        }
        List<Future<Long>> runs = new ArrayList<>();
        for (List<TileJob> run : contiguousRuns(present)) {
          runs.add(pool.submit(() -> fetchAndSample(run, keys, ilon, ilat, selev, stats)));
        }
        for (Future<Long> run : runs) {
          stats.noData += await(run);
        }
      }
    } finally {
      pool.shutdownNow();
    }
    stats.sampled = n - stats.uncovered - stats.absentTile - stats.noData;
    return selev;
  }

  /** @return how many of the run's nodes fell on no-data pixels. */
  private long fetchAndSample(List<TileJob> run, long[] keys, int[] ilon, int[] ilat,
                              short[] selev, Stats stats) throws IOException {
    PmTilesArchive pmtiles = run.get(0).archive.pmtiles;
    long start = run.get(0).location.offset;
    long end = 0;
    for (TileJob job : run) {
      end = Math.max(end, job.location.offset + job.location.length);
    }
    byte[] bytes = pmtiles.readRange(start, PmTilesArchive.checkedInt(end - start, "run length"));
    stats.requests.incrementAndGet();
    stats.bytes.addAndGet(bytes.length);
    long noData = 0;
    for (TileJob job : run) {
      int off = (int) (job.location.offset - start);
      byte[] body = Arrays.copyOfRange(bytes, off, off + job.location.length);
      TerrariumTileDecoder.Tile tile = TerrariumTileDecoder.decode(pmtiles.decompressTile(body));
      stats.tiles.incrementAndGet();
      for (int k = job.from; k < job.to; k++) {
        int i = (int) keys[k];
        short e = sampleQuarterMetres(tile, tileX(ilon[i]) - job.x, tileY(ilat[i]) - job.y);
        if (e == Short.MIN_VALUE) {
          noData++;
        }
        selev[i] = e;
      }
    }
    return noData;
  }

  /**
   * Bilinear elevation at a position inside the tile, given as the fraction of the tile's
   * width and height from its north-west corner; no-data if any of the four pixels is.
   */
  static short sampleQuarterMetres(TerrariumTileDecoder.Tile tile, double tx, double ty) {
    int size = tile.size();
    double u = tx * size - 0.5;
    double v = ty * size - 0.5;
    int i0 = clamp((int) Math.floor(u), 0, size - 2);
    int j0 = clamp((int) Math.floor(v), 0, size - 2);
    double a = Math.max(0.0, Math.min(1.0, u - i0));
    double b = Math.max(0.0, Math.min(1.0, v - j0));
    double e00 = tile.get(i0, j0);
    double e10 = tile.get(i0 + 1, j0);
    double e01 = tile.get(i0, j0 + 1);
    double e11 = tile.get(i0 + 1, j0 + 1);
    double e = (1 - b) * ((1 - a) * e00 + a * e10) + b * ((1 - a) * e01 + a * e11);
    if (Double.isNaN(e) || e < TerrariumTileDecoder.MIN_VALID_ELEVATION) {
      return Short.MIN_VALUE;
    }
    long q = Math.round(e * 4.0);
    return q >= Short.MAX_VALUE ? Short.MIN_VALUE : (short) q;
  }

  private Archive archiveFor(int x, int y) {
    for (int az : archiveZooms) {
      int shift = zoom - az;
      Archive a = archives.get(key(az, x >>> shift, y >>> shift));
      if (a != null) {
        return a;
      }
    }
    return null;
  }

  /** Tiles whose bodies follow each other in one archive, up to {@link #MAX_RUN_BYTES}. */
  private static List<List<TileJob>> contiguousRuns(List<TileJob> jobs) {
    List<TileJob> sorted = new ArrayList<>(jobs);
    sorted.sort((p, q) -> p.archive != q.archive
      ? Integer.compare(p.archive.index, q.archive.index)
      : Long.compare(p.location.offset, q.location.offset));
    List<List<TileJob>> runs = new ArrayList<>();
    List<TileJob> run = null;
    long runStart = 0;
    for (TileJob job : sorted) {
      if (run != null) {
        TileJob prev = run.get(run.size() - 1);
        if (prev.archive == job.archive && job.location.isContiguousWith(prev.location)
          && job.location.offset + job.location.length - runStart <= MAX_RUN_BYTES) {
          run.add(job);
          continue;
        }
      }
      run = new ArrayList<>();
      run.add(job);
      runStart = job.location.offset;
      runs.add(run);
    }
    return runs;
  }

  /** Fractional tile column at {@link #zoom}. */
  private double tileX(int ilon) {
    double lon = (ilon - 180000000) / 1e6;
    return (lon + 180.0) / 360.0 * (1 << zoom);
  }

  /** Fractional tile row at {@link #zoom}, NaN outside Web Mercator's latitude range. */
  private double tileY(int ilat) {
    double lat = (ilat - 90000000) / 1e6;
    if (Math.abs(lat) >= 85.0511287798066) {
      return Double.NaN;
    }
    double r = Math.toRadians(lat);
    return (1.0 - Math.log(Math.tan(r) + 1.0 / Math.cos(r)) / Math.PI) / 2.0 * (1 << zoom);
  }

  private static long key(int z, int x, int y) {
    return ((long) z << 58) | ((long) x << 29) | y;
  }

  private static int clamp(int v, int lo, int hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static long await(Future<Long> f) throws IOException {
    try {
      return f.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("interrupted", e);
    } catch (java.util.concurrent.ExecutionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof IOException) {
        throw (IOException) cause;
      }
      throw new IOException(cause);
    }
  }

  @Override
  public void close() throws IOException {
    IOException failure = null;
    for (Archive a : archives.values()) {
      try {
        a.pmtiles.close();
      } catch (IOException e) {
        failure = e;
      }
    }
    archives.clear();
    if (failure != null) {
      throw failure;
    }
  }

  /** Node ids and positions of one node file, in file order. */
  private static final class NodePositions implements NodeListener {
    long[] nid = new long[1 << 16];
    int[] ilon = new int[1 << 16];
    int[] ilat = new int[1 << 16];
    int size;

    @Override
    public void nodeFileStart(File nodefile) {
      size = 0;
    }

    @Override
    public void nextNode(NodeData n) {
      if (size == nid.length) {
        int grown = Math.multiplyExact(nid.length, 2);
        nid = Arrays.copyOf(nid, grown);
        ilon = Arrays.copyOf(ilon, grown);
        ilat = Arrays.copyOf(ilat, grown);
      }
      nid[size] = n.nid;
      ilon[size] = n.ilon;
      ilat[size] = n.ilat;
      size++;
    }

    @Override
    public void nodeFileEnd(File nodefile) {
    }
  }
}
