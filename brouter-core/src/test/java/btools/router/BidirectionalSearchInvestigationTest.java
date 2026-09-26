package btools.router;

import org.junit.Assert;
import org.junit.Assume;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.PrintStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import btools.mapaccess.OsmFile;
import btools.mapaccess.OsmNodesMap;
import btools.mapaccess.PhysicalFile;
import btools.codec.MicroCache;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;

/**
 * Investigation: could the routing loop benefit from a bidirectional search?
 * <p>
 * Not a correctness test - a benchmark. It needs full-size segment data
 * (set env var BENCH_SEGMENTS to a segments4 directory) and is skipped otherwise.
 * <p>
 * Test 1 measures the node-pop volume of the real engine per search pass
 * (weighted-A* pass with pass1coefficient=1.5, bounded Dijkstra refinement pass,
 * and the guided "re-tracking" pass) over routes of increasing length.
 * <p>
 * Test 2 harvests the real graph that test 1's longest route loaded into the
 * nodes cache and compares, on identical (start,end) node pairs, the number of
 * settled nodes for: plain Dijkstra, A* with factor 1.0, weighted-A* with
 * factor 1.5 (engine pass 0), Dijkstra bounded by the weighted-A* cost + 5000
 * (engine pass 1), and bidirectional Dijkstra.
 */
public class BidirectionalSearchInvestigationTest {

  private File segmentDir;
  private File workDir;

  @Before
  public void before() throws Exception {
    String sd = System.getenv("BENCH_SEGMENTS");
    if (sd != null) {
      segmentDir = new File(sd);
      if (!segmentDir.isDirectory()) {
        segmentDir = null;
      }
    }
    Assume.assumeTrue("benchmark needs full segment data (set env BENCH_SEGMENTS)", segmentDir != null);
    workDir = Files.createTempDirectory("bidir-bench").toFile();
  }

  // ------------------------------------------------------------------
  // Part 1: real engine, real profile (trekking), real segment data
  // ------------------------------------------------------------------

  // start: Frankfurt/Main center
  private static final double START_LON = 8.6821;
  private static final double START_LAT = 50.1109;

  private static final Object[][] ENDPOINTS = {
    // {lon, lat, label, approx straight-line distance in km}
    {8.6511, 49.8728, "Darmstadt", 27},
    {9.9290, 49.7913, "Wuerzburg", 93},
    {10.9633, 50.2594, "Coburg", 165},
    {11.5755, 48.1372, "Munich", 305},
  };

  @Test
  public void engineSearchVolumesByRouteLength() throws Exception {
    List<String> report = new ArrayList<>();
    report.add("route,endLabel,beelineKm,totalCost,nodesPass0_heur1.5,nodesPass1_boundedDijkstra,nodesRetrack_guided,engineMs");

    PrintStream oldOut = System.out;
    try {
      for (Object[] ep : ENDPOINTS) {
        RoutingContext rctx = new RoutingContext();
        rctx.localFunction = profilePath();

        List<OsmNodeNamed> wplist = new ArrayList<>();
        wplist.add(makeWp("from", START_LON, START_LAT));
        wplist.add(makeWp("to", (double) ep[0], (double) ep[1]));

        ByteArrayOutputStream capture = new ByteArrayOutputStream();
        System.setOut(new PrintStream(capture));
        long t0 = System.currentTimeMillis();
        String err;
        try {
          File outBase = new File(workDir, "benchtrack");
          RoutingEngine re = new RoutingEngine(
            outBase.getAbsolutePath(), outBase.getAbsolutePath(), segmentDir, wplist, rctx);
          re.doRun(0);
          err = re.getErrorMessage();
        } finally {
          System.setOut(oldOut);
        }
        long ms = System.currentTimeMillis() - t0;
        Assert.assertNull("routing failed for " + ep[2] + ": " + err, err);

        // parse per-pass pop counts from the captured engine log
        double factor = -1;
        boolean hasCostCutting = false;
        long nodesHeur = 0;
        long nodesBounded = 0;
        long nodesRetrack = 0;
        int lastCost = -1;
        for (String line : capture.toString("UTF-8").split("\n")) {
          if (line.startsWith("findtrack with airDistanceCostFactor=")) {
            factor = Double.parseDouble(line.substring("findtrack with airDistanceCostFactor=".length()));
            hasCostCutting = false;
          } else if (line.startsWith("costCuttingTrack.cost=")) {
            hasCostCutting = true;
          } else if (line.startsWith("found track at cost ")) {
            String rest = line.substring("found track at cost ".length());
            lastCost = (int) Double.parseDouble(rest.substring(0, rest.indexOf(' ')));
            long nodes = Long.parseLong(rest.substring(rest.indexOf("nodesVisited = ") + 15).trim());
            if (factor == 1.5) {
              nodesHeur += nodes;
            } else if (hasCostCutting) {
              nodesBounded += nodes;
            } else {
              nodesRetrack += nodes;
            }
          }
        }
        String row = "Frankfurt" + "," + ep[2] + "," + ((Number) ep[3]).intValue() + "," + lastCost + "," + nodesHeur + "," + nodesBounded + "," + nodesRetrack + "," + ms;
        report.add(row);
        System.out.println("[GAIN-DIAG] ENGINE " + row);
      }
    } finally {
      System.setOut(oldOut);
    }
    Files.write(new File(workDir, "engine-passes.csv").toPath(), report);
    System.out.println("[GAIN-DIAG] report written to " + new File(workDir, "engine-passes.csv"));
  }

  // ------------------------------------------------------------------
  // Part 2: structural shootout on the real harvested graph
  // ------------------------------------------------------------------

  @Test
  public void bidirectionalStructuralBenchmark() throws Exception {
    // Harvest a real graph directly from the segment files (no engine run):
    // the engine rebuilds its nodes cache for every search pass, so harvesting
    // its live cache mid-run is not reliable.
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profilePath();
    Assert.assertTrue("profile parse failed", rc2expctx(rctx));

    // region: lon 8..9.9 E, lat 49..49.9 N (Frankfurt - Wuerzburg corridor),
    // loaded straight from the rd5 files the way AreaReader does.
    // A 5-degree tile is subdivided into cells of 1/divisor degree.
    System.out.println("[GAIN-DIAG] test VM max mem MB=" + (Runtime.getRuntime().maxMemory() >> 20));
    Map<Long, OsmNode> loaded = new HashMap<>();
    Map<String, PhysicalFile> openFiles = new HashMap<>();
    try {
      for (int latDeg = 49; latDeg <= 49; latDeg++) {
        for (int lonDeg = 8; lonDeg <= 9; lonDeg++) {
          int lon5 = 5 * (lonDeg / 5);
          int lat5 = 5 * (latDeg / 5);
          String tileName = "E" + lon5 + "_N" + lat5;
          PhysicalFile pf = openFiles.get(tileName);
          btools.codec.DataBuffers dataBuffers = new btools.codec.DataBuffers();
          if (pf == null) {
            File file = new File(segmentDir, tileName + ".rd5");
            Assert.assertTrue("missing tile " + file, file.exists());
            pf = new PhysicalFile(file, dataBuffers, -1, -1);
            openFiles.put(tileName, pf);
          }
          OsmFile osmf = new OsmFile(pf, (180000000 + lonDeg * 1000000) / 1000000, (90000000 + latDeg * 1000000) / 1000000, dataBuffers);
          Assert.assertTrue("no data in " + tileName, osmf.hasData());
          int divisor = pf.divisor;
          int cellsize = 1000000 / divisor;
          int kept = 0;
          int total = 0;
          for (int slon = 0; slon < divisor; slon++) {
            for (int slat = 0; slat < divisor; slat++) {
              int inlon = 180000000 + lonDeg * 1000000 + slon * cellsize;
              int inlat = 90000000 + latDeg * 1000000 + slat * cellsize;
              MicroCache segment = osmf.createMicroCache(inlon / cellsize, inlat / cellsize, dataBuffers, rctx.expctxWay, null, true, null);
              if (segment == null) {
                continue;
              }
              OsmNodesMap hollows = new OsmNodesMap();
              int size = segment.getSize();
              for (int i = 0; i < size; i++) {
                long id = segment.getIdForIndex(i);
                if (segment.getAndClear(id)) {
                  OsmNode node = new OsmNode(id);
                  node.parseNodeBody(segment, hollows, rctx.expctxWay);
                  loaded.put(node.getIdFromPos(), node);
                  kept++;
                }
              }
              total += size;
            }
          }
          System.out.println("[GAIN-DIAG] loaded cell " + lonDeg + "E " + latDeg + "N kept=" + kept + "/" + total + ", total nodes=" + loaded.size());
        }
      }
    } finally {
      for (PhysicalFile pf : openFiles.values()) {
        pf.close();
      }
    }
    Assert.assertFalse("no nodes loaded", loaded.isEmpty());

    BenchGraph g = buildGraph(loaded);
    System.out.println("[GAIN-DIAG] GRAPH nodes=" + g.n + " directedEdges=" + g.edgeCnt);

    // sample (start,end) pairs by beeline-distance bucket
    int[] bucketLimitsKm = {5, 20, 50, 150, 1000};
    String[] bucketNames = {"<5km", "5-20km", "20-50km", "50-150km", ">150km"};
    int perBucket = 4;
    Random rnd = new Random(4711);
    List<int[]> pairs = new ArrayList<>();
    for (int b = 0; b < bucketLimitsKm.length; b++) {
      int tries = 0;
      int minKm = b == 0 ? 0 : bucketLimitsKm[b - 1];
      while (pairs.size() < (b + 1) * perBucket && tries++ < 4000) {
        int s = rnd.nextInt(g.n);
        int t = rnd.nextInt(g.n);
        double km = g.distMeters(s, t) / 1000.0;
        if (km >= minKm && km < bucketLimitsKm[b]) {
          pairs.add(new int[]{s, t});
        }
      }
    }

    System.out.println("[GAIN-DIAG] bucket,pairIdx,beelineKm,dijkstraPops,dijkstraCost,aStar1.0Pops,aStar1.5Pops,aStar1.5Cost,boundedDijkstraPops,bidirPops,bidirCost,engineSum_estimate,bidirVsEngineSum");

    int b = 0;
    for (int[] pair : pairs) {
      int s = pair[0];
      int t = pair[1];
      String bucket = bucketNames[Math.min(b / perBucket, bucketNames.length - 1)];
      b++;

      long[] d0 = dijkstra(g, s, t, null);
      if (Double.isInfinite(d0[1])) {
        continue; // unreachable in harvested subgraph
      }
      long popsDijkstra = d0[0];
      double costOpt = d0[1];

      long[] a1 = aStar(g, s, t, 1.0);
      long[] a15 = aStar(g, s, t, 1.5);
      long[] bounded = dijkstra(g, s, t, (double) (a15[1] + 5000)); // engine pass1 bound: pass0 cost + 5000
      long[] bd = bidirectional(g, s, t);

      double beelineKm = g.distMeters(s, t) / 1000.0;
      // engine analogue: pass0 (weighted A* 1.5) + pass1 (bounded Dijkstra)
      long engineSum = a15[0] + bounded[0];
      double ratio = bd[0] > 0 ? (double) engineSum / bd[0] : Double.NaN;
      System.out.println("[GAIN-DIAG] BENCH " + bucket + "," + b + "," + String.format("%.1f", beelineKm)
        + "," + popsDijkstra + "," + (long) costOpt
        + "," + a1[0] + "," + a15[0] + "," + (long) a15[1]
        + "," + bounded[0] + "," + bd[0] + "," + (long) bd[1]
        + "," + engineSum + "," + String.format("%.2f", ratio));
    }
  }

  // ------------------------------------------------------------------
  // Part 3: what if pass 1 ran with an admissible A* heuristic
  // (pass2coefficient=1.0 instead of plain Dijkstra)?
  // ------------------------------------------------------------------

  @Test
  public void pass2HeuristicExperiment() throws Exception {
    Object[][] routes = {
      {9.9290, 49.7913, "Wuerzburg"},
      {10.9633, 50.2594, "Coburg"},
      {11.5755, 48.1372, "Munich"},
    };
    PrintStream oldOut = System.out;
    try {
      for (Object[] rt : routes) {
        for (double p2 : new double[]{0.0, 1.0}) {
          RoutingContext rctx = new RoutingContext();
          rctx.localFunction = profilePath();
          if (p2 != 0.0) {
            rctx.keyValues = new HashMap<>();
            rctx.keyValues.put("pass2coefficient", String.valueOf(p2));
          }
          List<OsmNodeNamed> wplist = new ArrayList<>();
          wplist.add(makeWp("from", START_LON, START_LAT));
          wplist.add(makeWp("to", (double) rt[0], (double) rt[1]));

          ByteArrayOutputStream capture = new ByteArrayOutputStream();
          System.setOut(new PrintStream(capture));
          long t0 = System.currentTimeMillis();
          String err;
          try {
            File outBase = new File(workDir, "p2track" + p2);
            RoutingEngine re = new RoutingEngine(outBase.getAbsolutePath(), outBase.getAbsolutePath(), segmentDir, wplist, rctx);
            re.doRun(0);
            err = re.getErrorMessage();
          } finally {
            System.setOut(oldOut);
          }
          long ms = System.currentTimeMillis() - t0;
          Assert.assertNull("routing failed for " + rt[2] + ": " + err, err);

          double factor = -1;
          boolean hasCostCutting = false;
          long nodesHeur = 0;
          long nodesBounded = 0;
          int lastCost = -1;
          for (String line : capture.toString("UTF-8").split("\n")) {
            if (line.startsWith("findtrack with airDistanceCostFactor=")) {
              factor = Double.parseDouble(line.substring("findtrack with airDistanceCostFactor=".length()));
              hasCostCutting = false;
            } else if (line.startsWith("costCuttingTrack.cost=")) {
              hasCostCutting = true;
            } else if (line.startsWith("found track at cost ")) {
              String rest = line.substring("found track at cost ".length());
              lastCost = (int) Double.parseDouble(rest.substring(0, rest.indexOf(' ')));
              long nodes = Long.parseLong(rest.substring(rest.indexOf("nodesVisited = ") + 15).trim());
              if (factor == 1.5) {
                nodesHeur += nodes;
              } else if (hasCostCutting) {
                nodesBounded += nodes;
              }
            }
          }
          System.out.println("[GAIN-DIAG] P2 route=" + rt[2] + " pass2coefficient=" + p2
            + " cost=" + lastCost + " pass0pops=" + nodesHeur + " pass1pops=" + nodesBounded + " ms=" + ms);
        }
      }
    } finally {
      System.setOut(oldOut);
    }
  }

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------

  private String profilePath() throws Exception {
    String[] candidates = {
      new File(segmentDir, "../misc/profiles2/trekking.brf").getCanonicalPath(),
      new File("misc/profiles2/trekking.brf").getCanonicalPath(),
      new File("../misc/profiles2/trekking.brf").getCanonicalPath(),
    };
    for (String c : candidates) {
      if (new File(c).exists()) {
        return c;
      }
    }
    throw new IllegalStateException("trekking.brf profile not found");
  }

  private static boolean rc2expctx(RoutingContext rctx) {
    return ProfileCache.parseProfile(rctx) || rctx.expctxWay != null;
  }

  // ------------------------------------------------------------------
  // graph model: adjacency from the live nodes cache.
  // Weights are plain link distances (meters, symmetric); direction flags
  // are honored for the adjacency itself.
  // ------------------------------------------------------------------

  private static final class BenchGraph {
    int n;
    int[][][] arc; // forward adjacency: arc[u] = array of {to, weightMeters}
    int[][][] rarc; // reverse adjacency: rarc[v] = array of {from, weightMeters}
    double[] latDeg;
    double[] lonDeg;
    int edgeCnt;

    double distMeters(int a, int bb) {
      // cheap equirectangular approximation, good enough for a bench
      double dlat = latDeg[a] - latDeg[bb];
      double dlon = (lonDeg[a] - lonDeg[bb]) * Math.cos(Math.toRadians((latDeg[a] + latDeg[bb]) / 2));
      return 111194.9 * Math.hypot(dlat, dlon);
    }
  }

  private BenchGraph buildGraph(Map<Long, OsmNode> loaded) {
    Map<Long, Integer> idxMap = new HashMap<>();
    List<OsmNode> nodes = new ArrayList<>();
    for (OsmNode nd : loaded.values()) {
      if (nd.isHollow() || nd.firstlink == null) {
        continue;
      }
      idxMap.put(nd.getIdFromPos(), nodes.size());
      nodes.add(nd);
    }
    BenchGraph g = new BenchGraph();
    g.n = nodes.size();
    g.latDeg = new double[g.n];
    g.lonDeg = new double[g.n];
    List<int[]>[] fw = new List[g.n];
    List<int[]>[] rw = new List[g.n];
    for (int i = 0; i < g.n; i++) {
      OsmNode nd = nodes.get(i);
      g.latDeg[i] = (nd.ilat - 90000000) / 1000000.0;
      g.lonDeg[i] = (nd.ilon - 180000000) / 1000000.0;
      fw[i] = new ArrayList<>();
      rw[i] = new ArrayList<>();
    }
    for (int i = 0; i < g.n; i++) {
      OsmNode nd = nodes.get(i);
      for (OsmLink link = nd.firstlink; link != null; link = link.getNext(nd)) {
        OsmNode tn = link.getTarget(nd);
        Integer jObj = idxMap.get(tn.getIdFromPos());
        if (jObj == null) {
          continue; // hollow or not loaded
        }
        int j = jObj;
        int w = nd.calcDistance(tn);
        boolean bidir = link.isBidirectional();
        if (bidir || !link.isReverse(nd)) {
          fw[i].add(new int[]{j, w});
          rw[j].add(new int[]{i, w});
          g.edgeCnt++;
        }
        if (bidir || link.isReverse(nd)) {
          fw[j].add(new int[]{i, w});
          rw[i].add(new int[]{j, w});
          g.edgeCnt++;
        }
      }
    }
    g.arc = new int[g.n][][];
    g.rarc = new int[g.n][][];
    for (int i = 0; i < g.n; i++) {
      g.arc[i] = flatten(fw[i]);
      g.rarc[i] = flatten(rw[i]);
    }
    return g;
  }

  private static int[][] flatten(List<int[]> l) {
    return l.toArray(new int[0][]);
  }

  // min-heap of (key -> nodeIntEncoded) over double keys
  private static final class MinHeap {
    double[] keys = new double[1024];
    int[] vals = new int[1024];
    int size = 0;

    void push(double k, int v) {
      if (size == keys.length) {
        double[] nk = new double[keys.length * 2];
        int[] nv = new int[vals.length * 2];
        System.arraycopy(keys, 0, nk, 0, size);
        System.arraycopy(vals, 0, nv, 0, size);
        keys = nk;
        vals = nv;
      }
      int i = size++;
      keys[i] = k;
      vals[i] = v;
      while (i > 0) {
        int p = (i - 1) / 2;
        if (keys[p] <= keys[i]) {
          break;
        }
        swap(i, p);
        i = p;
      }
    }

    int pop() {
      int ret = vals[0];
      size--;
      keys[0] = keys[size];
      vals[0] = vals[size];
      int i = 0;
      for (; ; ) {
        int l = 2 * i + 1;
        int r = l + 1;
        int m = i;
        if (l < size && keys[l] < keys[m]) {
          m = l;
        }
        if (r < size && keys[r] < keys[m]) {
          m = r;
        }
        if (m == i) {
          break;
        }
        swap(i, m);
        i = m;
      }
      return ret;
    }

    double topKey() {
      return size == 0 ? Double.POSITIVE_INFINITY : keys[0];
    }

    boolean isEmpty() {
      return size == 0;
    }

    private void swap(int a, int b) {
      double k = keys[a];
      keys[a] = keys[b];
      keys[b] = k;
      int v = vals[a];
      vals[a] = vals[b];
      vals[b] = v;
    }
  }

  /** plain Dijkstra; bound==null => unbounded. Returns {pops, costToTarget}. */
  private static long[] dijkstra(BenchGraph g, int s, int t, Double bound) {
    double[] dist = new double[g.n];
    java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
    dist[s] = 0;
    MinHeap h = new MinHeap();
    h.push(0, s);
    long pops = 0;
    while (!h.isEmpty()) {
      double k = h.topKey();
      int u = h.pop();
      if (k > dist[u]) {
        continue; // stale
      }
      pops++;
      if (u == t) {
        return new long[]{pops, (long) dist[t]};
      }
      for (int[] e : g.arc[u]) {
        double nd = dist[u] + e[1];
        if (bound != null && nd > bound) {
          continue;
        }
        if (nd < dist[e[0]]) {
          dist[e[0]] = nd;
          h.push(nd, e[0]);
        }
      }
    }
    return new long[]{pops, (long) Double.POSITIVE_INFINITY};
  }

  /** A* with heuristic factor f (f=1.0 admissible, f=1.5 = engine pass-0). */
  private static long[] aStar(BenchGraph g, int s, int t, double f) {
    double[] dist = new double[g.n];
    java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
    dist[s] = 0;
    MinHeap h = new MinHeap();
    h.push(0, s);
    long pops = 0;
    while (!h.isEmpty()) {
      double k = h.topKey();
      int u = h.pop();
      if (k > dist[u] + f * g.distMeters(u, t)) {
        continue;
      }
      pops++;
      if (u == t) {
        return new long[]{pops, (long) dist[t]};
      }
      for (int[] e : g.arc[u]) {
        double nd = dist[u] + e[1];
        if (nd < dist[e[0]]) {
          dist[e[0]] = nd;
          h.push(nd + f * g.distMeters(e[0], t), e[0]);
        }
      }
    }
    return new long[]{pops, (long) Double.POSITIVE_INFINITY};
  }

  /** bidirectional Dijkstra on directed graph (reverse front uses rarc). */
  private static long[] bidirectional(BenchGraph g, int s, int t) {
    double[] df = new double[g.n];
    double[] db = new double[g.n];
    java.util.Arrays.fill(df, Double.POSITIVE_INFINITY);
    java.util.Arrays.fill(db, Double.POSITIVE_INFINITY);
    df[s] = 0;
    db[t] = 0;
    MinHeap hf = new MinHeap();
    MinHeap hb = new MinHeap();
    hf.push(0, s);
    hb.push(0, t);
    double mu = Double.POSITIVE_INFINITY;
    long pops = 0;
    while (!hf.isEmpty() && !hb.isEmpty()) {
      if (hf.topKey() + hb.topKey() >= mu) {
        break;
      }
      boolean forward = hf.topKey() <= hb.topKey();
      MinHeap h = forward ? hf : hb;
      double[] dThis = forward ? df : db;
      double[] dOther = forward ? db : df;
      int[][][] adj = forward ? g.arc : g.rarc;
      double k = h.topKey();
      int u = h.pop();
      if (k > dThis[u]) {
        continue;
      }
      pops++;
      for (int[] e : adj[u]) {
        double nd = dThis[u] + e[1];
        if (nd < dThis[e[0]]) {
          dThis[e[0]] = nd;
          h.push(nd, e[0]);
          if (dOther[e[0]] < Double.POSITIVE_INFINITY) {
            double cand = nd + dOther[e[0]];
            if (cand < mu) {
              mu = cand;
            }
          }
        }
      }
    }
    return new long[]{pops, Double.isInfinite(mu) ? (long) Double.POSITIVE_INFINITY : (long) mu};
  }

  private static OsmNodeNamed makeWp(String name, double lon, double lat) {
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = name;
    n.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    n.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    return n;
  }
}
