package btools.router;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

/**
 * Regression anchor for the shared-corridor crossing miss (2026-06-11/12
 * investigation): annecy fastbike 50km AUTO visually self-crosses but
 * countSelfIntersections reports 0 — the crossings are routed THROUGH shared
 * corridors (Rond-Point de la Contamine roundabout on the W route; the Route
 * des Diacquenods same-direction figure-eight on the N route), which the
 * per-node shared-edge guard exempts. The dark corridor counter
 * ({@link RoundTripQualityGate#countCorridorCrossings}) must see exactly one
 * crossing per route, and the opposite-direction retraces (Route de Clermont,
 * Route de Genève) must stay at zero. Also keeps the diagnostic trace output
 * for future investigations.
 */
public class CrossingDetectReproTest {

  @Test
  public void annecyMissedCorridorCrossings() throws Exception {
    for (int dir : new int[]{0, 270}) {
      System.out.println("=============== direction " + dir + " ===============");
      runCase(dir);
    }
  }

  private void runCase(int direction) throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    File profileFile = new File(projectDir, "misc/profiles2/fastbike.brf");

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = (int) ((6.130 + 180.0) * 1e6);
    start.ilat = (int) ((45.900 + 90.0) * 1e6);
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = direction;
    rctx.roundTripDistance = 8000;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    rctx.roundTripStrictQuality = false;

    File out = File.createTempFile("xrepro", "");
    RoutingEngine re = new RoutingEngine(out.getAbsolutePath(), out.getAbsolutePath(),
      segDir, wplist, rctx, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    System.out.println("error=" + re.getErrorMessage());
    OsmTrack track = re.getFoundTrack();
    org.junit.Assert.assertNotNull("routing must produce a track", track);
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    System.out.println("track distance=" + track.distance + " nodes=" + n);

    LoopQualityMetrics m = LoopQualityMetrics.compute(track, 50000, 270);
    System.out.println("metrics selfIntersections=" + m.getSelfIntersections()
      + " gate countSelfIntersections=" + RoundTripQualityGate.countSelfIntersections(track));

    // The regression anchor: each route carries exactly ONE shared-corridor
    // crossing (N: Diacquenods figure-eight; W: Contamine roundabout) that the
    // per-node count misses; the opposite-direction Clermont/Genève retraces
    // must not be counted. Route geometry can legitimately drift with planner
    // changes — if this assert fires after one, re-investigate with the trace
    // output below before re-baselining.
    org.junit.Assert.assertEquals("dark corridor counter must see the missed crossing",
      1, RoundTripQualityGate.countCorridorCrossings(nodes));

    // --- Full-resolution raw segment scan with NO exemptions; classify each hit.
    double[] cum = new double[n];
    for (int k = 1; k < n; k++) cum[k] = cum[k - 1] + nodes.get(k - 1).calcDistance(nodes.get(k));
    double perim = cum[n - 1];
    int raw = 0;
    for (int i = 0; i < n - 1; i++) {
      for (int j = i + 2; j < n - 1; j++) {
        if (i == 0 && j == n - 2) continue;
        if (segCross(nodes.get(i), nodes.get(i + 1), nodes.get(j), nodes.get(j + 1))) {
          raw++;
          boolean home = cum[i + 1] <= 500 || cum[i] >= perim - 500
            || cum[j + 1] <= 500 || cum[j] >= perim - 500;
          boolean bri = RoundTripQualityGate.bridgeOrTunnelEdge(nodes.get(i + 1));
          boolean brj = RoundTripQualityGate.bridgeOrTunnelEdge(nodes.get(j + 1));
          System.out.printf("RAW-X seg %d x %d arc %.1fkm x %.1fkm at (%.5f,%.5f) home=%b bridgeI=%b bridgeJ=%b tagsI=%s tagsJ=%s%n",
            i, j, cum[i] / 1000, cum[j] / 1000,
            nodes.get(i).getILat() / 1e6 - 90.0, nodes.get(i).getILon() / 1e6 - 180.0,
            home, bri, brj, tags(nodes.get(i + 1)), tags(nodes.get(j + 1)));
        }
      }
    }
    System.out.println("raw segment crossings (no exemptions): " + raw);

    // --- Node revisits: every repeated position, with transversality verdict.
    java.util.Map<Long, List<Integer>> occ = new java.util.HashMap<>();
    for (int k = 1; k < n - 1; k++) {
      long id = nodes.get(k).getIdFromPos();
      List<Integer> prev = occ.computeIfAbsent(id, x -> new ArrayList<>());
      for (int k1 : prev) {
        if (k - k1 <= 1) continue;
        boolean trans = RoundTripQualityGate.isTransverseRevisit(nodes, k1, k);
        boolean home = cum[k] <= 500 || cum[k] >= perim - 500 || cum[k1] <= 500;
        System.out.printf("REVISIT idx %d->%d arc %.1fkm->%.1fkm at (%.5f,%.5f) transverse=%b home=%b%n",
          k1, k, cum[k1] / 1000, cum[k] / 1000,
          nodes.get(k).getILat() / 1e6 - 90.0, nodes.get(k).getILon() / 1e6 - 180.0,
          trans, home);
      }
      prev.add(k);
    }

    // --- Corridor-level analysis: group node revisits into maximal shared
    // corridors, collapse each to its centroid, and run the angular interleave
    // test on probe points ~150m OUTSIDE the corridor (arc-wise). Interleaved
    // directions = the loop swaps sides across the corridor = net transverse
    // crossing the per-node shared-edge guard hides.
    List<int[]> pairs = new ArrayList<>(); // {k1, k2} revisits outside home zone
    java.util.Map<Long, List<Integer>> occ2 = new java.util.HashMap<>();
    for (int k = 1; k < n - 1; k++) {
      long id = nodes.get(k).getIdFromPos();
      List<Integer> prev = occ2.computeIfAbsent(id, x -> new ArrayList<>());
      for (int k1 : prev) {
        if (k - k1 <= 1) continue;
        if (cum[k] <= 500 || cum[k] >= perim - 500 || cum[k1] <= 500) continue;
        pairs.add(new int[]{k1, k});
      }
      prev.add(k);
    }
    // group: pairs whose k1 AND k2 are arc-adjacent (<300m) belong to one corridor
    pairs.sort((a, b) -> Integer.compare(a[0], b[0]));
    List<List<int[]>> corridors = new ArrayList<>();
    for (int[] p : pairs) {
      List<int[]> cur = corridors.isEmpty() ? null : corridors.get(corridors.size() - 1);
      int[] last = cur == null ? null : cur.get(cur.size() - 1);
      if (last != null && Math.abs(cum[p[0]] - cum[last[0]]) < 300
        && Math.abs(cum[p[1]] - cum[last[1]]) < 300) {
        cur.add(p);
      } else {
        List<int[]> nc = new ArrayList<>();
        nc.add(p);
        corridors.add(nc);
      }
    }
    for (List<int[]> c : corridors) {
      int a1 = Integer.MAX_VALUE, a2 = -1, b1 = Integer.MAX_VALUE, b2 = -1;
      for (int[] p : c) {
        a1 = Math.min(a1, p[0]); a2 = Math.max(a2, p[0]);
        b1 = Math.min(b1, p[1]); b2 = Math.max(b2, p[1]);
      }
      // centroid of pass-1 corridor span
      double clat = 0, clon = 0; int cnt = 0;
      for (int k = a1; k <= a2; k++) { clat += nodes.get(k).getILat(); clon += nodes.get(k).getILon(); cnt++; }
      clat /= cnt; clon /= cnt;
      int p1in = probeIdx(cum, a1, -150), p1out = probeIdx(cum, a2, +150);
      int p2in = probeIdx(cum, b1, -150), p2out = probeIdx(cum, b2, +150);
      double b1in = bearingFrom(clon, clat, nodes.get(p1in));
      double b1out = bearingFrom(clon, clat, nodes.get(p1out));
      double c1 = bearingFrom(clon, clat, nodes.get(p2in));
      double c2 = bearingFrom(clon, clat, nodes.get(p2out));
      boolean c1in = angleInSectorLocal(c1, b1in, b1out);
      boolean c2in = angleInSectorLocal(c2, b1in, b1out);
      boolean transverse = c1in != c2in;
      System.out.printf("CORRIDOR pass1 idx %d..%d (arc %.1f-%.1fkm) pass2 idx %d..%d (arc %.1f-%.1fkm) len %.0fm at (%.5f,%.5f) NET-TRANSVERSE=%b (pass1 dirs %.0f/%.0f, pass2 dirs %.0f/%.0f)%n",
        a1, a2, cum[a1] / 1000, cum[a2] / 1000, b1, b2, cum[b1] / 1000, cum[b2] / 1000,
        cum[a2] - cum[a1],
        clat / 1e6 - 90.0, clon / 1e6 - 180.0, transverse, b1in, b1out, c1, c2);
    }
    dumpClermont(nodes, cum);
  }

  // appended by runCase: detailed dump + GeoJSON of the Route de Clermont area
  private void dumpClermont(List<OsmPathElement> nodes, double[] cum) throws Exception {
    StringBuilder gj = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
    int[][] spans = {{1100, 1160}, {1400, 1460}, {830, 890}, {1550, 1610}};
    String[] names = {"clermont-outbound", "clermont-return", "roundabout-outbound", "roundabout-return"};
    for (int s = 0; s < spans.length; s++) {
      StringBuilder line = new StringBuilder();
      for (int k = spans[s][0]; k <= spans[s][1]; k++) {
        double lat = nodes.get(k).getILat() / 1e6 - 90.0;
        double lon = nodes.get(k).getILon() / 1e6 - 180.0;
        if (line.length() > 0) line.append(",");
        line.append(String.format(java.util.Locale.US, "[%.6f,%.6f]", lon, lat));
      }
      if (s > 0) gj.append(",");
      gj.append("{\"type\":\"Feature\",\"properties\":{\"name\":\"").append(names[s])
        .append("\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[").append(line).append("]}}");
    }
    gj.append("]}");
    java.nio.file.Files.write(java.nio.file.Paths.get("/tmp/clermont.geojson"),
      gj.toString().getBytes());
    System.out.println("wrote /tmp/clermont.geojson");
  }

  private static int probeIdx(double[] cum, int from, int deltaM) {
    int k = from;
    if (deltaM < 0) { while (k > 0 && cum[from] - cum[k] < -deltaM) k--; }
    else { while (k < cum.length - 1 && cum[k] - cum[from] < deltaM) k++; }
    return k;
  }

  private static double bearingFrom(double clon, double clat, OsmPathElement p) {
    double dlat = p.getILat() - clat;
    double dlon = (p.getILon() - clon) * Math.cos(Math.toRadians(clat / 1e6 - 90.0));
    return (Math.toDegrees(Math.atan2(dlon, dlat)) + 360.0) % 360.0;
  }

  /** is angle a inside the CW sector from s1 to s2? (same convention as gate's angleInSector) */
  private static boolean angleInSectorLocal(double a, double s1, double s2) {
    double span = (s2 - s1 + 360.0) % 360.0;
    double off = (a - s1 + 360.0) % 360.0;
    return off < span;
  }

  private static String tags(OsmPathElement e) {
    if (e == null || e.message == null || e.message.wayKeyValues == null) return "-";
    return e.message.wayKeyValues;
  }

  private static boolean segCross(OsmPathElement p1, OsmPathElement p2,
                                  OsmPathElement p3, OsmPathElement p4) {
    long d1 = ccw(p3, p4, p1), d2 = ccw(p3, p4, p2), d3 = ccw(p1, p2, p3), d4 = ccw(p1, p2, p4);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  }

  private static long ccw(OsmPathElement a, OsmPathElement b, OsmPathElement c) {
    long ax = a.getILon(), ay = a.getILat();
    return (long) (c.getILon() - ax) * (b.getILat() - ay)
      - (long) (b.getILon() - ax) * (c.getILat() - ay);
  }
}
