package btools.router;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.NodesCache;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import btools.mapaccess.OsmNodePairSet;
import org.junit.Test;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * One-off diagnostic trace for the user-confirmed spur defect:
 * urban_berlin_100km_gravel_E [iso_greedy], the eastern out-and-back near Wegendorf.
 *
 * Goal: dump the committed step-endpoint waypoints and the spur span, then test the
 * root-cause hypothesis — did a committed waypoint land in the near-dead-end at the
 * antenna tip (forcing the out-and-back), or is the spur internal to a single Dijkstra leg?
 *
 * Not a gate. Prints to stdout; always "passes". Run with:
 *   ./gradlew :brouter-core:integrationTest --tests '*LoopWegendorfTraceTest*'
 */
public class LoopWegendorfTraceTest {

  // Mirror LoopQualityMetrics spur primitive (LOCKED).
  private static final double SPUR_EPS_METERS = 60.0;
  private static final double SPUR_MIN_ARC_GAP = 600.0;
  private static final double SPUR_MAX_ARC_GAP = 6000.0;

  @Test
  public void traceWegendorf() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    // Parameterizable: -Dloop.region=BASEL -Dloop.dir=180 -Dloop.radius=4800 -Dloop.profile=gravel
    LoopTestRegion region = LoopTestRegion.valueOf(System.getProperty("loop.region", "URBAN_BERLIN"));
    int dir = Integer.getInteger("loop.dir", 90);
    int radius = Integer.getInteger("loop.radius", 15900);
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + System.getProperty("loop.profile", "gravel") + ".brf");
    if (!profileFile.exists()) {
      System.out.println("TRACE: profile missing, skipping: " + profileFile);
      return;
    }
    System.out.printf("TRACE region=%s dir=%d radius=%d%n", region, dir, radius);

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = region.ilon;
    start.ilat = region.ilat;
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = dir;
    rctx.roundTripDistance = radius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISO_GREEDY;
    rctx.roundTripStrictQuality = false; // get the route even if gate-rejected

    File tmp = File.createTempFile("wegendorf", "");
    String outPath = tmp.getAbsolutePath();
    RoutingEngine re = new RoutingEngine(
      outPath, outPath, segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);

    OsmTrack track = re.getFoundTrack();
    if (track == null) track = re.getLastRejectedTrack();
    if (track == null) {
      System.out.println("TRACE: no track at all. error=" + re.getErrorMessage());
      return;
    }

    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    System.out.println("TRACE: track nodes=" + n + " dist=" + track.distance
      + " error=" + re.getErrorMessage());

    // --- committed step-endpoint waypoints ---
    List<MatchedWaypoint> mwps = track.matchedWaypoints;
    System.out.println("TRACE: committed waypoints (count="
      + (mwps == null ? 0 : mwps.size()) + "):");
    if (mwps != null) {
      for (int w = 0; w < mwps.size(); w++) {
        MatchedWaypoint mw = mwps.get(w);
        if (mw.crosspoint == null) { System.out.println("  wp[" + w + "] crosspoint=null name=" + mw.name); continue; }
        double lon = (mw.crosspoint.getILon() - 180000000) / 1000000.0;
        double lat = (mw.crosspoint.getILat() - 90000000) / 1000000.0;
        // nearest track-node index to this waypoint
        int nearIdx = nearestNodeIndex(nodes, mw.crosspoint.getILon(), mw.crosspoint.getILat());
        System.out.printf("  wp[%d] name=%s lat=%.5f lon=%.5f nearTrackIdx=%d%n",
          w, mw.name, lat, lon, nearIdx);
      }
    }

    // --- spur span detection (re-run the locked scan but keep indices) ---
    double[] cum = new double[n];
    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    int i = 0;
    int spurNo = 0;
    while (i < n) {
      int bestJ = -1;
      OsmPathElement a = nodes.get(i);
      for (int j = i + 1; j < n; j++) {
        double gap = cum[j] - cum[i];
        if (gap > SPUR_MAX_ARC_GAP) break;
        if (gap >= SPUR_MIN_ARC_GAP && a.calcDistance(nodes.get(j)) <= SPUR_EPS_METERS) bestJ = j;
      }
      if (bestJ >= 0) {
        spurNo++;
        int arc = (int) Math.round(cum[bestJ] - cum[i]);
        // tip = farthest-from-entry node within [i, bestJ]
        int tip = i;
        double maxd = 0;
        for (int k = i; k <= bestJ; k++) {
          double d = a.calcDistance(nodes.get(k));
          if (d > maxd) { maxd = d; tip = k; }
        }
        OsmPathElement tipNode = nodes.get(tip);
        double tipLon = (tipNode.getILon() - 180000000) / 1000000.0;
        double tipLat = (tipNode.getILat() - 90000000) / 1000000.0;
        double reachOverLoop = arc > 0 ? maxd / arc : 0;
        System.out.printf("TRACE SPUR #%d: idx[%d..%d] arc=%dm tipIdx=%d tipLat=%.5f tipLon=%.5f tipReach=%dm reach/loop=%.3f%n",
          spurNo, i, bestJ, arc, tip, tipLat, tipLon, (int) maxd, reachOverLoop);

        // (check 3) actual min approach distance between the entry node[i] and any
        // later node in the span — and the global min over all pairs in span.
        double minEntryReentry = Double.MAX_VALUE;
        for (int k = i + 1; k <= bestJ; k++) {
          double d = a.calcDistance(nodes.get(k));
          if (cum[k] - cum[i] >= SPUR_MIN_ARC_GAP) minEntryReentry = Math.min(minEntryReentry, d);
        }
        System.out.printf("   minEntryReentryDist=%dm (removeMicroDetours proximity=50m)%n", (int) minEntryReentry);

        // (check 2) node-ID sharing: outbound [i..tip] vs inbound [tip..bestJ].
        java.util.Set<Long> outbound = new java.util.HashSet<>();
        for (int k = i; k <= tip; k++) outbound.add(nodes.get(k).getIdFromPos());
        int shared = 0, inboundCount = 0;
        for (int k = tip; k <= bestJ; k++) {
          inboundCount++;
          if (outbound.contains(nodes.get(k).getIdFromPos())) shared++;
        }
        System.out.printf("   nodeID sharing outbound∩inbound: %d/%d inbound nodes also on outbound (retrace if high, parallel if ~0)%n",
          shared, inboundCount);

        // (beeline check) dump every node in [i-2 .. bestJ+2] with segment length.
        // A beeline shows as ONE long edge (no road shape-nodes); road-following
        // shows many short segments. Flag segments > 200m.
        int lo = Math.max(0, i - 2), hi = Math.min(n - 1, bestJ + 2);
        System.out.println("   --- node dump [" + lo + ".." + hi + "] (segLen = dist from prev) ---");
        for (int k = lo; k <= hi; k++) {
          OsmPathElement nd = nodes.get(k);
          double lat = (nd.getILat() - 90000000) / 1000000.0;
          double lon = (nd.getILon() - 180000000) / 1000000.0;
          int seg = (k > lo) ? (int) nodes.get(k - 1).calcDistance(nd) : 0;
          String flag = seg > 200 ? "  <== LONG EDGE" : "";
          String mark = (k == i ? " [ENTRY]" : (k == tip ? " [TIP/via3]" : (k == bestJ ? " [REENTRY]" : "")));
          String way = (nd.message == null) ? "msg=null" : ("way=[" + nd.message.wayKeyValues + "]");
          String beeline = (nd.message == null || nd.message.wayKeyValues == null || nd.message.wayKeyValues.isEmpty())
            ? "  <<< BEELINE (no way tags)" : "";
          System.out.printf("     idx=%d lat=%.5f lon=%.5f segLen=%dm %s%s%s%s%n", k, lat, lon, seg, way, mark, flag, beeline);
        }
        // Is any committed waypoint near the tip? (root-cause test)
        if (mwps != null) {
          for (int w = 0; w < mwps.size(); w++) {
            MatchedWaypoint mw = mwps.get(w);
            if (mw.crosspoint == null) continue;
            double dWp = CheapRulerDist(tipNode.getILon(), tipNode.getILat(),
              mw.crosspoint.getILon(), mw.crosspoint.getILat());
            if (dWp <= 300) {
              System.out.printf("   -> committed wp[%d] (%s) is %dm from this spur tip  <== WAYPOINT-IN-DEADEND%n",
                w, mw.name, (int) dWp);
            }
          }
          // also report nearest committed wp to the tip regardless of distance
          int bw = -1; double bd = Double.MAX_VALUE;
          for (int w = 0; w < mwps.size(); w++) {
            MatchedWaypoint mw = mwps.get(w);
            if (mw.crosspoint == null) continue;
            double dWp = CheapRulerDist(tipNode.getILon(), tipNode.getILat(),
              mw.crosspoint.getILon(), mw.crosspoint.getILat());
            if (dWp < bd) { bd = dWp; bw = w; }
          }
          System.out.printf("   nearest committed wp to tip = wp[%d] at %dm%n", bw, (int) bd);
        }
        i = bestJ;
      } else {
        i++;
      }
    }
    if (spurNo == 0) System.out.println("TRACE: no spur detected on this track.");
  }

  /**
   * Decisive A-vs-B probe: route point-to-point from junction X (52.60672,13.77000)
   * to the via3 dead-end tip (52.60652,13.78198) on gravel. If a real road track
   * comes back (shape nodes + way tags), the null-way leg in the loop was an
   * UNDETAILED real road (metadata gap); if it islands/beelines, via3 is genuinely
   * off-road (candidate-placement defect).
   */
  @Test
  public void probeJunctionXtoVia3() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, LoopTestRegion.URBAN_BERLIN);
    File profileFile = new File(projectDir, "misc/profiles2/gravel.brf");
    if (!profileFile.exists()) { System.out.println("PROBE: profile missing"); return; }

    OsmNodeNamed x = new OsmNodeNamed();
    x.name = "junctionX";
    x.ilon = 180000000 + (int) (13.77000 * 1000000 + 0.5);
    x.ilat = 90000000 + (int) (52.60672 * 1000000 + 0.5);
    OsmNodeNamed v3 = new OsmNodeNamed();
    v3.name = "via3tip";
    v3.ilon = 180000000 + (int) (13.78198 * 1000000 + 0.5);
    v3.ilat = 90000000 + (int) (52.60652 * 1000000 + 0.5);
    List<OsmNodeNamed> wplist = new ArrayList<>();
    wplist.add(x);
    wplist.add(v3);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    File tmp = File.createTempFile("probe", "");
    RoutingEngine re = new RoutingEngine(
      tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wplist, rctx, 0);
    re.doRun(0);
    OsmTrack t = re.getFoundTrack();
    if (t == null) {
      System.out.println("PROBE: NO TRACK (island/unroutable). error=" + re.getErrorMessage()
        + "  => via3 is genuinely off-road (candidate-placement defect)");
      return;
    }
    int nulls = 0, tagged = 0, longEdges = 0;
    for (int k = 0; k < t.nodes.size(); k++) {
      OsmPathElement nd = t.nodes.get(k);
      boolean nullWay = nd.message == null || nd.message.wayKeyValues == null || nd.message.wayKeyValues.isEmpty();
      if (nullWay) nulls++; else tagged++;
      int seg = k > 0 ? (int) t.nodes.get(k - 1).calcDistance(nd) : 0;
      if (seg > 200) longEdges++;
    }
    System.out.printf("PROBE X->via3: dist=%dm nodes=%d nullWayNodes=%d taggedNodes=%d longEdges(>200m)=%d%n",
      t.distance, t.nodes.size(), nulls, tagged, longEdges);
    System.out.println("PROBE interpretation: many tagged nodes + short segments => via3 IS road-reachable"
      + " (loop beelined when it should have routed = leg-routing/detail defect);"
      + " all-null + long edges => genuine beeline.");
    for (int k = 0; k < Math.min(t.nodes.size(), 25); k++) {
      OsmPathElement nd = t.nodes.get(k);
      double lat = (nd.getILat() - 90000000) / 1000000.0;
      double lon = (nd.getILon() - 180000000) / 1000000.0;
      int seg = k > 0 ? (int) t.nodes.get(k - 1).calcDistance(nd) : 0;
      String way = nd.message == null ? "msg=null" : ("way=[" + nd.message.wayKeyValues + "]");
      System.out.printf("   idx=%d lat=%.5f lon=%.5f segLen=%dm %s%n", k, lat, lon, seg, way);
    }
  }

  /**
   * Segment-level analysis of the Wegendorf antenna: for each edge in the spur span,
   * report way tags + whether the edge's two endpoints are actually GRAPH-ADJACENT
   * (connected by a real OsmLink). A long edge whose endpoints are NOT adjacent is a
   * beeline (the route asserts a connection the graph does not have). For the offending
   * edge, dump the entry node's real link-neighbors so we can see the true road shape.
   */
  @Test
  public void analyzeWegendorfSegments() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    // Parameterizable for any regressed cell: -Dloop.region=GRENOBLE -Dloop.dir=270 -Dloop.radius=8000
    // -Dloop.profile=gravel -Droundtrip.beeline.disable=true (to see the PRE-fix loop the fix excised).
    LoopTestRegion region = LoopTestRegion.valueOf(System.getProperty("loop.region", "URBAN_BERLIN"));
    int dir = Integer.getInteger("loop.dir", 90);
    int radius = Integer.getInteger("loop.radius", 15900);
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + System.getProperty("loop.profile", "gravel") + ".brf");
    if (!profileFile.exists()) { System.out.println("ANALYZE: profile missing"); return; }
    System.out.printf("ANALYZE region=%s dir=%d radius=%d beelineDisabled=%s%n",
      region, dir, radius, System.getProperty("roundtrip.beeline.disable", "false"));

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from";
    start.ilon = region.ilon;
    start.ilat = region.ilat;
    wplist.add(start);

    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = dir;
    rctx.roundTripDistance = radius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISO_GREEDY;
    rctx.roundTripStrictQuality = false;

    File tmp = File.createTempFile("wegseg", "");
    RoutingEngine re = new RoutingEngine(
      tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    OsmTrack track = re.getFoundTrack();
    if (track == null) track = re.getLastRejectedTrack();
    if (track == null) { System.out.println("ANALYZE: no track"); return; }
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();

    // locate the longest single null-way edge inside a spur (the beeline), reusing the scan
    double[] cum = new double[n];
    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    int spurI = -1, spurJ = -1, beelineK = -1, beelineLen = 0;
    int i = 0;
    while (i < n) {
      int bestJ = -1; OsmPathElement a = nodes.get(i);
      for (int j = i + 1; j < n; j++) {
        double gap = cum[j] - cum[i];
        if (gap > SPUR_MAX_ARC_GAP) break;
        if (gap >= SPUR_MIN_ARC_GAP && a.calcDistance(nodes.get(j)) <= SPUR_EPS_METERS) bestJ = j;
      }
      if (bestJ >= 0) {
        for (int k = i + 1; k <= bestJ; k++) {
          OsmPathElement c = nodes.get(k);
          String tags = (c.message != null) ? c.message.wayKeyValues : null;
          if ((tags == null || tags.isEmpty())) {
            int seg = nodes.get(k - 1).calcDistance(c);
            if (seg > beelineLen) { beelineLen = seg; beelineK = k; spurI = i; spurJ = bestJ; }
          }
        }
        i = bestJ;
      } else i++;
    }
    if (beelineK < 0) { System.out.println("ANALYZE: no null-edge-in-spur found this run"); return; }

    System.out.printf("ANALYZE: spur idx[%d..%d], longest null edge = idx[%d->%d] len=%dm%n",
      spurI, spurJ, beelineK - 1, beelineK, beelineLen);

    // Separate point-to-point engine for graph queries: a P2P doRun builds expctxWay
    // (round-trip mode delegates to children, so the RT engine's parent context is null),
    // then resetCache(true) revives a live nodesCache that survives for adjacency walks.
    OsmPathElement entryN = nodes.get(beelineK - 1), tipN = nodes.get(beelineK);
    OsmNodeNamed gA = new OsmNodeNamed(); gA.name = "a"; gA.ilon = entryN.getILon(); gA.ilat = entryN.getILat();
    OsmNodeNamed gB = new OsmNodeNamed(); gB.name = "b"; gB.ilon = tipN.getILon(); gB.ilat = tipN.getILat();
    List<OsmNodeNamed> gwp = new ArrayList<>(); gwp.add(gA); gwp.add(gB);
    RoutingContext gctx = new RoutingContext();
    gctx.localFunction = profileFile.getAbsolutePath();
    File gtmp = File.createTempFile("weggraph", "");
    RoutingEngine g = new RoutingEngine(gtmp.getAbsolutePath(), gtmp.getAbsolutePath(), segDir, gwp, gctx, 0);
    g.doRun(0);
    OsmTrack p2p = g.getFoundTrack();
    System.out.printf("P2P entry->tip: %s%n", p2p == null ? "NO ROUTE (island)"
      : ("routed " + p2p.distance + "m vs straight " + (int) CheapRulerDist(gA.ilon, gA.ilat, gB.ilon, gB.ilat) + "m"));

    System.out.println("=== spur span edges: tags + (for NULL edges) P2P straight-vs-road classification ===");
    System.out.println("    rule: a NULL edge whose straight length < shortest real-road route between its");
    System.out.println("          endpoints (or whose P2P beelines) is a true beeline, not an undetailed road.");
    for (int k = spurI + 1; k <= spurJ; k++) {
      OsmPathElement A = nodes.get(k - 1), B = nodes.get(k);
      int seg = A.calcDistance(B);
      String tags = (B.message != null) ? B.message.wayKeyValues : null;
      boolean nullWay = tags == null || tags.isEmpty();
      String mark = (k == beelineK) ? "  <== LONGEST NULL EDGE" : "";
      if (!nullWay) {
        System.out.printf("  idx[%d->%d] %dm REAL-ROAD tags=[%s]%s%n", k - 1, k, seg, tags, mark);
      } else {
        String cls = classifyNullEdgeP2P(segDir, profileFile, A, B, seg);
        System.out.printf("  idx[%d->%d] %dm NULL %s%s%n", k - 1, k, seg, cls, mark);
      }
    }

    // DEFINITIVE test for the 811m edge: route entry->tip but FORBID the straight edge's
    // own corridor — if a real road still connects them, the straight edge was redundant/
    // beeline; we already saw P2P detours 963m via junction 1289 (north) which the straight
    // edge skips. Print the P2P path's max perpendicular deviation from the straight chord:
    // a real road following the chord deviates little; a detour-via-other-junction deviates far.
    reportCorridorDeviation(segDir, profileFile, entryN, tipN);
  }

  /**
   * Classify a NULL-way track edge by routing its endpoints point-to-point on the same
   * profile. The undetailed expansion strips tags from BOTH real roads and beeline seams,
   * so we re-derive ground truth from a clean P2P route:
   *   - no route / P2P itself beelines  → true beeline (no real way connects A,B);
   *   - routed dist >> straight (edge)  → beeline (the straight edge skips the real road's detour);
   *   - routed dist ≈ straight          → an undetailed REAL road.
   */
  /**
   * Render the baseline loop to a self-contained Leaflet HTML, highlighting every span the
   * backstop WOULD excise (over-cap near-return), colour-coded by the corridor-deviation
   * verdict so the user can visually verify: RED = classified beeline (would-remove looks
   * correct), ORANGE = real winding/sparse road (false positive — must NOT remove).
   * Params: -Dloop.region=GRENOBLE -Dloop.dir=270 -Dloop.radius=8000 -Dloop.profile=gravel
   *         -Dloop.html=/abs/path/out.html
   */
  @Test
  public void dumpRouteHtml() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestRegion region = LoopTestRegion.valueOf(System.getProperty("loop.region", "GRENOBLE"));
    int dir = Integer.getInteger("loop.dir", 270);
    int radius = Integer.getInteger("loop.radius", 8000);
    String profile = System.getProperty("loop.profile", "gravel");
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + profile + ".brf");
    if (!profileFile.exists()) { System.out.println("HTML: profile missing"); return; }

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from"; start.ilon = region.ilon; start.ilat = region.ilat;
    wplist.add(start);
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = dir; rctx.roundTripDistance = radius;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISO_GREEDY; rctx.roundTripStrictQuality = false;
    // Route TWICE on the same case: backstop OFF (baseline loop, beeline intact) and ON
    // (excised loop). The diff = exactly what the backstop removed — the ground truth, not
    // a replication. Highlight those removed spans on the baseline loop, colour-coded by
    // the corridor verdict so the user can verify each is a beeline or a real road.
    System.clearProperty("roundtrip.beeline.enable");
    OsmTrack track = routeLoop(segDir, profileFile, region, dir, radius); // baseline
    if (track == null) { System.out.println("HTML: no baseline track"); return; }
    System.setProperty("roundtrip.beeline.enable", "true");
    OsmTrack excised = routeLoop(segDir, profileFile, region, dir, radius); // backstop on
    System.clearProperty("roundtrip.beeline.enable");
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();

    // Build the set of node positions surviving in the excised loop; contiguous runs of
    // baseline nodes absent from it are what the backstop removed.
    java.util.Set<Long> survived = new java.util.HashSet<>();
    if (excised != null) for (OsmPathElement nd : excised.nodes) survived.add(nd.getIdFromPos());
    List<int[]> spans = new ArrayList<>(); // {startIdx, endIdx} removed runs (baseline indices)
    int runStart = -1;
    for (int i = 0; i < n; i++) {
      boolean removed = excised != null && !survived.contains(nodes.get(i).getIdFromPos());
      if (removed && runStart < 0) runStart = i;
      if (!removed && runStart >= 0) {
        if (i - runStart >= 1) spans.add(new int[]{Math.max(0, runStart - 1), i}); // include anchors
        runStart = -1;
      }
    }
    if (runStart >= 0) spans.add(new int[]{Math.max(0, runStart - 1), n - 1});
    System.out.printf("HTML: baseline %dm (%d nodes) -> excised %dm (%d nodes); %d removed span(s)%n",
      track.distance, n, excised == null ? 0 : excised.distance, excised == null ? 0 : excised.nodes.size(), spans.size());

    StringBuilder route = new StringBuilder();
    for (int i = 0; i < n; i++) {
      OsmPathElement nd = nodes.get(i);
      route.append(String.format(Locale.US, "[%.6f,%.6f],",
        (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6));
    }
    StringBuilder spansJs = new StringBuilder();
    System.out.println("HTML: " + spans.size() + " over-cap excision-candidate span(s) found");
    for (int[] sp : spans) {
      int mi = sp[0], i = sp[1];
      // Scan ALL edges >300m in the span (as the backstop's spanContainsBeeline does) and
      // pick the worst (max corridor deviation / no-route) — that is what triggers excision.
      int bestK = -1, bestLen = 0, worstDev = -1, worstRoad = 0, worstK = -1; boolean anyNoRoute = false;
      for (int k = mi + 1; k <= i; k++) {
        int seg = nodes.get(k - 1).calcDistance(nodes.get(k));
        if (seg > bestLen) { bestLen = seg; bestK = k; }
        if (seg >= 300) {
          int[] c = corridorOf(segDir, profileFile, nodes.get(k - 1), nodes.get(k));
          if (c[2] == 0) { anyNoRoute = true; if (worstDev < 999999) { worstDev = 999999; worstK = k; worstRoad = 0; } }
          else if (c[1] > worstDev) { worstDev = c[1]; worstK = k; worstRoad = c[0]; }
        }
      }
      int trigK = worstK >= 0 ? worstK : bestK;
      OsmPathElement A = nodes.get(trigK - 1), B = nodes.get(trigK);
      int trigLen = A.calcDistance(B);
      String tags = (B.message != null && B.message.wayKeyValues != null) ? B.message.wayKeyValues : "null";
      boolean beeline = anyNoRoute || worstDev > 120;
      int[] corr = new int[]{worstRoad, worstDev < 0 ? 0 : worstDev, anyNoRoute ? 0 : 1};
      bestLen = trigLen;
      String verdict = beeline ? "BEELINE (would-remove looks correct)" : "REAL ROAD (false positive)";
      StringBuilder coords = new StringBuilder();
      for (int k = mi; k <= i; k++) {
        OsmPathElement nd = nodes.get(k);
        coords.append(String.format(Locale.US, "[%.6f,%.6f],", (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6));
      }
      int loopDist = 0; for (int j = mi + 1; j <= i; j++) loopDist += nodes.get(j).calcDistance(nodes.get(j - 1));
      spansJs.append(String.format(Locale.US, "{coords:[%s],beeline:%b,popup:%s},",
        coords, beeline,
        jsStr(String.format(Locale.US, "span idx[%d..%d] loopDist=%dm | longest edge %dm tags=[%s] | corridor: road=%dm dev=%dm hasRoute=%b | %s",
          mi, i, loopDist, bestLen, tags, corr[0], corr[1], corr[2] == 1, verdict))));
      System.out.printf("  span idx[%d..%d] loopDist=%dm longestEdge=%dm tags=[%s] corridor road=%dm dev=%dm -> %s%n",
        mi, i, loopDist, bestLen, tags, corr[0], corr[1], verdict);
    }

    double clat = (nodes.get(0).getILat() - 90000000) / 1e6, clon = (nodes.get(0).getILon() - 180000000) / 1e6;
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>" + region + " " + dir + " " + profile + "</title>"
      + "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>"
      + "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
      + "<style>#m{height:100vh}#lg{position:absolute;z-index:1000;top:8px;right:8px;background:#fff;padding:8px;font:13px sans-serif;box-shadow:0 1px 4px rgba(0,0,0,.3)}</style></head><body>"
      + "<div id='lg'><b>" + region + " " + dir + "deg " + profile + " ~" + (radius * 628 / 100000) + "km</b><br>"
      + "<span style='color:#1565c0'>route</span> &nbsp; <span style='color:red'>beeline (remove ok)</span> &nbsp; <span style='color:orange'>real road (false positive)</span><br>"
      + "click a highlighted span for details</div><div id='m'></div><script>"
      + "var map=L.map('m').setView([" + clat + "," + clon + "],12);"
      + "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'OSM'}).addTo(map);"
      + "var route=[" + route + "];L.polyline(route,{color:'#1565c0',weight:3,opacity:.7}).addTo(map);"
      + "L.marker(route[0]).addTo(map).bindPopup('start/end');"
      + "var spans=[" + spansJs + "];spans.forEach(function(s){"
      + "var pl=L.polyline(s.coords,{color:s.beeline?'red':'orange',weight:7,opacity:.9}).addTo(map);pl.bindPopup(s.popup);});"
      + "map.fitBounds(L.polyline(route).getBounds());"
      + "</script></body></html>";
    String outPath = System.getProperty("loop.html",
      new File(projectDir, "brouter-core/build/route-" + region + "-" + dir + "-" + profile + ".html").getAbsolutePath());
    Files.write(new File(outPath).toPath(), html.getBytes("UTF-8"));
    System.out.println("HTML WRITTEN: " + outPath);
  }

  /**
   * Plain point-to-point probe (no round-trip / no anti-reuse) through a list of waypoints.
   * Use to isolate anti-reuse vs placement: route the planner's exact via chain plainly; if
   * it avoids Ettingen, the loop's detour is the round-trip's anti-reuse/return, not placement.
   * -Dloop.waypoints="lat,lon;lat,lon;..."  (also reports gravel/track surface share).
   */
  @Test
  public void p2pProbe() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestRegion region = LoopTestRegion.valueOf(System.getProperty("loop.region", "BASEL"));
    LoopTestSegments.ensureRegion(segDir, region);
    String profilePath = System.getProperty("loop.profilepath", "");
    File profileFile = profilePath.isEmpty()
      ? new File(projectDir, "misc/profiles2/" + System.getProperty("loop.profile", "gravel") + ".brf")
      : new File(profilePath);
    if (!profileFile.exists()) { System.out.println("P2P: profile missing: " + profileFile); return; }
    String wps = System.getProperty("loop.waypoints", "");
    if (wps.isEmpty()) { System.out.println("P2P: set -Dloop.waypoints=lat,lon;lat,lon"); return; }

    List<OsmNodeNamed> wp = new ArrayList<>();
    int idx = 0;
    for (String p : wps.split(";")) {
      String[] ll = p.trim().split(",");
      OsmNodeNamed nd = new OsmNodeNamed();
      nd.name = (idx == 0 ? "from" : "to" + idx);
      nd.ilat = 90000000 + (int) (Double.parseDouble(ll[0]) * 1e6 + 0.5);
      nd.ilon = 180000000 + (int) (Double.parseDouble(ll[1]) * 1e6 + 0.5);
      wp.add(nd); idx++;
    }
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profileFile.getAbsolutePath();
    File tmp = File.createTempFile("p2p", "");
    RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc, 0);
    e.doRun(0);
    OsmTrack t = e.getFoundTrack();
    if (t == null) { System.out.println("P2P: NO ROUTE. error=" + e.getErrorMessage()); return; }
    double minLat = 999, maxLat = -999, minLon = 999, maxLon = -999;
    int trackM = 0, gravelM = 0;
    int ettLon = 180000000 + (int) (7.61 * 1e6), ettLat = 90000000 + (int) (47.47 * 1e6);
    int furLon = 180000000 + (int) (7.53067 * 1e6), furLat = 90000000 + (int) (47.46769 * 1e6);
    int minToEtt = Integer.MAX_VALUE, minToFur = Integer.MAX_VALUE;
    for (int i = 0; i < t.nodes.size(); i++) {
      OsmPathElement nd = t.nodes.get(i);
      double lat = (nd.getILat() - 90000000) / 1e6, lon = (nd.getILon() - 180000000) / 1e6;
      minLat = Math.min(minLat, lat); maxLat = Math.max(maxLat, lat);
      minLon = Math.min(minLon, lon); maxLon = Math.max(maxLon, lon);
      minToEtt = Math.min(minToEtt, (int) CheapRulerDist(nd.getILon(), nd.getILat(), ettLon, ettLat));
      minToFur = Math.min(minToFur, (int) CheapRulerDist(nd.getILon(), nd.getILat(), furLon, furLat));
      if (i > 0) {
        int seg = t.nodes.get(i - 1).calcDistance(nd);
        String tg = nd.message != null ? nd.message.wayKeyValues : null;
        if (tg != null && (tg.contains("highway=track") || tg.contains("surface=gravel")
            || tg.contains("surface=ground") || tg.contains("surface=dirt") || tg.contains("tracktype"))) gravelM += seg;
        if (tg != null && tg.contains("highway=track")) trackM += seg;
      }
    }
    System.out.printf("P2P: profile=%s dist=%dm bbox lat[%.4f..%.4f] lon[%.4f..%.4f]%n",
      profileFile.getName(), t.distance, minLat, maxLat, minLon, maxLon);
    System.out.printf("P2P: minDistToEttingen=%dm minDistToFürstenstein=%dm | gravelish=%dm (%.0f%%) track=%dm%n",
      minToEtt, minToFur, gravelM, 100.0 * gravelM / Math.max(1, t.distance), trackM);

    // Optional HTML: colour gravel/track segments green, paved/other blue; mark start + ruin.
    String outHtml = System.getProperty("loop.html", "");
    if (!outHtml.isEmpty()) {
      StringBuilder segs = new StringBuilder();
      int rs = 0; boolean curGravel = false;
      for (int i = 1; i < t.nodes.size(); i++) {
        OsmPathElement nd = t.nodes.get(i);
        String tg = nd.message != null ? nd.message.wayKeyValues : null;
        boolean g = tg != null && (tg.contains("highway=track") || tg.contains("surface=gravel")
          || tg.contains("surface=ground") || tg.contains("surface=dirt") || tg.contains("tracktype"));
        if (i == 1) { rs = 0; curGravel = g; }
        if (g != curGravel) {
          appendRun(segs, t.nodes, rs, i - 1, curGravel);
          rs = i - 1; curGravel = g;
        }
      }
      appendRun(segs, t.nodes, rs, t.nodes.size() - 1, curGravel);
      double clat = (t.nodes.get(0).getILat() - 90000000) / 1e6, clon = (t.nodes.get(0).getILon() - 180000000) / 1e6;
      String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Fürstenstein loop " + profileFile.getName() + "</title>"
        + "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>"
        + "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
        + "<style>#m{height:100vh}#lg{position:absolute;z-index:1000;top:8px;right:8px;background:#fff;padding:8px;font:13px sans-serif;box-shadow:0 1px 4px rgba(0,0,0,.3)}</style></head><body>"
        + "<div id='lg'><b>" + profileFile.getName() + "</b><br>" + (t.distance / 1000) + "." + (t.distance % 1000 / 100) + "km, "
        + String.format(Locale.US, "%.0f%%", 100.0 * gravelM / Math.max(1, t.distance)) + " gravel/track<br>"
        + "<span style='color:green'>gravel/track</span> &nbsp; <span style='color:#1565c0'>paved/other</span></div>"
        + "<div id='m'></div><script>var map=L.map('m').setView([" + clat + "," + clon + "],13);"
        + "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'OSM'}).addTo(map);"
        + segs
        + "L.marker([" + clat + "," + clon + "]).addTo(map).bindPopup('start/end (Basel)');"
        + "L.circleMarker([47.46769,7.53067],{radius:8,color:'#b8860b',fillColor:'#ffd700',fillOpacity:1}).addTo(map).bindPopup('Burgruine Fürstenstein');"
        + "var all=L.featureGroup();map.eachLayer(function(l){if(l.getBounds)all.addLayer(l);});map.fitBounds(all.getBounds());"
        + "</script></body></html>";
      Files.write(new File(outHtml).toPath(), html.getBytes("UTF-8"));
      System.out.println("P2P HTML WRITTEN: " + outHtml);
    }
  }

  private static void appendRun(StringBuilder sb, List<OsmPathElement> nodes, int from, int to, boolean gravel) {
    if (to <= from) return;
    StringBuilder c = new StringBuilder();
    for (int k = from; k <= to; k++) {
      OsmPathElement nd = nodes.get(k);
      c.append(String.format(Locale.US, "[%.6f,%.6f],", (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6));
    }
    sb.append("L.polyline([").append(c).append("],{color:'").append(gravel ? "green" : "#1565c0")
      .append("',weight:5,opacity:.85}).addTo(map);");
  }

  /** Route one round-trip loop for the region/dir/radius; foundTrack or lastRejected, else null. */
  private static OsmTrack routeLoop(File segDir, File profileFile, LoopTestRegion region, int dir, int radius) {
    return routeLoop(segDir, profileFile, region, dir, radius, RoundTripAlgorithm.ISO_GREEDY);
  }

  private static OsmTrack routeLoop(File segDir, File profileFile, LoopTestRegion region, int dir, int radius,
                                    RoundTripAlgorithm algo) {
    try {
      List<OsmNodeNamed> wp = new ArrayList<>();
      OsmNodeNamed s = new OsmNodeNamed(); s.name = "from"; s.ilon = region.ilon; s.ilat = region.ilat;
      wp.add(s);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profileFile.getAbsolutePath();
      rc.startDirection = dir; rc.roundTripDistance = radius;
      rc.roundTripAlgorithm = algo; rc.roundTripStrictQuality = false;
      File tmp = File.createTempFile("loop", "");
      RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc,
        RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
      e.doRun(0);
      OsmTrack t = e.getFoundTrack();
      return t != null ? t : e.getLastRejectedTrack();
    } catch (Exception ex) { return null; }
  }

  /**
   * ARCHITECTURE PROBE for the elastic anti-reuse fix (no code under test yet).
   *
   * <p>Routes the Basel loop, runs the {@link LoopQualityMetrics#nearRevisitSpans}
   * detector at the elastic-fix thresholds (10 km cap, unlike computeSpurInfo's 6 km),
   * and for each detected out-and-back / small-sub-loop span reports whether a committed
   * via (leg boundary) lies <em>strictly inside</em> it:
   * <ul>
   *   <li><b>no via inside</b> → the detour is contained in ONE shipped leg → the per-leg
   *       elastic re-route hook (accepted sub-leg / return / force-close) catches it;</li>
   *   <li><b>a via inside</b> → the detour straddles legs → per-leg detection never fires on
   *       any single leg → the fix must detect on the assembled loop and map the span back to
   *       the leg(s) it overlaps.</li>
   * </ul>
   * This decides the architecture BEFORE touching RoutingContext/OsmPath/planner, and the
   * same output is the before/after baseline for the fix itself.
   * Run: {@code -Dloop.region=BASEL -Dloop.dir=180 -Dloop.radius=4800 -Dloop.profile=gravel}.
   */
  @Test
  public void probeElasticArchitecture() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestRegion region = LoopTestRegion.valueOf(System.getProperty("loop.region", "BASEL"));
    int dir = Integer.getInteger("loop.dir", 180);
    int radius = Integer.getInteger("loop.radius", 4800);
    LoopTestSegments.ensureRegion(segDir, region);
    File profileFile = new File(projectDir, "misc/profiles2/" + System.getProperty("loop.profile", "gravel") + ".brf");
    if (!profileFile.exists()) { System.out.println("ARCH: profile missing: " + profileFile); return; }

    RoundTripAlgorithm mainAlgo = RoundTripAlgorithm.valueOf(System.getProperty("loop.algo", "ISO_GREEDY"));
    OsmTrack track = routeLoop(segDir, profileFile, region, dir, radius, mainAlgo);
    if (track == null) { System.out.println("ARCH: no track"); return; }
    List<OsmPathElement> nodes = track.nodes;
    int n = nodes.size();
    System.out.printf("ARCH region=%s dir=%d radius=%d nodes=%d dist=%dm%n", region, dir, radius, n, track.distance);

    // Clean-alternative test (d2bbea4c logic + advisor's discriminating run): does the
    // OTHER algorithm produce a teardrop-free loop, and crucially is it GATE-ACCEPTED and
    // what is its RCS vs the teardrop's? The RCS gap sizes the near-revisit penalty needed
    // to flip AUTO's pick; the gate verdict says win (accepted) vs tradeoff (short-but-clean).
    double reqDist = 2 * Math.PI * radius;
    String profName = System.getProperty("loop.profile", "gravel");
    System.out.printf("ARCH reqDist=%.0fm (2*pi*%d)%n", reqDist, radius);
    System.out.printf("ARCH teardrop penalty in effect: %.3f (loop.teardroppenalty)%n",
      RouteChoiceScore.SHAPE_PENALTY_PER_TEARDROP);
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.ISO_GREEDY, RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.AUTO}) {
      OsmTrack t = routeLoop(segDir, profileFile, region, dir, radius, algo);
      if (t == null || t.nodes == null) { System.out.printf("ARCH alt[%s]: no track%n", algo); continue; }
      List<int[]> sp = LoopQualityMetrics.nearRevisitSpans(t.nodes, 60, 600, 10000);
      double[] c2 = new double[t.nodes.size()];
      for (int k = 1; k < t.nodes.size(); k++) c2[k] = c2[k - 1] + t.nodes.get(k - 1).calcDistance(t.nodes.get(k));
      int worst = 0;
      for (int[] s2 : sp) worst = Math.max(worst, (int) Math.round(c2[s2[1]] - c2[s2[0]]));
      RoundTripQualityResult v = RoundTripQualityGate.evaluate(t, reqDist, profName, false);
      LoopQualityMetrics m = LoopQualityMetrics.compute(t, (int) reqDist, dir);
      RouteChoiceScore.Verdict rcs = RouteChoiceScore.score(t, reqDist, profName, v, dir);
      String gate = v.isAccepted() ? ("ACCEPT/" + v.getShape())
        : ("REJECT(" + v.getRejectionTier() + ":" + v.getRejectionReason() + ")");
      System.out.printf("ARCH alt[%-10s] dist=%dm distR=%.3f nearRevisit=%d worstArc=%dm xings=%d | gate=%s | RCS score=%.4f quality=%.4f%n",
        algo, t.distance, m.getDistanceRatio(), sp.size(), worst,
        RoundTripQualityGate.countSelfIntersections(t), gate, rcs.score(), rcs.qualityScore());
      writeLoopHtml("/tmp/" + region.name().toLowerCase() + "-" + algo + ".html", t, algo.toString());
      // Does this variant pass through the gravel the user wanted (Biel-Benken/Neuwiller)
      // and how close to the Ettingen detour? (Basel-specific landmarks.)
      int bbIlon = (int) ((7.5294 + 180) * 1e6), bbIlat = (int) ((47.4869 + 90) * 1e6);
      int nwIlon = (int) ((7.4897 + 180) * 1e6), nwIlat = (int) ((47.4842 + 90) * 1e6);
      int etIlon = (int) ((7.5456 + 180) * 1e6), etIlat = (int) ((47.4817 + 90) * 1e6);
      double dBB = 1e9, dNW = 1e9, dET = 1e9;
      for (OsmPathElement p : t.nodes) {
        dBB = Math.min(dBB, btools.util.CheapRuler.distance(p.getILon(), p.getILat(), bbIlon, bbIlat));
        dNW = Math.min(dNW, btools.util.CheapRuler.distance(p.getILon(), p.getILat(), nwIlon, nwIlat));
        dET = Math.min(dET, btools.util.CheapRuler.distance(p.getILon(), p.getILat(), etIlon, etIlat));
      }
      System.out.printf("ARCH alt[%-10s] minDist: BielBenken=%dm Neuwiller=%dm Ettingen=%dm%n",
        algo, (int) dBB, (int) dNW, (int) dET);
    }

    // Committed via node indices = leg boundaries.
    List<int[]> vias = new ArrayList<>(); // {nodeIndex, waypointIndex}
    List<MatchedWaypoint> mwps = track.matchedWaypoints;
    if (mwps != null) {
      for (int w = 0; w < mwps.size(); w++) {
        MatchedWaypoint mw = mwps.get(w);
        if (mw.crosspoint == null) continue;
        int idx = nearestNodeIndex(nodes, mw.crosspoint.getILon(), mw.crosspoint.getILat());
        vias.add(new int[]{idx, w});
        System.out.printf("  via w=%d nodeIdx=%d name=%s%n", w, idx, mw.name);
      }
    }

    double eps = 60, minArc = 600, maxArc = 10000;
    List<int[]> spans = LoopQualityMetrics.nearRevisitSpans(nodes, eps, minArc, maxArc);
    System.out.printf("ARCH near-revisit spans (eps=%.0f minArc=%.0f maxArc=%.0f): %d%n",
      eps, minArc, maxArc, spans.size());

    // What does the planner's OWN self-intersection machinery see on this loop?
    // RouteChoiceScore penalises RoundTripQualityGate.countSelfIntersections (transverse
    // X-crossings). If this is 0 while a near-revisit span exists, the teardrop is INVISIBLE
    // to the scorer -> that's why it shipped despite d2bbea4c.
    System.out.printf("ARCH gate: countSelfIntersections=%d countHairpinTurns=%d (MAX_SELF=%d)%n",
      RoundTripQualityGate.countSelfIntersections(track),
      RoundTripQualityGate.countHairpinTurns(track),
      RoundTripQualityGate.MAX_SELF_INTERSECTIONS);

    double[] cum = new double[n];
    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));

    for (int s = 0; s < spans.size(); s++) {
      int i = spans.get(s)[0], j = spans.get(s)[1];
      int arc = (int) Math.round(cum[j] - cum[i]);
      int tip = i; double maxd = 0;
      OsmPathElement a = nodes.get(i);
      for (int k = i; k <= j; k++) { double d = a.calcDistance(nodes.get(k)); if (d > maxd) { maxd = d; tip = k; } }
      double tipLat = (nodes.get(tip).getILat() - 90000000) / 1e6;
      double tipLon = (nodes.get(tip).getILon() - 180000000) / 1e6;
      List<Integer> insideVias = new ArrayList<>();
      for (int[] v : vias) if (v[0] > i && v[0] < j) insideVias.add(v[1]);
      String verdict = insideVias.isEmpty()
        ? "WITHIN-LEG (per-leg hook catches it)"
        : "CROSS-LEG (vias " + insideVias + " inside -> needs whole-loop detection)";
      System.out.printf("  span#%d idx[%d..%d] arc=%dm tip(idx=%d lat=%.5f lon=%.5f reach=%dm) %s%n",
        s, i, j, arc, tip, tipLat, tipLon, (int) maxd, verdict);
    }
    // --- Deep dive on the longest span: is the out-and-back removable by softening
    // anti-reuse, or forced by via placement (no alternative road back)? ---
    if (!spans.isEmpty()) {
      int best = 0, bestArc = -1;
      for (int s = 0; s < spans.size(); s++) {
        int arc = (int) Math.round(cum[spans.get(s)[1]] - cum[spans.get(s)[0]]);
        if (arc > bestArc) { bestArc = arc; best = s; }
      }
      int i = spans.get(best)[0], j = spans.get(best)[1];
      int tip = i; double maxd = 0; OsmPathElement a = nodes.get(i);
      for (int k = i; k <= j; k++) { double d = a.calcDistance(nodes.get(k)); if (d > maxd) { maxd = d; tip = k; } }

      // (1) node-ID sharing: out half [i..tip] vs back half [tip..j]. High share = true retrace.
      java.util.Set<Long> outIds = new java.util.HashSet<>();
      for (int k = i; k <= tip; k++) outIds.add(nodes.get(k).getIdFromPos());
      int shared = 0, backN = 0;
      for (int k = tip; k <= j; k++) { backN++; if (outIds.contains(nodes.get(k).getIdFromPos())) shared++; }
      System.out.printf("  DEEP span[%d..%d] tip=%d: backNodes=%d sharedNodeIds=%d (%.0f%% true edge-retrace)%n",
        i, j, tip, backN, shared, backN > 0 ? 100.0 * shared / backN : 0);

      // (2) geometric proximity of back half to out half (catches parallel-corridor reuse
      // that edge-identity anti-reuse misses).
      int backNear = 0;
      for (int k = tip; k <= j; k++) {
        double mind = Double.MAX_VALUE;
        for (int q = i; q <= tip; q++) mind = Math.min(mind, nodes.get(k).calcDistance(nodes.get(q)));
        if (mind <= 60) backNear++;
      }
      System.out.printf("  DEEP back half within 60m of out half: %d/%d nodes (parallel-or-retrace corridor)%n", backNear, backN);

      // (3) alternative-return test: route tip(=via3)->start with NO anti-reuse (plain P2P).
      // If this natural return still hugs the out corridor, the out-and-back is PLACEMENT-forced
      // (softening anti-reuse cannot help); if it peels away, an alternative exists.
      OsmPathElement tipNode = nodes.get(tip);
      OsmTrack p2p = routeP2P(segDir, profileFile, tipNode.getILon(), tipNode.getILat(), region.ilon, region.ilat);
      if (p2p == null || p2p.nodes.isEmpty()) {
        System.out.println("  DEEP alt-return: no P2P route tip->start");
      } else {
        int near = 0;
        for (OsmPathElement nd : p2p.nodes) {
          double mind = Double.MAX_VALUE;
          for (int q = i; q <= tip; q++) mind = Math.min(mind, nd.calcDistance(nodes.get(q)));
          if (mind <= 60) near++;
        }
        System.out.printf("  DEEP alt-return (tip->start, NO anti-reuse): dist=%dm nodes=%d, %d (%.0f%%) within 60m of out half%n",
          p2p.distance, p2p.nodes.size(), near, 100.0 * near / p2p.nodes.size());
        System.out.println("  DEEP  high%% retrace => PLACEMENT-forced (softening won't help); low%% => alternative exists (softening/parallel-return can help)");
      }

      // --- HTML viz: full loop (blue), near-revisit span / teardrop (red), vias (green),
      // tip=Ettingen (pin), start (black), reference gravel towns Biel-Benken/Neuwiller (orange) ---
      StringBuilder pts = new StringBuilder();
      for (int k = 0; k < n; k++) {
        if (k > 0) pts.append(',');
        pts.append('[').append((nodes.get(k).getILat() - 90000000) / 1e6)
           .append(',').append((nodes.get(k).getILon() - 180000000) / 1e6).append(']');
      }
      StringBuilder red = new StringBuilder();
      for (int k = i; k <= j; k++) {
        if (k > i) red.append(',');
        red.append('[').append((nodes.get(k).getILat() - 90000000) / 1e6)
           .append(',').append((nodes.get(k).getILon() - 180000000) / 1e6).append(']');
      }
      StringBuilder vmarks = new StringBuilder();
      for (int[] v : vias) {
        OsmPathElement nd = nodes.get(Math.max(0, Math.min(n - 1, v[0])));
        double la = (nd.getILat() - 90000000) / 1e6, lo = (nd.getILon() - 180000000) / 1e6;
        vmarks.append("L.circleMarker([").append(la).append(',').append(lo)
              .append("],{radius:6,color:'#0a0'}).addTo(map).bindTooltip('w").append(v[1]).append("');");
      }
      double tLat = (nodes.get(tip).getILat() - 90000000) / 1e6, tLon = (nodes.get(tip).getILon() - 180000000) / 1e6;
      double sLat = (region.ilat - 90000000) / 1e6, sLon = (region.ilon - 180000000) / 1e6;
      String html = "<!doctype html><html><head><meta charset=utf-8>"
        + "<link rel=stylesheet href='https://unpkg.com/leaflet/dist/leaflet.css'/>"
        + "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>"
        + "<style>#m{height:100vh}</style></head><body><div id=m></div><script>"
        + "var map=L.map('m');L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
        + "var loop=L.polyline([" + pts + "],{color:'#06c',weight:3}).addTo(map);"
        + "L.polyline([" + red + "],{color:'red',weight:6,opacity:0.85}).addTo(map).bindTooltip('near-revisit span / teardrop');"
        + vmarks
        + "L.marker([" + tLat + "," + tLon + "]).addTo(map).bindTooltip('tip=via3 (Ettingen)',{permanent:true});"
        + "L.circleMarker([" + sLat + "," + sLon + "],{radius:7,color:'#000'}).addTo(map).bindTooltip('start');"
        + "L.circleMarker([47.4869,7.5294],{radius:6,color:'#f80'}).addTo(map).bindTooltip('Biel-Benken');"
        + "L.circleMarker([47.4842,7.4897],{radius:6,color:'#f80'}).addTo(map).bindTooltip('Neuwiller');"
        + "map.fitBounds(loop.getBounds());</script></body></html>";
      String out = System.getProperty("loop.html", "/tmp/basel-teardrop.html");
      java.nio.file.Files.write(java.nio.file.Paths.get(out), html.getBytes());
      System.out.println("ARCH wrote HTML: " + out);
    } else {
      System.out.println("  (no near-revisit at these thresholds — try other params; Ettingen may not reproduce here)");
    }
  }

  /**
   * Corpus A/B for the teardrop penalty (offline AUTO simulation, d2bbea4c pattern).
   * Routes ISO_GREEDY + GREEDY for each (region, dir) cell, then REPLICATES the production
   * AUTO rule ({@code runAutoCandidateCompetition}: ship ISO unless its score &lt; 0.85, else
   * route GREEDY and pick the higher-scoring) analytically across a sweep of teardrop weights.
   * Run with {@code -Dloop.teardroppenalty=0} so the per-variant baseRCS is teardrop-free; the
   * sweep then subtracts {@code W * severity} itself. Reports, per weight: shipped-teardrop %,
   * how many cells flipped vs the W=0 baseline, and how many flips went to a worse-distance loop
   * (the Basel tradeoff — the failure mode to watch). {@code -Dloop.regions=A,B} overridable.
   */
  @Test
  public void corpusTeardropAB() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    String profName = System.getProperty("loop.profile", "gravel");
    File profileFile = new File(projectDir, "misc/profiles2/" + profName + ".brf");
    if (!profileFile.exists()) { System.out.println("AB: profile missing: " + profileFile); return; }
    String regionsCsv = System.getProperty("loop.regions", "BASEL,FREIBURG,GRENOBLE,ANNECY");
    String[] regionNames = regionsCsv.split(",");
    int radius = Integer.getInteger("loop.radius", 4800);
    double reqDist = 2 * Math.PI * radius;
    int[] dirs = {0, 45, 90, 135, 180, 225, 270, 315};
    double[] W = {0.0, 0.08, 0.10, 0.12, 0.15, 0.20};
    final double THRESH = 0.85; // RoutingEngine.CLEAR_ACCEPT_THRESHOLD
    if (RouteChoiceScore.SHAPE_PENALTY_PER_TEARDROP != 0.0)
      System.out.println("AB WARNING: run with -Dloop.teardroppenalty=0 so baseRCS is teardrop-free (current="
        + RouteChoiceScore.SHAPE_PENALTY_PER_TEARDROP + ")");

    int cells = 0;
    int[] teardropShip = new int[W.length];
    int[] flips = new int[W.length];
    int[] flipsWorse = new int[W.length];
    double[] distRSum = new double[W.length];
    double[] sevSum = new double[W.length];

    for (String rn : regionNames) {
      LoopTestRegion region;
      try { region = LoopTestRegion.valueOf(rn.trim()); } catch (Exception e) { System.out.println("AB bad region " + rn); continue; }
      try { LoopTestSegments.ensureRegion(segDir, region); } catch (Exception e) { System.out.println("AB ensureRegion failed " + rn + ": " + e); continue; }
      for (int dir : dirs) {
        double[] iso = variantRecord(segDir, profileFile, region, dir, radius, RoundTripAlgorithm.ISO_GREEDY, reqDist, profName);
        double[] grd = variantRecord(segDir, profileFile, region, dir, radius, RoundTripAlgorithm.GREEDY, reqDist, profName);
        if (iso == null && grd == null) continue;
        cells++;
        double[] pick0 = productionPick(iso, grd, 0.0, THRESH);
        for (int wi = 0; wi < W.length; wi++) {
          double[] pick = productionPick(iso, grd, W[wi], THRESH);
          if (pick == null) continue;
          if (pick[0] > 0) teardropShip[wi]++;
          distRSum[wi] += pick[1];
          sevSum[wi] += pick[0];
          if (pick != pick0) {
            flips[wi]++;
            if (pick0 != null && Math.abs(pick[1] - 1.0) > Math.abs(pick0[1] - 1.0) + 0.03) flipsWorse[wi]++;
          }
        }
        System.out.printf("AB cell %s dir=%-3d iso{sev=%.2f distR=%.2f rcs=%.3f} grd{%s}%n",
          region, dir, iso == null ? -1 : iso[0], iso == null ? -1 : iso[1], iso == null ? -1 : iso[2],
          grd == null ? "rejected" : String.format("sev=%.2f distR=%.2f rcs=%.3f", grd[0], grd[1], grd[2]));
        // Surface the worse-distance flips at the candidate weight for manual inspection
        // (the only place teardrop-detector over-firing could trade a good loop for a short one).
        double[] pT = productionPick(iso, grd, 0.12, THRESH);
        if (pT != null && pick0 != null && pT != pick0
            && Math.abs(pT[1] - 1.0) > Math.abs(pick0[1] - 1.0) + 0.03) {
          System.out.printf("AB WORSEFLIP %s dir=%d  before{pick=%s sev=%.2f distR=%.2f}  after{pick=%s sev=%.2f distR=%.2f}%n",
            region, dir, pick0 == iso ? "ISO" : "GRD", pick0[0], pick0[1],
            pT == iso ? "ISO" : "GRD", pT[0], pT[1]);
        }
      }
    }
    System.out.println("AB SUMMARY cells=" + cells + " regions=" + regionsCsv + " radius=" + radius);
    System.out.printf("%-8s %-14s %-8s %-15s %-10s %-10s%n", "weight", "teardropShip", "flips", "flipsWorseDist", "avgDistR", "avgShipSev");
    for (int wi = 0; wi < W.length; wi++) {
      System.out.printf("%-8.2f %-14s %-8d %-15d %-10.3f %-10.3f%n", W[wi],
        teardropShip[wi] + "/" + cells + "=" + (cells == 0 ? 0 : 100 * teardropShip[wi] / cells) + "%",
        flips[wi], flipsWorse[wi], cells == 0 ? 0 : distRSum[wi] / cells, cells == 0 ? 0 : sevSum[wi] / cells);
    }
  }

  /** {sev, distR, baseRCS} for an accepted variant, or null if no track / gate-rejected. */
  private static double[] variantRecord(File segDir, File profileFile, LoopTestRegion region, int dir, int radius,
                                        RoundTripAlgorithm algo, double reqDist, String profName) {
    OsmTrack t = routeLoop(segDir, profileFile, region, dir, radius, algo);
    if (t == null || t.nodes == null || t.nodes.size() < 4) return null;
    RoundTripQualityResult v = RoundTripQualityGate.evaluate(t, reqDist, profName, false);
    if (!v.isAccepted()) return null;
    double sev = teardropSeverityLocal(t);
    double distR = LoopQualityMetrics.compute(t, (int) reqDist, dir).getDistanceRatio();
    double baseRCS = RouteChoiceScore.score(t, reqDist, profName, v, dir).score(); // teardrop-free when penalty=0
    return new double[]{sev, distR, baseRCS};
  }

  /** Production AUTO rule: ship ISO unless weak (score &lt; thresh); if weak, GREEDY is routed and
   *  the higher penalised score wins. Mirrors RoutingEngine.runAutoCandidateCompetition. */
  private static double[] productionPick(double[] iso, double[] grd, double w, double thresh) {
    double isoScore = iso != null ? iso[2] - w * iso[0] : Double.NEGATIVE_INFINITY;
    boolean isoWeak = iso == null || isoScore < thresh;
    if (iso != null && !isoWeak) return iso; // strong ISO: GREEDY never routed
    double grdScore = grd != null ? grd[2] - w * grd[0] : Double.NEGATIVE_INFINITY;
    if (iso == null && grd == null) return null;
    if (iso == null) return grd;
    if (grd == null) return iso;
    return grdScore > isoScore ? grd : iso;
  }

  private static double teardropSeverityLocal(OsmTrack t) {
    List<OsmPathElement> nodes = t.nodes;
    int n = nodes.size();
    double[] cum = new double[n];
    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + nodes.get(i - 1).calcDistance(nodes.get(i));
    double total = cum[n - 1];
    double sev = 0;
    for (int[] s : LoopQualityMetrics.nearRevisitSpans(nodes, 60, 600, 10000)) {
      double arc = cum[s[1]] - cum[s[0]];
      if (total > 0 && arc > 0.85 * total) continue;
      sev += Math.min(1.0, arc / 5000.0);
    }
    return sev;
  }

  /** Bare loop (blue) + Basel reference markers (Biel-Benken/Neuwiller gravel, Ettingen). */
  private static void writeLoopHtml(String path, OsmTrack t, String label) {
    try {
      List<OsmPathElement> nd = t.nodes;
      StringBuilder pts = new StringBuilder();
      for (int k = 0; k < nd.size(); k++) {
        if (k > 0) pts.append(',');
        pts.append('[').append((nd.get(k).getILat() - 90000000) / 1e6)
           .append(',').append((nd.get(k).getILon() - 180000000) / 1e6).append(']');
      }
      // Highlight any near-revisit / teardrop spans in red so the rendered loop is
      // self-documenting (lets a glance confirm genuine teardrop vs benign near-pass).
      StringBuilder reds = new StringBuilder();
      for (int[] s : LoopQualityMetrics.nearRevisitSpans(nd, 60, 600, 10000)) {
        reds.append("L.polyline([");
        for (int k = s[0]; k <= s[1]; k++) {
          if (k > s[0]) reds.append(',');
          reds.append('[').append((nd.get(k).getILat() - 90000000) / 1e6)
              .append(',').append((nd.get(k).getILon() - 180000000) / 1e6).append(']');
        }
        reds.append("],{color:'red',weight:6,opacity:0.85}).addTo(map);");
      }
      String html = "<!doctype html><html><head><meta charset=utf-8>"
        + "<link rel=stylesheet href='https://unpkg.com/leaflet/dist/leaflet.css'/>"
        + "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>"
        + "<style>#m{height:100vh}</style></head><body><div id=m></div><script>"
        + "var map=L.map('m');L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
        + "var loop=L.polyline([" + pts + "],{color:'#06c',weight:3}).addTo(map);"
        + reds
        + "L.circleMarker([47.4869,7.5294],{radius:6,color:'#f80'}).addTo(map).bindTooltip('Biel-Benken',{permanent:true});"
        + "L.circleMarker([47.4842,7.4897],{radius:6,color:'#f80'}).addTo(map).bindTooltip('Neuwiller',{permanent:true});"
        + "L.marker([47.4817,7.5456]).addTo(map).bindTooltip('Ettingen',{permanent:true});"
        + "map.fitBounds(loop.getBounds());var t=L.control({position:'topright'});"
        + "t.onAdd=function(){var d=L.DomUtil.create('div');d.style.background='#fff';d.style.padding='4px';"
        + "d.innerHTML='" + label + " " + t.distance + "m (red=near-revisit)';return d;};t.addTo(map);</script></body></html>";
      java.nio.file.Files.write(java.nio.file.Paths.get(path), html.getBytes());
      System.out.println("ARCH wrote " + label + " HTML: " + path);
    } catch (Exception e) { System.out.println("ARCH writeLoopHtml failed: " + e); }
  }

  private static OsmTrack routeP2P(File segDir, File profileFile, int fromIlon, int fromIlat, int toIlon, int toIlat) {
    try {
      OsmNodeNamed a = new OsmNodeNamed(); a.name = "a"; a.ilon = fromIlon; a.ilat = fromIlat;
      OsmNodeNamed b = new OsmNodeNamed(); b.name = "b"; b.ilon = toIlon; b.ilat = toIlat;
      List<OsmNodeNamed> wp = new ArrayList<>(); wp.add(a); wp.add(b);
      RoutingContext rc = new RoutingContext(); rc.localFunction = profileFile.getAbsolutePath();
      File tmp = File.createTempFile("p2p", "");
      RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc, 0);
      e.doRun(0);
      return e.getFoundTrack();
    } catch (Exception ex) { return null; }
  }

  /** {roadDist, maxDevFromChord, hasRoute(1/0)} for the P2P route between A and B. */
  private static int[] corridorOf(File segDir, File profileFile, OsmPathElement A, OsmPathElement B) {
    try {
      OsmNodeNamed a = new OsmNodeNamed(); a.name = "a"; a.ilon = A.getILon(); a.ilat = A.getILat();
      OsmNodeNamed b = new OsmNodeNamed(); b.name = "b"; b.ilon = B.getILon(); b.ilat = B.getILat();
      List<OsmNodeNamed> wp = new ArrayList<>(); wp.add(a); wp.add(b);
      RoutingContext rc = new RoutingContext(); rc.localFunction = profileFile.getAbsolutePath();
      File tmp = File.createTempFile("corr", "");
      RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc, 0);
      e.doRun(0);
      OsmTrack t = e.getFoundTrack();
      if (t == null) return new int[]{0, 0, 0};
      double maxDev = 0;
      for (OsmPathElement nd : t.nodes) {
        double dev = perpDistMeters(nd.getILon(), nd.getILat(), A.getILon(), A.getILat(), B.getILon(), B.getILat());
        if (dev > maxDev) maxDev = dev;
      }
      return new int[]{t.distance, (int) maxDev, 1};
    } catch (Exception ex) { return new int[]{0, 0, 0}; }
  }

  private static String jsStr(String s) {
    return "'" + s.replace("\\", "\\\\").replace("'", "\\'") + "'";
  }

  /**
   * Route entry->tip P2P and report the real road's max perpendicular deviation from the
   * straight edge's chord. A real road that the straight edge approximates deviates little;
   * a real road that detours via a different junction (so the straight edge is a beeline
   * across the gap) deviates far. Threshold-free, distance-confound-free corridor test.
   */
  private static void reportCorridorDeviation(File segDir, File profileFile,
                                              OsmPathElement entry, OsmPathElement tip) {
    try {
      OsmNodeNamed a = new OsmNodeNamed(); a.name = "a"; a.ilon = entry.getILon(); a.ilat = entry.getILat();
      OsmNodeNamed b = new OsmNodeNamed(); b.name = "b"; b.ilon = tip.getILon(); b.ilat = tip.getILat();
      List<OsmNodeNamed> wp = new ArrayList<>(); wp.add(a); wp.add(b);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profileFile.getAbsolutePath();
      File tmp = File.createTempFile("corr", "");
      RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc, 0);
      e.doRun(0);
      OsmTrack t = e.getFoundTrack();
      System.out.println();
      if (t == null) { System.out.println("CORRIDOR: no P2P route (island) => beeline"); return; }
      double maxDev = 0; double devLat = 0, devLon = 0;
      for (OsmPathElement nd : t.nodes) {
        double dev = perpDistMeters(nd.getILon(), nd.getILat(),
          entry.getILon(), entry.getILat(), tip.getILon(), tip.getILat());
        if (dev > maxDev) { maxDev = dev; devLat = (nd.getILat() - 90000000) / 1e6; devLon = (nd.getILon() - 180000000) / 1e6; }
      }
      int chord = (int) CheapRulerDist(entry.getILon(), entry.getILat(), tip.getILon(), tip.getILat());
      System.out.printf("CORRIDOR entry->tip: real road %dm, chord %dm, max deviation from chord = %dm (at %.5f,%.5f)%n",
        t.distance, chord, (int) maxDev, devLat, devLon);
      System.out.println(maxDev > 120
        ? "  => real road detours FAR from the straight edge's line: the straight edge is a BEELINE across the gap."
        : "  => real road hugs the straight edge's line: the edge approximates a real (sparse) road.");
    } catch (Exception ex) {
      System.out.println("CORRIDOR: failed " + ex.getClass().getSimpleName());
    }
  }

  /** Perpendicular distance (m) from point P to the infinite line through A,B (CheapRuler-scaled). */
  private static double perpDistMeters(int plon, int plat, int alon, int alat, int blon, int blat) {
    double abLon = blon - alon, abLat = blat - alat;
    double apLon = plon - alon, apLat = plat - alat;
    double abLen2 = abLon * abLon + abLat * abLat;
    if (abLen2 == 0) return CheapRulerDist(plon, plat, alon, alat);
    double tt = (apLon * abLon + apLat * abLat) / abLen2;
    double projLon = alon + tt * abLon, projLat = alat + tt * abLat;
    return CheapRulerDist(plon, plat, (int) projLon, (int) projLat);
  }

  /**
   * Verify voice-hint correctness AFTER the beeline excision (user requirement:
   * "voice instructions must still be correct"). Checks the de-beelined Wegendorf loop:
   * no leftover U-turn/beeline at the old antenna, monotonic hint indices, hint distances
   * sum to the track distance, and every committed waypoint resolves to a surviving node.
   */
  @Test
  public void verifyWegendorfVoiceHints() throws Exception {
    File projectDir = new File(".").getCanonicalFile().getParentFile();
    File segDir = new File(projectDir, "segments4");
    LoopTestSegments.ensureRegion(segDir, LoopTestRegion.URBAN_BERLIN);
    File profileFile = new File(projectDir, "misc/profiles2/gravel.brf");
    if (!profileFile.exists()) { System.out.println("VOICE: profile missing"); return; }

    List<OsmNodeNamed> wplist = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from"; start.ilon = LoopTestRegion.URBAN_BERLIN.ilon; start.ilat = LoopTestRegion.URBAN_BERLIN.ilat;
    wplist.add(start);
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profileFile.getAbsolutePath();
    rctx.startDirection = 90; rctx.roundTripDistance = 15900;
    rctx.roundTripAlgorithm = RoundTripAlgorithm.ISO_GREEDY; rctx.roundTripStrictQuality = false;
    rctx.turnInstructionMode = 2; // locus — generate voice hints so we can verify them
    File tmp = File.createTempFile("wegvoice", "");
    RoutingEngine re = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wplist, rctx,
      RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.doRun(0);
    OsmTrack track = re.getFoundTrack();
    if (track == null) { System.out.println("VOICE: no track"); return; }
    int n = track.nodes.size();
    System.out.printf("VOICE: track nodes=%d dist=%dm%n", n, track.distance);

    if (track.voiceHints == null) { System.out.println("VOICE: no voiceHints (FAIL — expected hints)"); return; }
    List<VoiceHint> hints = track.voiceHints.list;
    int uTurns = 0, beelines = 0, prevIdx = -1; boolean monotonic = true;
    double sumDist = 0;
    for (VoiceHint h : hints) {
      sumDist += h.distanceToNext;
      if (h.indexInTrack < prevIdx) monotonic = false;
      if (h.indexInTrack < 0 || h.indexInTrack >= n) System.out.printf("  !! hint index %d out of [0,%d)%n", h.indexInTrack, n);
      prevIdx = h.indexInTrack;
      if (h.cmd == 10 || h.cmd == 11 || h.cmd == 15) { // TLU / TRU / TU
        uTurns++;
        OsmPathElement nd = track.nodes.get(Math.max(0, Math.min(n - 1, h.indexInTrack)));
        System.out.printf("  U-TURN cmd=%d at idx=%d (%.5f,%.5f) distToNext=%.0f%n",
          h.cmd, h.indexInTrack, (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6, h.distanceToNext);
      }
      if (h.cmd == 16) { // BL beeline
        beelines++;
        OsmPathElement nd = track.nodes.get(Math.max(0, Math.min(n - 1, h.indexInTrack)));
        System.out.printf("  BEELINE-HINT at idx=%d (%.5f,%.5f)%n",
          h.indexInTrack, (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6);
      }
    }
    System.out.println("  hints within 250m of old antenna junction (52.60672,13.77000):");
    int probeLon = 180000000 + (int) (13.77000 * 1e6 + 0.5), probeLat = 90000000 + (int) (52.60672 * 1e6 + 0.5);
    for (VoiceHint h : hints) {
      if (h.indexInTrack < 0 || h.indexInTrack >= n) continue;
      OsmPathElement nd = track.nodes.get(h.indexInTrack);
      if (CheapRulerDist(nd.getILon(), nd.getILat(), probeLon, probeLat) <= 250) {
        System.out.printf("    cmd=%d idx=%d (%.5f,%.5f) distToNext=%.0f%n",
          h.cmd, h.indexInTrack, (nd.getILat() - 90000000) / 1e6, (nd.getILon() - 180000000) / 1e6, h.distanceToNext);
      }
    }
    System.out.printf("VOICE SUMMARY: hints=%d uTurns=%d beelineHints=%d monotonic=%s hintDistSum=%.0fm vs trackDist=%dm (delta=%.0fm)%n",
      hints.size(), uTurns, beelines, monotonic, sumDist, track.distance, Math.abs(sumDist - track.distance));

    if (track.matchedWaypoints != null) {
      for (int w = 0; w < track.matchedWaypoints.size(); w++) {
        MatchedWaypoint mw = track.matchedWaypoints.get(w);
        boolean ok = mw.indexInTrack >= 0 && mw.indexInTrack < n;
        System.out.printf("  wp[%d] %s indexInTrack=%d %s%n", w, mw.name, mw.indexInTrack,
          ok ? "(valid)" : "<<< INVALID INDEX");
      }
    }
    System.out.println(uTurns == 0 && beelines == 0 && monotonic && Math.abs(sumDist - track.distance) < 50
      ? "VOICE: PASS (no U-turn/beeline, monotonic, distances consistent)"
      : "VOICE: CHECK (see flags above)");
  }

  private static String classifyNullEdgeP2P(File segDir, File profileFile,
                                            OsmPathElement A, OsmPathElement B, int straight) {
    try {
      OsmNodeNamed a = new OsmNodeNamed(); a.name = "a"; a.ilon = A.getILon(); a.ilat = A.getILat();
      OsmNodeNamed b = new OsmNodeNamed(); b.name = "b"; b.ilon = B.getILon(); b.ilat = B.getILat();
      List<OsmNodeNamed> wp = new ArrayList<>(); wp.add(a); wp.add(b);
      RoutingContext rc = new RoutingContext();
      rc.localFunction = profileFile.getAbsolutePath();
      File tmp = File.createTempFile("edgeP2P", "");
      RoutingEngine e = new RoutingEngine(tmp.getAbsolutePath(), tmp.getAbsolutePath(), segDir, wp, rc, 0);
      e.doRun(0);
      OsmTrack t = e.getFoundTrack();
      if (t == null) return "=> BEELINE (no P2P route between endpoints; error=" + e.getErrorMessage() + ")";
      boolean p2pBeeline = false;
      for (OsmPathElement nd : t.nodes) {
        String tg = nd.message != null ? nd.message.wayKeyValues : null;
        if (tg != null && tg.contains("direct_segment")) p2pBeeline = true;
      }
      double ratio = straight > 0 ? (double) t.distance / straight : 1.0;
      if (p2pBeeline) return String.format("=> BEELINE (P2P itself beelines; road=%dm straight=%dm)", t.distance, straight);
      if (ratio > 1.25) return String.format("=> BEELINE (straight %dm SHORTCUTS real road %dm, ratio %.2f)", straight, t.distance, ratio);
      return String.format("=> real undetailed road (P2P %dm ≈ straight %dm, ratio %.2f)", t.distance, straight, ratio);
    } catch (Exception ex) {
      return "=> classify failed: " + ex.getClass().getSimpleName();
    }
  }

  private static int nearestNodeIndex(List<OsmPathElement> nodes, int ilon, int ilat) {
    int best = -1; double bd = Double.MAX_VALUE;
    for (int k = 0; k < nodes.size(); k++) {
      double d = CheapRulerDist(nodes.get(k).getILon(), nodes.get(k).getILat(), ilon, ilat);
      if (d < bd) { bd = d; best = k; }
    }
    return best;
  }

  private static double CheapRulerDist(int lon1, int lat1, int lon2, int lat2) {
    return btools.util.CheapRuler.distance(lon1, lat1, lon2, lat2);
  }
}
