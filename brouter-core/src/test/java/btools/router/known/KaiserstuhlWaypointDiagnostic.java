package btools.router.known;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import btools.router.OsmNodeNamed;
import btools.router.OsmPathElement;
import btools.router.OsmTrack;
import btools.router.RoundTripAlgorithm;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

import org.junit.Assume;
import org.junit.Test;

/**
 * Waypoint-placement diagnostic for the Kaiserstuhl Classic 53km loop.
 *
 * Start: (48.01983, 7.81957) — Freiburg area
 * Target: 53km, initial bearing 240°
 * Reference corpus: 210+ similar loops within 5km of this start.
 *
 * <p>Two complementary measurements:
 * <ol>
 *   <li><b>Baseline AUTO run</b>: what loop does the planner produce
 *       with just (start, distance, direction)? Where are its
 *       intermediate waypoints?</li>
 *   <li><b>Forced-vias run</b>: take the cyclist's actual GPX, sample 4
 *       intermediate points, feed them as via waypoints. If brouter
 *       produces a clean reproduction, the bottleneck is confirmed to
 *       be waypoint placement.</li>
 * </ol>
 *
 * <p>Edge-overlap metric measures what fraction of the cyclist's
 * route segments the planner's output also traverses (by node-ID
 * pairs).
 */
public class KaiserstuhlWaypointDiagnostic {

  private static final double START_LAT = 48.01983;
  private static final double START_LON = 7.81957;
  private static final int TARGET_KM = 53;
  private static final int TARGET_DIRECTION = 240;
  private static final String REFERENCE_GPX = "2024-04-25__53km__1532074047__Kaiserstuhl Classic.gpx";

  private File profileFile(String name) {
    File p = new File("misc/profiles2/" + name + ".brf");
    if (!p.exists()) p = new File("../misc/profiles2/" + name + ".brf");
    return p.exists() ? p : null;
  }

  private File segDir() {
    File f = new File("../segments4");
    if (!f.exists()) f = new File("segments4");
    return f.exists() ? f : null;
  }

  private File gpxFile() {
    File f = new File("../tmp/gpx/freiburg/" + REFERENCE_GPX);
    if (!f.exists()) f = new File("tmp/gpx/freiburg/" + REFERENCE_GPX);
    return f.exists() ? f : null;
  }

  private static int ilon(double lon) { return (int) Math.round((lon + 180) * 1_000_000); }
  private static int ilat(double lat) { return (int) Math.round((lat + 90) * 1_000_000); }

  private static List<double[]> parseTrkpts(File gpx) throws Exception {
    List<double[]> out = new ArrayList<>();
    String txt = new String(Files.readAllBytes(gpx.toPath()));
    Matcher m = Pattern.compile("<trkpt\\s+lat=\"([^\"]+)\"\\s+lon=\"([^\"]+)\"").matcher(txt);
    while (m.find()) out.add(new double[]{Double.parseDouble(m.group(2)), Double.parseDouble(m.group(1))});
    return out;
  }

  /** Set of (nodeA,nodeB) un-ordered pairs as Strings. */
  private static Set<String> edgeSet(OsmTrack track) {
    Set<String> set = new HashSet<>();
    if (track == null || track.nodes == null) return set;
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      long ka = a.getIdFromPos();
      long kb = b.getIdFromPos();
      String key = ka < kb ? (ka + "_" + kb) : (kb + "_" + ka);
      set.add(key);
    }
    return set;
  }

  @Test
  public void diagnose() throws Exception {
    Assume.assumeTrue("loop-tests opt-in", Boolean.getBoolean("loop.tests"));
    File seg = segDir();
    Assume.assumeTrue("segments4 not found", seg != null);
    Assume.assumeTrue("E5_N45 segment", new File(seg, "E5_N45.rd5").exists());
    File pf = profileFile("fastbike");
    Assume.assumeTrue("fastbike profile", pf != null);
    File ref = gpxFile();
    Assume.assumeTrue("reference gpx present", ref != null);

    PrintWriter out = new PrintWriter("../docs/features/kaiserstuhl-waypoint-diagnostic.md");
    out.println("# Kaiserstuhl 53km — Waypoint Placement Diagnostic");
    out.println();
    out.printf("Start: (%.5f, %.5f)  •  Target: %d km, bearing %d°%n",
      START_LAT, START_LON, TARGET_KM, TARGET_DIRECTION);
    out.println("Reference: `" + REFERENCE_GPX + "`");
    out.println();

    // ----- Pass 1: route the reference GPX via brouter point-to-point ------
    out.println("## 1. Reference cyclist route (brouter point-to-point through GPX vias)");
    out.println();
    List<double[]> refPts = parseTrkpts(ref);
    out.printf("GPX trkpts: %d, first/last close: %.0fm gap%n", refPts.size(),
      distance(refPts.get(0), refPts.get(refPts.size() - 1)));
    out.println();

    // Sample 6 waypoints from the reference GPX
    int n = 6;
    List<double[]> refSampled = new ArrayList<>();
    for (int i = 0; i < n; i++) refSampled.add(refPts.get((int) (i * (refPts.size() - 1.0) / n)));
    refSampled.add(refPts.get(0)); // close

    OsmTrack refTrack = routeP2P(refSampled, seg, pf);
    out.printf("brouter point-to-point through cyclist's vias: distance=%dm, nodes=%d%n",
      refTrack != null ? refTrack.distance : -1,
      refTrack != null && refTrack.nodes != null ? refTrack.nodes.size() : 0);
    out.println();
    Set<String> refEdges = edgeSet(refTrack);
    out.println("Cyclist's sampled waypoints:");
    for (int i = 0; i < refSampled.size(); i++) {
      double[] p = refSampled.get(i);
      out.printf("  %d: (%.5f, %.5f) — https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=15/%.5f/%.5f%n",
        i, p[1], p[0], p[1], p[0], p[1], p[0]);
    }
    out.println();

    // ----- Pass 2: AUTO planner (no vias, just start+direction+distance) ----
    out.println("## 2. AUTO planner output (start + distance + direction only)");
    out.println();
    List<OsmNodeNamed> wp = new ArrayList<>();
    OsmNodeNamed start = new OsmNodeNamed();
    start.name = "from"; start.ilon = ilon(START_LON); start.ilat = ilat(START_LAT);
    wp.add(start);
    RoutingContext rc = new RoutingContext();
    rc.localFunction = pf.getAbsolutePath();
    rc.startDirection = TARGET_DIRECTION;
    rc.roundTripDistance = (int)(TARGET_KM * 1000 / (2 * Math.PI));
    rc.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    RoutingEngine re = new RoutingEngine(null, null, seg, wp, rc, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(0);
    String err = re.getErrorMessage();
    OsmTrack autoTrack = re.getFoundTrack();
    out.printf("error: %s%n", err == null ? "PASSED" : err);
    if (autoTrack != null) {
      out.printf("AUTO route: distance=%dm, nodes=%d%n", autoTrack.distance, autoTrack.nodes.size());
    }
    out.println();

    // AUTO planner's actual chosen waypoints — the smoking gun for diagnosis.
    // These are the intermediate "stops" the planner picked geometrically.
    out.println("**AUTO planner's chosen waypoints** (the candidate placements):");
    OsmTrack inspectAuto = autoTrack != null ? autoTrack : re.getLastRejectedTrack();
    List<btools.mapaccess.MatchedWaypoint> inspectedWaypoints =
      inspectAuto == null ? null : inspectAuto.getMatchedWaypoints();
    if (inspectedWaypoints != null) {
      int i = 0;
      for (btools.mapaccess.MatchedWaypoint mw : inspectedWaypoints) {
        double lat = mw.crosspoint.ilat / 1_000_000.0 - 90;
        double lon = mw.crosspoint.ilon / 1_000_000.0 - 180;
        out.printf("  %d: (%.5f, %.5f) name=%s — https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=15/%.5f/%.5f%n",
          i++, lat, lon, mw.name, lat, lon, lat, lon);
      }
    } else {
      out.println("  (matchedWaypoints not populated)");
    }
    out.println();

    Set<String> autoEdges = edgeSet(autoTrack);
    int autoOverlap = 0;
    for (String e : autoEdges) if (refEdges.contains(e)) autoOverlap++;
    out.printf("Edge overlap with cyclist's vias-routed track: %d / %d = %.1f%% of AUTO's edges%n",
      autoOverlap, autoEdges.size(), autoEdges.isEmpty() ? 0.0 : 100.0 * autoOverlap / autoEdges.size());
    out.println();

    // ----- Pass 3: brouter point-to-point through the same sampled cyclist vias
    // (already done in Pass 1 - that's refTrack). Just confirm it's a clean loop.
    out.println("## 3. Verdict");
    out.println();
    out.printf("**Cyclist's-vias loop** (Pass 1) — clean: %s, length %dm%n",
      refTrack != null && refTrack.distance > 0 ? "YES" : "NO",
      refTrack != null ? refTrack.distance : 0);
    out.printf("**AUTO planner loop** (Pass 2) — clean: %s, length %dm%n",
      autoTrack != null && err == null ? "YES (gate accepted)" : "NO (" + err + ")",
      autoTrack != null ? autoTrack.distance : 0);
    out.printf("**Edge overlap AUTO vs cyclist-vias loop**: %.1f%%%n",
      autoEdges.isEmpty() ? 0.0 : 100.0 * autoOverlap / autoEdges.size());
    out.println();
    out.println("**Interpretation:**");
    out.println("- If Pass 1 is clean and Pass 2 fails or overlaps < 30%, the planner's");
    out.println("  candidate placement is the bottleneck. The engine+profile can route");
    out.println("  the same area cleanly given good waypoints; the planner just doesn't");
    out.println("  pick those waypoints.");
    out.println("- Edge overlap close to 0% confirms the planner is in a different");
    out.println("  geographic neighbourhood from the cyclist.");

    // ----- Pass 3: AUTO planner with NO direction constraint --------------
    // Isolates the bearing-anchor hypothesis: if the planner picks a
    // cyclist-like (north-going) loop when the 240° constraint is removed,
    // the candidate provider's angular anchor is the bottleneck.
    out.println();
    out.println("## 3. AUTO planner output, NO direction constraint");
    out.println();
    List<OsmNodeNamed> wp2 = new ArrayList<>();
    OsmNodeNamed start2 = new OsmNodeNamed();
    start2.name = "from"; start2.ilon = ilon(START_LON); start2.ilat = ilat(START_LAT);
    wp2.add(start2);
    RoutingContext rc2 = new RoutingContext();
    rc2.localFunction = pf.getAbsolutePath();
    // No startDirection set
    rc2.roundTripDistance = (int)(TARGET_KM * 1000 / (2 * Math.PI));
    rc2.roundTripAlgorithm = RoundTripAlgorithm.AUTO;
    RoutingEngine re2 = new RoutingEngine(null, null, seg, wp2, rc2, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re2.quite = true;
    re2.doRun(0);
    String err2 = re2.getErrorMessage();
    OsmTrack autoNoDir = re2.getFoundTrack();
    out.printf("error: %s%n", err2 == null ? "PASSED" : err2);
    if (autoNoDir != null) {
      out.printf("AUTO route (no direction): distance=%dm, nodes=%d%n",
        autoNoDir.distance, autoNoDir.nodes.size());
    }
    Set<String> noDirEdges = edgeSet(autoNoDir);
    int noDirOverlap = 0;
    for (String e : noDirEdges) if (refEdges.contains(e)) noDirOverlap++;
    out.printf("Edge overlap with cyclist's vias-routed track: %.1f%%%n",
      noDirEdges.isEmpty() ? 0.0 : 100.0 * noDirOverlap / noDirEdges.size());

    OsmTrack inspectNoDir = autoNoDir != null ? autoNoDir : re2.getLastRejectedTrack();
    List<btools.mapaccess.MatchedWaypoint> noDirMwps =
      inspectNoDir == null ? null : inspectNoDir.getMatchedWaypoints();
    if (noDirMwps != null) {
      out.println("Chosen waypoints (no direction):");
      int i = 0;
      for (btools.mapaccess.MatchedWaypoint mw : noDirMwps) {
        double lat = mw.crosspoint.ilat / 1_000_000.0 - 90;
        double lon = mw.crosspoint.ilon / 1_000_000.0 - 180;
        out.printf("  %d: (%.5f, %.5f) name=%s%n", i++, lat, lon, mw.name);
      }
    }

    out.close();
    System.out.println("Wrote ../docs/features/kaiserstuhl-waypoint-diagnostic.md");
    System.out.printf("ref-track:  %dm (Pass 1)%n", refTrack != null ? refTrack.distance : -1);
    System.out.printf("auto-track: %dm (Pass 2)  err=%s%n", autoTrack != null ? autoTrack.distance : -1, err);
    System.out.printf("overlap: %.1f%%%n", autoEdges.isEmpty() ? 0.0 : 100.0 * autoOverlap / autoEdges.size());
    System.out.printf("auto-no-direction: %dm  err=%s  overlap=%.1f%%%n",
      autoNoDir != null ? autoNoDir.distance : -1, err2,
      noDirEdges.isEmpty() ? 0.0 : 100.0 * noDirOverlap / noDirEdges.size());
  }

  private static OsmTrack routeP2P(List<double[]> pts, File seg, File pf) {
    List<OsmNodeNamed> wp = new ArrayList<>();
    for (int i = 0; i < pts.size(); i++) {
      double[] p = pts.get(i);
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : i == pts.size() - 1 ? "to" : "via" + i;
      n.ilon = ilon(p[0]); n.ilat = ilat(p[1]);
      wp.add(n);
    }
    RoutingContext rc = new RoutingContext();
    rc.localFunction = pf.getAbsolutePath();
    RoutingEngine re = new RoutingEngine(null, null, seg, wp, rc);
    re.quite = true;
    try {
      re.doRun(0);
      return re.getFoundTrack();
    } catch (Exception e) {
      return null;
    }
  }

  private static double distance(double[] a, double[] b) {
    double lat1 = a[1] * Math.PI / 180, lat2 = b[1] * Math.PI / 180;
    double dLat = lat2 - lat1;
    double dLon = (b[0] - a[0]) * Math.PI / 180;
    double s = Math.sin(dLat / 2);
    double c = Math.sin(dLon / 2);
    double aa = s * s + Math.cos(lat1) * Math.cos(lat2) * c * c;
    return 2 * 6371000 * Math.asin(Math.sqrt(aa));
  }
}
