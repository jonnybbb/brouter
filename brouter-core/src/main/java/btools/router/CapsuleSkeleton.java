package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.util.CheapRuler;

/**
 * Working-graph CONTRACTION skeleton for an urban capsule (the user's ring+cross model):
 * replace a dense interior's spaghetti with a small, fully-routable graph built from REAL
 * roads — an outer RING of boundary portals plus two crossing CORRIDORS (one per axis,
 * e.g. N–S and E–W). This is how cities are actually traversable (ring road + arterials),
 * and unlike masking it keeps everything routable while collapsing thousands of interior
 * nodes to a handful of real paths.
 *
 * <p>All elements are REAL (no synthetic geometry, honouring the spec's "no fake ring
 * roads"): portals are the real nodes where a radial route crosses the capsule boundary;
 * ring/cross segments are real routed paths between portals. Final reconstruction therefore
 * needs only the portal/corridor coordinates as via points.
 *
 * <p>This class is profile-agnostic — it takes a {@link Router} (any point-to-point router)
 * so it is trivially testable and reuses the production cost model.
 */
public final class CapsuleSkeleton {

  private CapsuleSkeleton() {}

  /** Point-to-point router abstraction (returns null on failure). Coordinates are BRouter ilon/ilat. */
  public interface Router {
    OsmTrack route(int fromIlon, int fromIlat, int toIlon, int toIlat);
  }

  public static final class Result {
    /** Real boundary portals in compass order (entries may be null where a direction failed). */
    public final List<int[]> portals;
    /** Real routed ring segments between consecutive portals. */
    public final List<OsmTrack> ring;
    /** Real routed crossing corridors (one per axis). */
    public final List<OsmTrack> cross;

    Result(List<int[]> portals, List<OsmTrack> ring, List<OsmTrack> cross) {
      this.portals = portals;
      this.ring = ring;
      this.cross = cross;
    }

    public int portalCount() {
      int c = 0;
      for (int[] p : portals) if (p != null) c++;
      return c;
    }
  }

  /**
   * @param cx,cy       capsule centre (BRouter ilon/ilat)
   * @param radiusM     capsule radius (meters); portals sit where radial routes cross it
   * @param numPortals  how many compass directions to probe (e.g. 8)
   * @param router      point-to-point router (real cost model)
   */
  public static Result build(int cx, int cy, double radiusM, int numPortals, Router router) {
    List<int[]> portals = new ArrayList<>(numPortals);
    for (int i = 0; i < numPortals; i++) {
      double bearing = 360.0 * i / numPortals;
      int[] dest = CheapRuler.destination(cx, cy, radiusM * 1.6, bearing);
      OsmTrack t = router.route(cx, cy, dest[0], dest[1]);
      portals.add(boundaryCrossing(t, cx, cy, radiusM));
    }

    // Ring: real path between consecutive portals (around the capsule edge).
    List<OsmTrack> ring = new ArrayList<>();
    for (int i = 0; i < numPortals; i++) {
      int[] a = portals.get(i), b = portals.get((i + 1) % numPortals);
      if (a == null || b == null) continue;
      OsmTrack t = router.route(a[0], a[1], b[0], b[1]);
      if (t != null && t.nodes != null && t.nodes.size() >= 2) ring.add(t);
    }

    // Cross: real corridors between opposite portals, one per axis. For numPortals=8 that
    // is portal0↔portal4 and portal2↔portal6 (the N–S and E–W through-roads).
    List<OsmTrack> cross = new ArrayList<>();
    int half = numPortals / 2;
    int axes = Math.max(1, numPortals / 4); // number of distinct through-axes
    for (int a0 = 0; a0 < axes; a0++) {
      int[] a = portals.get(a0), b = portals.get((a0 + half) % numPortals);
      if (a == null || b == null) continue;
      OsmTrack t = router.route(a[0], a[1], b[0], b[1]);
      if (t != null && t.nodes != null && t.nodes.size() >= 2) cross.add(t);
    }
    return new Result(portals, ring, cross);
  }

  /** First node on the outward route at &ge; radius from the centre = the real boundary portal. */
  private static int[] boundaryCrossing(OsmTrack t, int cx, int cy, double radiusM) {
    if (t == null || t.nodes == null || t.nodes.size() < 2) return null;
    for (OsmPathElement n : t.nodes) {
      if (CheapRuler.distance(cx, cy, n.getILon(), n.getILat()) >= radiusM) {
        return new int[]{n.getILon(), n.getILat()};
      }
    }
    OsmPathElement last = t.nodes.get(t.nodes.size() - 1); // route never reached the radius
    return new int[]{last.getILon(), last.getILat()};
  }
}
