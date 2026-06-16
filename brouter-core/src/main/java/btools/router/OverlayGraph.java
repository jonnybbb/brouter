package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.util.CheapRuler;

/**
 * TRUE contraction overlay for round-trip loop planning (the user's portalized-capsule model).
 *
 * <p>Dense road areas are replaced by a tiny graph of REAL elements: each capsule contributes its
 * boundary PORTALS (real nodes where radial routes cross the dense edge) plus real routed shortcut
 * edges — a RING around the boundary and CROSS through-corridors across it. Capsules are stitched
 * together by CONNECTOR edges (real routed paths between nearby portals of different capsules) and
 * the start is joined by ACCESS edges. Every edge stores its real {@link OsmTrack} geometry, so the
 * loop search runs on this handful of nodes and reconstruction just concatenates the stored paths —
 * a contraction hierarchy in miniature, honouring the spec's "no fake ring roads".
 *
 * <p>The interior spaghetti is never exposed to the loop search, so a same-side dip-in is impossible
 * by construction; a capsule can only be entered at a portal and left at a portal.
 */
public final class OverlayGraph {

  public static final int START = 0;
  public static final int PORTAL = 1;

  // edge kinds
  public static final int ACCESS = 0;     // start -> portal
  public static final int CONNECTOR = 1;  // portal(capsule A) -> portal(capsule B), open country
  public static final int CORRIDOR = 2;   // through-capsule (cross), opposite portals
  public static final int RING = 3;       // around capsule boundary, consecutive portals

  public static final class Node {
    public final int id, ilon, ilat, type, capsule; // capsule = index into capsule list, -1 for START
    Node(int id, int ilon, int ilat, int type, int capsule) {
      this.id = id; this.ilon = ilon; this.ilat = ilat; this.type = type; this.capsule = capsule;
    }
  }

  public static final class Edge {
    public final int from, to, kind, distM;
    public final double cost;
    public final OsmTrack geom;
    Edge(int from, int to, int kind, int distM, double cost, OsmTrack geom) {
      this.from = from; this.to = to; this.kind = kind; this.distM = distM; this.cost = cost; this.geom = geom;
    }
  }

  /**
   * Point-to-point router that honours a soft-nogo mask. {@code nogos} are capsule interiors the
   * routed edge must avoid (finite weight = strongly discouraged but boundary-reachable); pass an
   * empty list for an unmasked edge.
   */
  public interface MaskedRouter {
    OsmTrack route(int fromIlon, int fromIlat, int toIlon, int toIlat, List<OsmNogoPolygon> nogos);
  }

  public final List<Node> nodes = new ArrayList<>();
  public final List<Edge> edges = new ArrayList<>();

  private int addNode(int ilon, int ilat, int type, int capsule) {
    int id = nodes.size();
    nodes.add(new Node(id, ilon, ilat, type, capsule));
    return id;
  }

  /** Existing node within {@code tolM} of (ilon,ilat), or -1. Small N → linear scan is fine. */
  private int findNear(int ilon, int ilat, double tolM) {
    for (Node n : nodes) {
      if (CheapRuler.distance(n.ilon, n.ilat, ilon, ilat) <= tolM) return n.id;
    }
    return -1;
  }

  /** Adjacency: edges incident to each node id (both directions; edges are undirected). */
  public List<List<Edge>> adjacency() {
    List<List<Edge>> adj = new ArrayList<>();
    for (int i = 0; i < nodes.size(); i++) adj.add(new ArrayList<>());
    for (Edge e : edges) { adj.get(e.from).add(e); adj.get(e.to).add(e); }
    return adj;
  }

  // ---- builder ----------------------------------------------------------------

  /**
   * Build the overlay around {@code (sx,sy)} from detected capsule polygons (each carries a
   * bounding centre+radius from {@link OsmNogoPolygon#calcBoundingCircle()}).
   *
   * @param capsules    dense-area polygons, biggest-first preferred (we cap at maxCapsules)
   * @param router      real point-to-point router (production cost model)
   * @param maxCapsules cap on how many capsules to contract (largest by radius)
   * @param numPortals  portal probes per capsule
   * @param kConnector  connectors per terminal to nearest other-capsule terminals
   * @param maxConnectorM air-distance cap for a connector (skip far pairs)
   */
  public static OverlayGraph build(int sx, int sy, List<OsmNogoPolygon> capsules,
                                   MaskedRouter router, int maxCapsules,
                                   int numPortals, int kConnector, double maxConnectorM,
                                   double softNogoWeight) {
    OverlayGraph g = new OverlayGraph();
    g.addNode(sx, sy, START, -1);

    List<OsmNogoPolygon> sel = selectCapsules(capsules, maxCapsules, softNogoWeight);
    // mask = soft-nogo over every contracted capsule interior; per-edge sublists are derived from it
    List<OsmNogoPolygon> maskAll = softNogoWeight > 0 ? sel : new ArrayList<>();

    // per-capsule: portals + ring + cross
    for (int ci = 0; ci < sel.size(); ci++) {
      OsmNogoPolygon poly = sel.get(ci);
      double radiusM = Math.max(400.0, poly.radius);

      // portals = real boundary crossings of radial probe routes (unmasked: we are leaving this
      // capsule, so its own interior must stay routable for the probe)
      int[] arr = new int[numPortals];
      for (int i = 0; i < numPortals; i++) {
        double bearing = 360.0 * i / numPortals;
        int[] dest = CheapRuler.destination(poly.ilon, poly.ilat, radiusM * 1.6, bearing);
        OsmTrack t = router.route(poly.ilon, poly.ilat, dest[0], dest[1], new ArrayList<>());
        int[] p = boundaryCrossing(t, poly.ilon, poly.ilat, radiusM);
        if (p == null) { arr[i] = -1; continue; }
        int existing = g.findNear(p[0], p[1], 120);
        arr[i] = existing >= 0 ? existing : g.addNode(p[0], p[1], PORTAL, ci);
      }

      // RING: consecutive portals, hugging the boundary — mask ALL interiors so it goes around
      for (int i = 0; i < numPortals; i++) {
        int a = arr[i], b = arr[(i + 1) % numPortals];
        if (a < 0 || b < 0 || a == b) continue;
        addRoutedEdge(g, router, a, b, RING, maskAll);
      }
      // CROSS / through-corridor: opposite portals — mask all EXCEPT this capsule, so it crosses
      // this town's interior (the real through-road) but no other.
      List<OsmNogoPolygon> maskExceptCi = exclude(maskAll, poly);
      int half = numPortals / 2;
      int axes = Math.max(1, numPortals / 4);
      for (int a0 = 0; a0 < axes; a0++) {
        int a = arr[a0], b = arr[(a0 + half) % numPortals];
        if (a < 0 || b < 0 || a == b) continue;
        addRoutedEdge(g, router, a, b, CORRIDOR, maskExceptCi);
      }
    }

    // CONNECTOR + ACCESS: each terminal -> k nearest terminals of a different capsule, masked so
    // open-country links never dip into any town. Terminals = START + all portals.
    List<Node> terminals = new ArrayList<>();
    for (Node n : g.nodes) if (n.type == START || n.type == PORTAL) terminals.add(n);
    for (Node a : terminals) {
      List<Node> cand = new ArrayList<>();
      for (Node b : terminals) {
        if (b.id == a.id || b.capsule == a.capsule) continue;
        double d = CheapRuler.distance(a.ilon, a.ilat, b.ilon, b.ilat);
        if (d <= maxConnectorM) cand.add(b);
      }
      cand.sort((x, y) -> Double.compare(
        CheapRuler.distance(a.ilon, a.ilat, x.ilon, x.ilat),
        CheapRuler.distance(a.ilon, a.ilat, y.ilon, y.ilat)));
      int added = 0;
      for (Node b : cand) {
        if (added >= kConnector) break;
        if (g.hasEdge(a.id, b.id)) { added++; continue; }
        int kind = (a.type == START || b.type == START) ? ACCESS : CONNECTOR;
        // the start capsule stays soft-exitable; access from START routes through maskAll anyway
        if (addRoutedEdge(g, router, a.id, b.id, kind, maskAll)) added++;
      }
    }
    return g;
  }

  /** Largest {@code maxCapsules} capsules by radius, with soft-nogo weight applied (shared by builder + planner). */
  public static List<OsmNogoPolygon> selectCapsules(List<OsmNogoPolygon> capsules, int maxCapsules, double softNogoWeight) {
    List<OsmNogoPolygon> sel = new ArrayList<>(capsules);
    sel.sort((a, b) -> Double.compare(b.radius, a.radius));
    if (sel.size() > maxCapsules) sel = new ArrayList<>(sel.subList(0, maxCapsules));
    if (softNogoWeight > 0) for (OsmNogoPolygon p : sel) p.nogoWeight = softNogoWeight;
    return sel;
  }

  private static List<OsmNogoPolygon> exclude(List<OsmNogoPolygon> all, OsmNogoPolygon keepOpen) {
    List<OsmNogoPolygon> out = new ArrayList<>(all.size());
    for (OsmNogoPolygon p : all) if (p != keepOpen) out.add(p);
    return out;
  }

  private boolean hasEdge(int a, int b) {
    for (Edge e : edges) if ((e.from == a && e.to == b) || (e.from == b && e.to == a)) return true;
    return false;
  }

  private static boolean addRoutedEdge(OverlayGraph g, MaskedRouter router,
                                       int aId, int bId, int kind, List<OsmNogoPolygon> nogos) {
    Node a = g.nodes.get(aId), b = g.nodes.get(bId);
    OsmTrack t = router.route(a.ilon, a.ilat, b.ilon, b.ilat, nogos);
    if (t == null || t.nodes == null || t.nodes.size() < 2) return false;
    g.edges.add(new Edge(aId, bId, kind, t.distance, t.cost, t));
    return true;
  }

  /** Append an externally-built node (e.g. an open-country anchor). Returns its id. */
  public int addAnchor(int ilon, int ilat) {
    return addNode(ilon, ilat, PORTAL, -2);
  }

  /** Add a routed edge between two existing nodes (used by the planner for anchor stitching). */
  public boolean connect(MaskedRouter router, int aId, int bId, int kind, List<OsmNogoPolygon> nogos) {
    if (hasEdge(aId, bId)) return false;
    return addRoutedEdge(this, router, aId, bId, kind, nogos);
  }

  /** First node on the outward route at &ge; radius from the centre = the real boundary portal. */
  private static int[] boundaryCrossing(OsmTrack t, int cx, int cy, double radiusM) {
    if (t == null || t.nodes == null || t.nodes.size() < 2) return null;
    for (OsmPathElement n : t.nodes) {
      if (CheapRuler.distance(cx, cy, n.getILon(), n.getILat()) >= radiusM) {
        return new int[]{n.getILon(), n.getILat()};
      }
    }
    OsmPathElement last = t.nodes.get(t.nodes.size() - 1);
    return new int[]{last.getILon(), last.getILat()};
  }
}
