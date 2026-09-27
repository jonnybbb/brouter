package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.NodesCache;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmLinkHolder;
import btools.mapaccess.OsmNode;
import btools.mapaccess.OsmNodesMap;

/**
 * The woven search graph, retained across the leg searches of one round-trip
 * request.
 *
 * <p>A leg search normally consumes its graph: every settled link is unlinked
 * from its source node, the counter-link is dropped, peninsulas are pruned at
 * weave time and nodes vanish under memory pressure. That is why
 * {@code findTrack} rebuilds the {@link NodesCache} per search — and why a
 * round trip, which routes hundreds of legs through one neighbourhood,
 * spends most of its time decoding the same segments again (measured 70–98 %
 * of decodes were repeats, 37–76 % of wall time).
 *
 * <p>Here the graph is never mutated. What the unlinking achieved is
 * reproduced through the holder chains the search already maintains:
 * <ul>
 *   <li>a settled link direction is one whose holders were invalidated at pop
 *       ({@code airdistance == -1}); expansion skips it, as it would skip an
 *       unlinked link;</li>
 *   <li>the counter-link the cold search drops gets a sentinel holder that
 *       reads as settled;</li>
 *   <li>the start link, which the cold search never unlinks, is exempt;</li>
 *   <li>peninsula pruning and memory collection stay off, so the graph only
 *       grows — bounded by {@link #overBudget()}, at which point the engine
 *       drops this cache and starts a fresh one.</li>
 * </ul>
 * Every holder set during a search is recorded per (link, direction) and
 * cleared by {@link #beginSearch()}, so the next search starts on a clean
 * graph. Routes are byte-identical to the cold search; only the work of
 * re-decoding disappears.
 */
final class WarmSearchCache {

  /** Rough footprint of a retained node with its links and map entry. */
  private static final long BYTES_PER_NODE = 150L;

  /** Marks a link direction the cold search would have unlinked; never expanded. */
  private static final OsmPath REMOVED = new StdPath();

  static {
    REMOVED.airdistance = -1;
  }

  final NodesCache cache;
  private final long maxBytes;
  private final List<OsmLink> links = new ArrayList<>();
  private final List<OsmNode> sources = new ArrayList<>();
  private OsmLink exempt1;
  private OsmLink exempt2;
  private int searches;

  WarmSearchCache(NodesCache cache, long maxBytes) {
    this.cache = cache;
    this.maxBytes = maxBytes;
    OsmNodesMap map = cache.nodesMap;
    map.keepWoven = true;
    // The cold search bounds its map because it collects under pressure; this
    // graph is bounded by overBudget() instead, so the collector must not run.
    map.maxmem = Long.MAX_VALUE / 4;
  }

  /** Clear every holder the previous search left on the graph. */
  void beginSearch() {
    for (int i = 0; i < links.size(); i++) {
      links.get(i).setFirstLinkHolder(null, sources.get(i));
    }
    links.clear();
    sources.clear();
    exempt1 = null;
    exempt2 = null;
    OsmNodesMap map = cache.nodesMap;
    map.endNode1 = null;
    map.endNode2 = null;
    map.destination = null;
    map.currentMaxCost = 1000000000;
    map.currentPathCost = 0;
    map.cleanupMode = 0;
    searches++;
  }

  /** Call before the first holder is added to {@code link} from {@code source}. */
  void recordHolder(OsmLink link, OsmNode source) {
    if (link.getFirstLinkHolder(source) == null) {
      links.add(link);
      sources.add(source);
    }
  }

  /**
   * The cold search's unlink step for a popped path: the settled direction is
   * already marked by its invalidated holders; when the cold search would also
   * drop the counter-link, give that direction a sentinel holder.
   */
  void settle(OsmLink link, OsmNode current, boolean dropCounterLink) {
    if (dropCounterLink && link.getFirstLinkHolder(current) == null) {
      links.add(link);
      sources.add(current);
      link.setFirstLinkHolder(REMOVED, current);
    }
  }

  /** The cold search leaves the start link attached; remember it so it stays expandable. */
  void exempt(OsmLink link) {
    if (exempt1 == null) {
      exempt1 = link;
    } else if (exempt2 == null) {
      exempt2 = link;
    }
  }

  /** Whether the cold search would already have unlinked {@code link} from {@code from}. */
  boolean isSettled(OsmLink link, OsmNode from) {
    if (link == exempt1 || link == exempt2) {
      return false;
    }
    OsmLinkHolder holder = link.getFirstLinkHolder(from);
    return holder != null && ((OsmPath) holder).airdistance == -1;
  }

  boolean overBudget() {
    return (long) cache.nodesMap.size() * BYTES_PER_NODE > maxBytes;
  }

  int searches() {
    return searches;
  }

  void close() {
    cache.close();
  }
}
