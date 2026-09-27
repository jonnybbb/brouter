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
 *   <li>the start link's direction, which the cold search leaves attached
 *       when popped at tree depth one, is exempt until a deeper pop settles
 *       it — exactly when the cold search would unlink it;</li>
 *   <li>peninsula pruning and memory collection stay off, so the graph only
 *       grows — bounded by {@link #overBudget}, checked before a search and,
 *       through {@link OverBudget}, while it expands.</li>
 * </ul>
 * Every holder set during a search is recorded per (link, direction) and
 * cleared by {@link #beginSearch()}, so the next search starts on a clean
 * graph.
 *
 * <p>Only the standard path model is served this way. Kinematic models price
 * junction crossings from every link at a node ({@code KinematicPrePath}),
 * so the dead-end links that pruning removed would change their costs.
 */
final class WarmSearchCache {

  /** Rough footprint of a retained node with its links and map entry. */
  private static final long BYTES_PER_NODE = 150L;
  /** Footprint of an open path, as the cold search's own bound counts it. */
  private static final long BYTES_PER_PATH = 200L;

  /** Marks a link direction the cold search would have unlinked; never expanded. */
  private static final OsmPath REMOVED = new StdPath();

  static {
    REMOVED.airdistance = -1;
  }

  /** Thrown from inside a search when the retained graph outgrows its budget. */
  static final class OverBudget extends RuntimeException {
    OverBudget() {
      super("retained search graph over budget", null, false, false);
    }
  }

  final NodesCache cache;
  private final long maxBytes;
  private final List<OsmLink> links = new ArrayList<>();
  private final List<OsmNode> sources = new ArrayList<>();
  private final OsmLink[] exemptLinks = new OsmLink[2];
  private final OsmNode[] exemptSources = new OsmNode[2];
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
    exemptLinks[0] = exemptLinks[1] = null;
    exemptSources[0] = exemptSources[1] = null;
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
   * The cold search's unlink step for a path popped at tree depth above one:
   * the direction from {@code source} is settled (its holders were invalidated
   * at the pop, and any start-link exemption on it ends here); when the cold
   * search would also drop the counter-link, that direction gets a sentinel.
   */
  void settle(OsmLink link, OsmNode source, OsmNode current, boolean dropCounterLink) {
    for (int i = 0; i < 2; i++) {
      if (exemptLinks[i] == link && exemptSources[i] == source) {
        exemptLinks[i] = null;
        exemptSources[i] = null;
      }
    }
    if (dropCounterLink && link.getFirstLinkHolder(current) == null) {
      links.add(link);
      sources.add(current);
      link.setFirstLinkHolder(REMOVED, current);
    }
  }

  /**
   * A pop at tree depth one leaves the link attached in the cold search:
   * keep that direction expandable despite its invalidated holders.
   */
  void exempt(OsmLink link, OsmNode source) {
    for (int i = 0; i < 2; i++) {
      if (exemptLinks[i] == null) {
        exemptLinks[i] = link;
        exemptSources[i] = source;
        return;
      }
    }
  }

  /** Whether the cold search would already have unlinked {@code link} from {@code from}. */
  boolean isSettled(OsmLink link, OsmNode from) {
    for (int i = 0; i < 2; i++) {
      if (exemptLinks[i] == link && exemptSources[i] == from) {
        return false;
      }
    }
    OsmLinkHolder holder = link.getFirstLinkHolder(from);
    return holder != null && ((OsmPath) holder).airdistance == -1;
  }

  /** Retained graph plus the search state of {@code openPaths} paths against the memory class. */
  boolean overBudget(int openPaths) {
    long bytes = (long) cache.nodesMap.size() * BYTES_PER_NODE
      + (long) openPaths * BYTES_PER_PATH
      + (long) links.size() * 16L;
    return bytes > maxBytes;
  }

  int searches() {
    return searches;
  }

  void close() {
    cache.close();
  }
}
