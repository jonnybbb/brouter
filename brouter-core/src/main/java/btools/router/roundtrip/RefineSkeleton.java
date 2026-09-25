package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmNode;

/**
 * Immutable representation of a round-trip route skeleton (§4.7).
 */
public final class RefineSkeleton {
  private final MatchedWaypoint startWp;
  private final List<MatchedWaypoint> vias;
  private final MatchedWaypoint endWp;

  public RefineSkeleton(MatchedWaypoint startWp, List<MatchedWaypoint> vias, MatchedWaypoint endWp) {
    this.startWp = copyWaypoint(startWp);
    List<MatchedWaypoint> copyVias = new ArrayList<>();
    if (vias != null) {
      for (MatchedWaypoint v : vias) {
        copyVias.add(copyWaypoint(v));
      }
    }
    this.vias = Collections.unmodifiableList(copyVias);
    this.endWp = copyWaypoint(endWp);
  }

  public MatchedWaypoint getStartWp() {
    return startWp;
  }

  public List<MatchedWaypoint> getVias() {
    return vias;
  }

  public MatchedWaypoint getEndWp() {
    return endWp;
  }

  /** Return the complete ordered list of waypoints (start + vias + end). */
  public List<MatchedWaypoint> getWaypoints() {
    List<MatchedWaypoint> all = new ArrayList<>(vias.size() + 2);
    all.add(startWp);
    all.addAll(vias);
    all.add(endWp);
    return Collections.unmodifiableList(all);
  }

  /** Return the number of legs in the skeleton. */
  public int getLegCount() {
    return vias.size() + 1;
  }

  private static OsmNode copyNode(OsmNode src) {
    if (src == null) {
      return null;
    }
    OsmNode n = new OsmNode();
    n.ilon = src.ilon;
    n.ilat = src.ilat;
    n.selev = src.selev;
    return n;
  }

  /** Deep copy of a MatchedWaypoint. */
  public static MatchedWaypoint copyWaypoint(MatchedWaypoint src) {
    if (src == null) {
      return null;
    }
    MatchedWaypoint c = new MatchedWaypoint();
    c.name = src.name;
    c.node1 = copyNode(src.node1);
    c.node2 = copyNode(src.node2);
    c.crosspoint = copyNode(src.crosspoint);
    c.waypoint = copyNode(src.waypoint);
    c.correctedpoint = copyNode(src.correctedpoint);
    c.radius = src.radius;
    c.wpttype = src.wpttype;
    c.generated = src.generated;
    c.indexInTrack = src.indexInTrack;
    c.directionToNext = src.directionToNext;
    c.directionDiff = src.directionDiff;
    c.hasUpdate = src.hasUpdate;
    if (src.wayDescription != null) {
      c.wayDescription = src.wayDescription.clone();
    }
    if (src.wayNearest != null) {
      c.wayNearest = new ArrayList<>(src.wayNearest);
    }
    return c;
  }
}
