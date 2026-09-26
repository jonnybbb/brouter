/**
 * Information on matched way point
 *
 * @author ab
 */
package btools.router;


public final class MessageData implements Cloneable {
  private btools.mapaccess.TurnRestriction cleanupRestrictions;
  private long restrictionNode;
  private boolean restrictionsRecorded;
  private boolean restrictionAtJunction;

  void recordTurnRestrictions(long node, btools.mapaccess.TurnRestriction restrictions, boolean atJunction) {
    cleanupRestrictions = null;
    restrictionNode = node;
    restrictionsRecorded = true;
    restrictionAtJunction = atJunction;
    btools.mapaccess.TurnRestriction tail = null;
    for (btools.mapaccess.TurnRestriction source = restrictions; source != null; source = source.next) {
      btools.mapaccess.TurnRestriction copy = new btools.mapaccess.TurnRestriction();
      copy.fromLon = source.fromLon;
      copy.fromLat = source.fromLat;
      copy.toLon = source.toLon;
      copy.toLat = source.toLat;
      copy.isPositive = source.isPositive;
      copy.exceptions = source.exceptions;
      if (tail == null) cleanupRestrictions = copy;
      else tail.next = copy;
      tail = copy;
    }
  }

  /** Restriction snapshot stays tied to its detailed point after origin-pointer repair. */
  public boolean permitsCleanupTurn(OsmPathElement before, OsmPathElement node, OsmPathElement after, RoutingContext rc) {
    return restrictionsRecorded && restrictionAtJunction && restrictionNode == node.getIdFromPos()
      && (!rc.considerTurnRestrictions || !btools.mapaccess.TurnRestriction.isTurnForbidden(cleanupRestrictions,
        rc.inverseDirection ? after.getILon() : before.getILon(),
        rc.inverseDirection ? after.getILat() : before.getILat(),
        rc.inverseDirection ? before.getILon() : after.getILon(),
        rc.inverseDirection ? before.getILat() : after.getILat(), rc.bikeMode || rc.footMode, rc.carMode));
  }


  /**
   * Way-tag key/value dump of the matched way (read-only round-trip seam).
   */
  public String getWayKeyValues() {
    return wayKeyValues;
  }

  /**
   * Profile costfactor recorded for this edge (read-only round-trip seam).
   */
  public float getCostfactor() {
    return costfactor;
  }

  // Original detailed segment, preserved through cleanup and cloning. Analysis only.
  private long measuredFrom;
  private long measuredTo;
  private float measuredMovingSeconds = Float.NaN;
  private boolean measuredSegment;

  void recordSegment(long from, long to, float movingSeconds) {
    measuredFrom = from;
    measuredTo = to;
    measuredMovingSeconds = movingSeconds;
    measuredSegment = true;
  }

  boolean describesSegment(long from, long to) {
    return measuredSegment && measuredFrom == from && measuredTo == to;
  }

  float measuredMovingSeconds() {
    return measuredMovingSeconds;
  }

  int linkdist = 0;
  int linkelevationcost = 0;
  int linkturncost = 0;
  int linknodecost = 0;
  int linkinitcost = 0;

  float costfactor;
  int priorityclassifier;
  int classifiermask;
  float turnangle;
  float gradient; // slope in percent (rise/run * 100)
  double deltaH; // elevation change in meters for this section
  String wayKeyValues;
  String nodeKeyValues;

  int lon;
  int lat;
  short ele;

  float time;
  float energy;

  // speed profile
  int vmaxExplicit = -1;
  int vmax = -1;
  int vmin = -1;
  int vnode0 = 999;
  int vnode1 = 999;
  int extraTime = 0;

  String toMessage() {
    if (wayKeyValues == null) {
      return null;
    }

    int iCost = (int) (costfactor * 1000 + 0.5f);
    int iGradient = (int) (gradient * 10 + (gradient >= 0 ? 0.5f : -0.5f));
    return (lon - 180000000) + "\t"
      + (lat - 90000000) + "\t"
      + ele / 4 + "\t"
      + linkdist + "\t"
      + iCost + "\t"
      + linkelevationcost
      + "\t" + linkturncost
      + "\t" + linknodecost
      + "\t" + linkinitcost
      + "\t" + wayKeyValues
      + "\t" + (nodeKeyValues == null ? "" : nodeKeyValues)
      + "\t" + ((int) time)
      + "\t" + ((int) energy)
      + "\t" + iGradient;
  }

  void add(MessageData d) {
    // recompute gradient as weighted average by distance
    deltaH += d.deltaH;
    int totalDist = linkdist + d.linkdist;
    if (totalDist > 0) {
      gradient = (float) (deltaH / totalDist * 100.);
    }
    linkdist = totalDist;
    linkelevationcost += d.linkelevationcost;
    linkturncost += d.linkturncost;
    linknodecost += d.linknodecost;
    linkinitcost += d.linkinitcost;
  }

  MessageData copy() {
    try {
      return (MessageData) clone();
    } catch (CloneNotSupportedException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public String toString() {
    return "dist=" + linkdist + " prio=" + priorityclassifier + " turn=" + turnangle;
  }

  public int getPrio() {
    return priorityclassifier;
  }

  public boolean isBadOneway() {
    return (classifiermask & 1) != 0;
  }

  public boolean isGoodOneway() {
    return (classifiermask & 2) != 0;
  }

  public boolean isRoundabout() {
    return (classifiermask & 4) != 0;
  }

  public boolean isLinktType() {
    return (classifiermask & 8) != 0;
  }

  public boolean isGoodForCars() {
    return (classifiermask & 16) != 0;
  }

}
