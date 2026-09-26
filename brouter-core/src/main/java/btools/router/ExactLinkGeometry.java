package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.GeometryDecoder;
import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import btools.mapaccess.OsmTransferNode;
import btools.mapaccess.TurnRestriction;
import btools.util.ByteDataWriter;
import btools.util.CheapRuler;

/** Replays explicit, quantized via samples on an existing graph link without dropping them. */
final class ExactLinkGeometry {
  final int advance;
  final byte[] geometry;

  private ExactLinkGeometry(int advance, byte[] geometry) {
    this.advance = advance;
    this.geometry = geometry;
  }

  static ExactLinkGeometry match(OsmLink link, OsmNode source, OsmNode target,
                                OsmTrack track, int start, GeometryDecoder decoder, Runnable checkBudget) {
    if (track.getMatchedWaypoints() == null || track.nodes.get(start).getIdFromPos() != source.getIdFromPos()) return null;
    List<OsmNode> shape = new ArrayList<>();
    shape.add(source);
    if (link.geometry != null) {
      OsmTransferNode node = decoder.decodeGeometry(link.geometry, source, target, link.isReverse(source));
      while (node != null) {
        checkBudget.run();
        OsmNode copy = new OsmNode(node.ilon, node.ilat);
        copy.selev = node.selev;
        shape.add(copy);
        node = node.next;
      }
    }
    shape.add(target);
    int index = start;
    List<Short> elevations = new ArrayList<>();
    elevations.add(source.selev);
    boolean inserted = false;
    for (int s = 1; s < shape.size(); s++) {
      checkBudget.run();
      OsmNode from = shape.get(s - 1);
      OsmNode to = shape.get(s);
      double previousFraction = 0;
      while (++index < track.nodes.size() && track.nodes.get(index).getIdFromPos() != to.getIdFromPos()) {
        checkBudget.run();
        OsmPathElement point = track.nodes.get(index);
        if (!isViaSample(point, source, target, track.getMatchedWaypoints()) || !onSection(point, from, to)) {
          return null;
        }
        double fraction = sectionFraction(point, from, to);
        double quantization = 2.0 / Math.max(Math.abs((double) to.ilon - from.ilon), Math.abs((double) to.ilat - from.ilat));
        if (fraction + quantization < previousFraction) return null;
        previousFraction = fraction;
        elevations.add(from.selev == Short.MIN_VALUE || to.selev == Short.MIN_VALUE ? Short.MIN_VALUE
          : (short) (from.selev * (1 - fraction) + to.selev * fraction));
        inserted = true;
      }
      if (index >= track.nodes.size()) return null;
      elevations.add(to.selev);
    }
    if (!inserted) return null;
    // Preserve direction-dependent tags: encode in the original link's storage direction.
    ByteDataWriter writer = new ByteDataWriter(new byte[(index - start) * 15]);
    boolean reverse = link.isReverse(source);
    int lon = reverse ? target.ilon : source.ilon;
    int lat = reverse ? target.ilat : source.ilat;
    int elevation = reverse ? target.selev : source.selev;
    for (int n = reverse ? index - 1 : start + 1; reverse ? n > start : n < index; n += reverse ? -1 : 1) {
      checkBudget.run();
      OsmPathElement point = track.nodes.get(n);
      writer.writeVarLengthSigned(point.getILon() - lon);
      writer.writeVarLengthSigned(point.getILat() - lat);
      writer.writeVarLengthSigned(elevations.get(n - start) - elevation);
      lon = point.getILon();
      lat = point.getILat();
      elevation = elevations.get(n - start);
    }
    return new ExactLinkGeometry(index, writer.toByteArray());
  }

  static long nativeNeighbor(OsmLink link, OsmNode source, OsmNode target,
                             boolean first, GeometryDecoder decoder) {
    long neighbor = first ? target.getIdFromPos() : source.getIdFromPos();
    if (link.geometry != null) {
      OsmTransferNode point = decoder.decodeGeometry(link.geometry, source, target, link.isReverse(source));
      while (point != null) {
        neighbor = ((long) point.ilon << 32) | point.ilat;
        if (first) break;
        point = point.next;
      }
    }
    return neighbor;
  }

  static boolean permitsTurn(OsmNode junction, long from, long to, RoutingContext context) {
    if (!context.considerTurnRestrictions) return true;
    return !TurnRestriction.isTurnForbidden(junction.firstRestriction,
      (int) ((context.inverseDirection ? to : from) >> 32), (int) (context.inverseDirection ? to : from),
      (int) ((context.inverseDirection ? from : to) >> 32), (int) (context.inverseDirection ? from : to),
      context.bikeMode || context.footMode, context.carMode);
  }

  private static boolean isViaSample(OsmPathElement point, OsmNode source, OsmNode target,
                                     List<MatchedWaypoint> waypoints) {
    for (MatchedWaypoint wp : waypoints) {
      if (wp.node1 == null || wp.node2 == null || wp.crosspoint == null) continue;
      long a = wp.node1.getIdFromPos();
      long b = wp.node2.getIdFromPos();
      if (!((a == source.getIdFromPos() && b == target.getIdFromPos())
          || (b == source.getIdFromPos() && a == target.getIdFromPos()))) continue;
      // A waypoint is projected again when each leg is detailed, with integer truncation.
      for (OsmNode crosspoint : new OsmNode[]{wp.crosspoint, wp.originalCrosspoint}) {
        if (crosspoint != null && Math.abs((long) point.getILon() - crosspoint.ilon) <= 2
            && Math.abs((long) point.getILat() - crosspoint.ilat) <= 2) return true;
      }
    }
    return false;
  }

  private static double sectionFraction(OsmPathElement point, OsmNode from, OsmNode to) {
    double[] scales = CheapRuler.getLonLatToMeterScales((from.ilat + to.ilat) >> 1);
    double dx = (to.ilon - from.ilon) * scales[0];
    double dy = (to.ilat - from.ilat) * scales[1];
    return ((point.getILon() - from.ilon) * scales[0] * dx
      + (point.getILat() - from.ilat) * scales[1] * dy) / (dx * dx + dy * dy);
  }

  private static boolean onSection(OsmPathElement point, OsmNode from, OsmNode to) {
    if (from.getIdFromPos() == to.getIdFromPos()) return false;
    // Native waypoint projection truncates each positive integer coordinate.
    // Test intersection with that coordinate cell. Perpendicular projection
    // followed by separate axis tolerances can reject a legitimate cell on
    // an oblique road (the Mallorca regression), even within one grid unit.
    double lower = 0;
    double upper = 1;
    for (int axis = 0; axis < 2; axis++) {
      double origin = axis == 0 ? from.ilon : from.ilat;
      double delta = (axis == 0 ? to.ilon : to.ilat) - origin;
      double value = axis == 0 ? point.getILon() : point.getILat();
      double cellEnd = value + 1.000001; // one integer cell plus floating-point roundoff
      if (delta == 0) {
        if (origin < value || origin > cellEnd) return false;
      } else {
        double first = (value - origin) / delta;
        double second = (cellEnd - origin) / delta;
        lower = Math.max(lower, Math.min(first, second));
        upper = Math.min(upper, Math.max(first, second));
      }
    }
    return lower <= upper;
  }
}
