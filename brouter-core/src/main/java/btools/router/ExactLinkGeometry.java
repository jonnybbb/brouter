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
    return match(link, source, target, track, start, null, null, decoder, checkBudget);
  }

  /** Match every native sample, permitting only attributed vias and clipped endpoints. */
  static ExactLinkGeometry match(OsmLink link, OsmNode source, OsmNode target,
                                OsmTrack track, int start, MatchedWaypoint opening, MatchedWaypoint closing,
                                GeometryDecoder decoder, Runnable checkBudget) {
    if (start < 0 || start >= track.nodes.size() - 1) return null;
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
    OsmPathElement first = track.nodes.get(start);
    for (int section = 0; section < shape.size() - 1; section++) {
      checkBudget.run();
      if (first.getIdFromPos() != shape.get(section).getIdFromPos()
          && !(start == 0 && matchesEdge(opening, source, target)
            && onSection(first, shape.get(section), shape.get(section + 1)))) continue;
      if (section > 0 && start != 0) return null;
      ExactLinkGeometry match = matchFromSection(link, source, target, track, start, closing,
        shape, section, checkBudget);
      if (match != null) return match;
    }
    return null;
  }

  private static ExactLinkGeometry matchFromSection(OsmLink link, OsmNode source, OsmNode target,
      OsmTrack track, int start, MatchedWaypoint closing, List<OsmNode> shape, int firstSection,
      Runnable checkBudget) {
    List<OsmNode> replay = new ArrayList<>(shape.subList(0, firstSection + 1));
    int index = start;
    boolean inserted = false;
    for (int section = firstSection; section < shape.size() - 1; section++) {
      checkBudget.run();
      OsmNode from = shape.get(section);
      OsmNode to = shape.get(section + 1);
      double previousFraction = section == firstSection
        ? sectionFraction(track.nodes.get(start), from, to) : 0;
      while (++index < track.nodes.size()) {
        checkBudget.run();
        OsmPathElement point = track.nodes.get(index);
        if (point.getIdFromPos() == to.getIdFromPos()) break;
        if (!onSection(point, from, to)) return null;
        double fraction = sectionFraction(point, from, to);
        double quantization = 2.0 / Math.max(Math.abs((double) to.ilon - from.ilon), Math.abs((double) to.ilat - from.ilat));
        if (fraction + quantization < previousFraction) return null;
        previousFraction = fraction;
        if (index == track.nodes.size() - 1 && matchesEdge(closing, source, target)) {
          replay.addAll(shape.subList(section + 1, shape.size()));
          return new ExactLinkGeometry(index, inserted ? encode(link, source, target, replay, checkBudget) : link.geometry);
        }
        if (!isViaSample(point, source, target, track.getMatchedWaypoints())) return null;
        OsmNode sample = new OsmNode(point.getILon(), point.getILat());
        sample.selev = from.selev == Short.MIN_VALUE || to.selev == Short.MIN_VALUE ? Short.MIN_VALUE
          : (short) (from.selev * (1 - fraction) + to.selev * fraction);
        replay.add(sample);
        inserted = true;
      }
      if (index >= track.nodes.size()) return null;
      replay.add(to);
      if (index == track.nodes.size() - 1 && section < shape.size() - 2) {
        if (!matchesEdge(closing, source, target)) return null;
        replay.addAll(shape.subList(section + 2, shape.size()));
        return new ExactLinkGeometry(index, inserted ? encode(link, source, target, replay, checkBudget) : link.geometry);
      }
    }
    return new ExactLinkGeometry(index, inserted ? encode(link, source, target, replay, checkBudget) : link.geometry);
  }

  private static boolean matchesEdge(MatchedWaypoint wp, OsmNode source, OsmNode target) {
    if (wp == null || wp.node1 == null || wp.node2 == null) return false;
    long a = wp.node1.getIdFromPos();
    long b = wp.node2.getIdFromPos();
    return (a == source.getIdFromPos() && b == target.getIdFromPos())
      || (b == source.getIdFromPos() && a == target.getIdFromPos());
  }

  private static byte[] encode(OsmLink link, OsmNode source, OsmNode target,
                               List<OsmNode> replay, Runnable checkBudget) {
    // Preserve direction-dependent tags by retaining the native storage direction.
    ByteDataWriter writer = new ByteDataWriter(new byte[replay.size() * 15]);
    boolean reverse = link.isReverse(source);
    int lon = reverse ? target.ilon : source.ilon;
    int lat = reverse ? target.ilat : source.ilat;
    int elevation = reverse ? target.selev : source.selev;
    for (int n = reverse ? replay.size() - 2 : 1; reverse ? n > 0 : n < replay.size() - 1; n += reverse ? -1 : 1) {
      checkBudget.run();
      OsmNode point = replay.get(n);
      writer.writeVarLengthSigned(point.ilon - lon);
      writer.writeVarLengthSigned(point.ilat - lat);
      writer.writeVarLengthSigned(point.selev - elevation);
      lon = point.ilon;
      lat = point.ilat;
      elevation = point.selev;
    }
    return writer.toByteArray();
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
    if (waypoints == null) return false;
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
