package btools.router;

import java.util.Arrays;

import org.junit.Test;

import btools.mapaccess.GeometryDecoder;
import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import btools.mapaccess.OsmTransferNode;
import btools.util.ByteDataWriter;

import static org.junit.Assert.*;

public class ExactLinkGeometryTest {
  private final OsmNode source = new OsmNode(180000000, 140000000);
  private final OsmNode target = new OsmNode(180002000, 140000000);

  public ExactLinkGeometryTest() {
    source.selev = 400;
    target.selev = 800;
  }

  private OsmTrack track(int... offsets) {
    OsmTrack track = new OsmTrack();
    for (int offset : offsets) {
      track.nodes.add(OsmPathElement.create(source.ilon + offset, source.ilat, (short) (offset / 10), null));
    }
    MatchedWaypoint via = new MatchedWaypoint();
    via.node1 = source;
    via.node2 = target;
    via.crosspoint = target;
    via.originalCrosspoint = new OsmNode(source.ilon + 500, source.ilat);
    track.setMatchedWaypoints(Arrays.asList(via));
    return track;
  }

  @Test
  public void preservesLiteralViaSamplesButUsesGraphElevationInBothDirections() {
    for (boolean reverse : new boolean[]{false, true}) {
      OsmLink link = reverse ? new OsmLink(target, source) : new OsmLink(source, target);
      OsmTrack track = track(0, 500, 501, 2000);
      ExactLinkGeometry match = ExactLinkGeometry.match(link, source, target, track, 0, new GeometryDecoder(), () -> { });
      assertNotNull(match);
      assertEquals(3, match.advance);
      OsmTransferNode node = new GeometryDecoder().decodeGeometry(match.geometry, source, target, reverse);
      assertEquals(source.ilon + 500, node.ilon);
      assertEquals("Map elevation must be interpolated, not taken from display metadata", 500, node.selev);
      assertEquals(source.ilon + 501, node.next.ilon);
      assertNull(node.next.next);
      assertNull("Matching must not mutate the graph link", link.geometry);
    }
  }

  @Test
  public void rejectsUnattributedAndOffEdgeSamples() {
    OsmLink link = new OsmLink(source, target);
    assertNull(ExactLinkGeometry.match(link, source, target, track(0, 800, 2000), 0, new GeometryDecoder(), () -> { }));
    OsmTrack offEdge = track(0, 500, 2000);
    offEdge.nodes.set(1, OsmPathElement.create(source.ilon + 500, source.ilat + 2, (short) 50, null));
    assertNull(ExactLinkGeometry.match(link, source, target, offEdge, 0, new GeometryDecoder(), () -> { }));
  }

  @Test
  public void doesNotOmitOriginalTransferGeometry() {
    OsmLink link = new OsmLink(source, target);
    ByteDataWriter writer = new ByteDataWriter(new byte[30]);
    writer.writeVarLengthSigned(1000);
    writer.writeVarLengthSigned(0);
    writer.writeVarLengthSigned(100 - source.selev);
    link.geometry = writer.toByteArray();
    assertNull(ExactLinkGeometry.match(link, source, target, track(0, 500, 2000), 0, new GeometryDecoder(), () -> { }));
    assertNotNull(ExactLinkGeometry.match(link, source, target, track(0, 500, 1000, 2000), 0, new GeometryDecoder(), () -> { }));
  }

  @Test
  public void addedViaCannotHideNativeTurnRestriction() {
    OsmLink link = new OsmLink(source, target);
    ByteDataWriter writer = new ByteDataWriter(new byte[30]);
    writer.writeVarLengthSigned(1000);
    writer.writeVarLengthSigned(0);
    writer.writeVarLengthSigned(200);
    link.geometry = writer.toByteArray();
    long nativeFrom = ExactLinkGeometry.nativeNeighbor(link, source, target, false, new GeometryDecoder());
    assertEquals(((long) (source.ilon + 1000) << 32) | source.ilat, nativeFrom);
    btools.mapaccess.TurnRestriction restriction = new btools.mapaccess.TurnRestriction();
    restriction.fromLon = source.ilon + 1000;
    restriction.fromLat = source.ilat;
    restriction.toLon = target.ilon;
    restriction.toLat = target.ilat + 500;
    target.firstRestriction = restriction;
    RoutingContext context = new RoutingContext();
    context.considerTurnRestrictions = true;
    long outgoing = ((long) target.ilon << 32) | (target.ilat + 500);
    assertFalse(ExactLinkGeometry.permitsTurn(target, nativeFrom, outgoing, context));
    long insertedVia = ((long) (source.ilon + 1500) << 32) | source.ilat;
    assertTrue("Using a rendered via instead would miss this restriction",
      ExactLinkGeometry.permitsTurn(target, insertedVia, outgoing, context));
  }

  @Test
  public void acceptsIntegerCellProducedByNativeProjectionOnObliqueRoad() {
    OsmNode from = new OsmNode(182792913, 129643725);
    OsmNode to = new OsmNode(182793355, 129643558);
    from.selev = 400;
    to.selev = 800;
    OsmNodeNamed requested = new OsmNodeNamed(new OsmNode(182793050, 129643672));
    requested.radius = 1.5;
    RoutingContext context = new RoutingContext();
    context.setWaypoint(requested, true);
    context.calcDistance(from.ilon, from.ilat, to.ilon, to.ilat);
    assertTrue(context.shortestmatch);
    OsmNode clipped = new OsmNode(context.ilonshortest, context.ilatshortest);
    context.unsetWaypoint();
    assertEquals(182793050, clipped.ilon);
    assertEquals(129643672, clipped.ilat);
    OsmTrack track = new OsmTrack();
    for (OsmNode node : Arrays.asList(from, clipped, to)) {
      track.nodes.add(OsmPathElement.create(node.ilon, node.ilat, (short) 0, null));
    }
    MatchedWaypoint via = new MatchedWaypoint();
    via.node1 = from;
    via.node2 = to;
    via.crosspoint = clipped;
    track.setMatchedWaypoints(Arrays.asList(via));
    OsmLink link = new OsmLink(from, to);
    ExactLinkGeometry match = ExactLinkGeometry.match(link, from, to, track, 0, new GeometryDecoder(), () -> { });
    assertNotNull("The router's own projected sample must be replayable", match);
    assertEquals(2, match.advance);
    // Keep attribution but move the sample to a different coordinate cell.
    track.nodes.set(1, OsmPathElement.create(clipped.ilon, clipped.ilat - 2, (short) 0, null));
    assertNull(ExactLinkGeometry.match(link, from, to, track, 0, new GeometryDecoder(), () -> { }));
  }

  @Test(expected = IllegalStateException.class)
  public void observesBudgetDuringMatching() {
    ExactLinkGeometry.match(new OsmLink(source, target), source, target, track(0, 500, 2000), 0,
      new GeometryDecoder(), () -> { throw new IllegalStateException("cancelled"); });
  }
}
