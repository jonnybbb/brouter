package btools.router;

import org.junit.Test;

import btools.mapaccess.TurnRestriction;

import static org.junit.Assert.*;

public class CleanupTurnRestrictionTest {
  private final OsmPathElement before = OsmPathElement.create(100, 100, (short) 0, null);
  private final OsmPathElement mouth = OsmPathElement.create(200, 100, (short) 0, null);
  private final OsmPathElement after = OsmPathElement.create(300, 100, (short) 0, null);

  @Test
  public void copiedRestrictionSurvivesCloningAndRejectsForbiddenJoin() {
    TurnRestriction rule = new TurnRestriction();
    rule.fromLon = 100;
    rule.fromLat = 100;
    rule.toLon = 300;
    rule.toLat = 100;
    MessageData message = new MessageData();
    message.recordTurnRestrictions(mouth.getIdFromPos(), rule, true);
    rule.toLon = 999;
    RoutingContext context = new RoutingContext();
    context.considerTurnRestrictions = true;
    assertFalse(message.permitsCleanupTurn(before, mouth, after, context));
    assertFalse(message.copy().permitsCleanupTurn(before, mouth, after, context));
    context.inverseDirection = true;
    assertTrue(message.permitsCleanupTurn(before, mouth, after, context));
  }

  @Test
  public void honorsBicycleExceptionAndPointProvenance() {
    TurnRestriction rule = new TurnRestriction();
    rule.fromLon = 100;
    rule.fromLat = 100;
    rule.toLon = 300;
    rule.toLat = 100;
    rule.exceptions = 1;
    MessageData message = new MessageData();
    message.recordTurnRestrictions(mouth.getIdFromPos(), rule, true);
    RoutingContext context = new RoutingContext();
    context.considerTurnRestrictions = true;
    context.bikeMode = true;
    assertTrue(message.permitsCleanupTurn(before, mouth, after, context));
    assertFalse(message.permitsCleanupTurn(before, after, after, context));
  }

  @Test
  public void cannotCreateMidEdgeUTurnOrUseUnknownRestrictions() {
    MessageData message = new MessageData();
    RoutingContext context = new RoutingContext();
    assertFalse(message.permitsCleanupTurn(before, mouth, after, context));
    message.recordTurnRestrictions(mouth.getIdFromPos(), null, false);
    assertFalse(message.permitsCleanupTurn(before, mouth, after, context));
    message.recordTurnRestrictions(mouth.getIdFromPos(), null, true);
    assertTrue(message.permitsCleanupTurn(before, mouth, after, context));
  }
}
