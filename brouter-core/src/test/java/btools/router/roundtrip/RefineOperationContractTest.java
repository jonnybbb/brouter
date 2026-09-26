package btools.router.roundtrip;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Proxy;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.Test;

import btools.mapaccess.MatchedWaypoint;
import btools.router.RoundTripFixture;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;

import static org.junit.Assert.*;

public class RefineOperationContractTest {
  private static RoutingEngine fixture() {
    return RoundTripFixture.engine("gravel", 90, 1000, rc -> {
      rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
      rc.roundTripStrictQuality = false;
    });
  }

  @Test
  public void pricingDistinguishesCancellationAndTimeoutAndRestoresDeadline() {
    RoutingEngine engine = fixture();
    RoundTripEngineOps ops = engine.roundTripOps();
    RoundTripResult result = engine.getLastRoundTripResult();
    long originalDeadline = ops.refinementDeadline();
    LoopPrice timeout = LoopCostOracle.evaluate(ops, result.getTrack(), result.getMatchedWaypoints(), 1);
    assertEquals(FinalizationOutcome.TIMEOUT, timeout.outcome);
    assertEquals(originalDeadline, ops.refinementDeadline());
    engine.terminate();
    LoopPrice cancelled = LoopCostOracle.evaluate(ops, result.getTrack(), result.getMatchedWaypoints(), 1);
    assertEquals(FinalizationOutcome.CANCELLED, cancelled.outcome);
    assertEquals(originalDeadline, ops.refinementDeadline());
  }

  @Test
  public void cancellationDuringLastWalkCannotBecomeSuccessfulPrice() {
    RoutingEngine engine = fixture();
    RoundTripEngineOps delegate = engine.roundTripOps();
    AtomicBoolean walked = new AtomicBoolean();
    RoundTripEngineOps ops = (RoundTripEngineOps) Proxy.newProxyInstance(
      RoundTripEngineOps.class.getClassLoader(), new Class<?>[]{RoundTripEngineOps.class}, (proxy, method, args) -> {
        if ("walkPathCost".equals(method.getName())) {
          walked.set(true);
          engine.terminate();
          return 123;
        }
        try {
          return method.invoke(delegate, args);
        } catch (InvocationTargetException e) {
          throw e.getCause();
        }
      });
    RoundTripResult result = engine.getLastRoundTripResult();
    LoopPrice price = LoopCostOracle.evaluate(ops, result.getTrack(), result.getMatchedWaypoints(),
      System.currentTimeMillis() + 10000);
    assertTrue(walked.get());
    assertEquals(FinalizationOutcome.CANCELLED, price.outcome);
    assertEquals(0, ops.refinementDeadline());
  }

  @Test
  public void activeDeadlineInterruptsTheWalkAndRestoresScope() {
    RoutingEngine engine = fixture();
    RoundTripEngineOps delegate = engine.roundTripOps();
    AtomicBoolean walked = new AtomicBoolean();
    RoundTripEngineOps ops = (RoundTripEngineOps) Proxy.newProxyInstance(
      RoundTripEngineOps.class.getClassLoader(), new Class<?>[]{RoundTripEngineOps.class}, (proxy, method, args) -> {
        if ("walkPathCost".equals(method.getName())) {
          walked.set(true);
          delegate.setRefinementDeadline(1);
        }
        try {
          return method.invoke(delegate, args);
        } catch (InvocationTargetException e) {
          throw e.getCause();
        }
      });
    RoundTripResult result = engine.getLastRoundTripResult();
    LoopPrice price = LoopCostOracle.evaluate(ops, result.getTrack(), result.getMatchedWaypoints(),
      System.currentTimeMillis() + 10000);
    assertTrue(walked.get());
    assertEquals(FinalizationOutcome.TIMEOUT, price.outcome);
    assertEquals(0, ops.refinementDeadline());
  }

  @Test
  public void rawLegRoutingUsesForcedHeadingOnlyAtStartAndRestoresOnFailure() {
    RoutingEngine engine = fixture();
    RoundTripEngineOps delegate = engine.roundTripOps();
    RoutingContext context = delegate.routingContext();
    context.forceUseStartDirection = true;
    context.startDirectionValid = true;
    AtomicBoolean expectedForced = new AtomicBoolean(true);
    RoundTripEngineOps ops = (RoundTripEngineOps) Proxy.newProxyInstance(
      RoundTripEngineOps.class.getClassLoader(), new Class<?>[]{RoundTripEngineOps.class}, (proxy, method, args) -> {
        if ("findTrackTimed".equals(method.getName())) {
          assertEquals(expectedForced.get(), context.forceUseStartDirection);
          assertFalse(context.startDirectionValid);
          throw new IllegalArgumentException("unroutable test leg");
        }
        try {
          return method.invoke(delegate, args);
        } catch (InvocationTargetException e) {
          throw e.getCause();
        }
      });
    DefaultLegEvaluator evaluator = new DefaultLegEvaluator(ops);
    List<MatchedWaypoint> waypoints = delegate.matchedWaypoints();
    assertTrue(waypoints.size() >= 3);
    assertNull(evaluator.route(waypoints.get(0), waypoints.get(1), 1000));
    assertTrue(context.forceUseStartDirection);
    assertTrue(context.startDirectionValid);
    expectedForced.set(false);
    assertNull(evaluator.route(waypoints.get(1), waypoints.get(2), 1000));
    assertTrue(context.forceUseStartDirection);
    assertTrue(context.startDirectionValid);
  }

  @Test
  public void wholeTrackPriceDoesNotDependOnPreviousLegHeadingState() {
    RoutingEngine engine = fixture();
    RoundTripEngineOps ops = engine.roundTripOps();
    RoundTripResult result = engine.getLastRoundTripResult();
    RoutingContext context = ops.routingContext();
    context.forceUseStartDirection = true;
    context.startDirectionValid = false;
    double first = LoopCostOracle.price(ops, result.getTrack(), result.getMatchedWaypoints());
    assertFalse(context.startDirectionValid);
    context.startDirectionValid = true;
    double second = LoopCostOracle.price(ops, result.getTrack(), result.getMatchedWaypoints());
    assertTrue(context.startDirectionValid);
    assertTrue(first > 0);
    assertEquals(first, second, 0);
  }

  @Test
  public void measurementCapturesOriginalAndResultFromOneRequest() {
    RoutingEngine original = fixture();
    long signature = LoopCostOracle.geometrySignature(original.getFoundTrack());
    String previous = System.getProperty("loop.refine.measure");
    try {
      System.setProperty("loop.refine.measure", "true");
      RoutingEngine refined = RoundTripFixture.engine("gravel", 90, 1000, rc -> {
        rc.roundTripAlgorithm = RoundTripAlgorithm.GREEDY;
        rc.roundTripStrictQuality = false;
        rc.roundTripRefine = "local";
        rc.roundTripRefineEvals = 1;
      });
      RefineDiagnostics d = refined.getLastRefineDiagnostics();
      assertNotNull(d);
      assertNotNull(d.baseline);
      assertNotNull(d.result);
      assertEquals(signature, d.baseline.geometrySignature);
      assertEquals(LoopCostOracle.geometrySignature(refined.getFoundTrack()), d.result.geometrySignature);
      assertEquals(2 * Math.PI * 1000, d.requestedDistance, 0.001);
      if (!d.refineApplied) assertEquals(d.baseline.geometrySignature, d.result.geometrySignature);
    } finally {
      if (previous == null) System.clearProperty("loop.refine.measure");
      else System.setProperty("loop.refine.measure", previous);
    }
  }
}
