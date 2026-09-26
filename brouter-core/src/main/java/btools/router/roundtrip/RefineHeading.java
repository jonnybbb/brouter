package btools.router.roundtrip;

import btools.router.RoutingContext;

/** Preserve request heading state while applying it only to a skeleton's opening leg. */
final class RefineHeading implements AutoCloseable {
  private final RoutingContext context;
  private final boolean forced;
  private final boolean valid;

  RefineHeading(RoutingContext context, boolean openingLeg) {
    this.context = context;
    forced = context.forceUseStartDirection;
    valid = context.startDirectionValid;
    context.forceUseStartDirection = forced && openingLeg;
    context.startDirectionValid = false;
  }

  @Override
  public void close() {
    context.forceUseStartDirection = forced;
    context.startDirectionValid = valid;
  }
}
