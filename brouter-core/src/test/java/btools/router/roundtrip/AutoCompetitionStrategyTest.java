package btools.router.roundtrip;

import org.junit.Assert;
import org.junit.Test;

public class AutoCompetitionStrategyTest {

  /**
   * The speculative permit pool cannot see server load, so its default must
   * never commit more than half the machine (bounded at 4) to speculation —
   * incoming requests always find free cores.
   */
  @Test
  public void speculativePermitDefaultCommitsAtMostHalfTheMachine() {
    Assert.assertEquals(0, AutoCompetitionStrategy.defaultParallelAutoPermits(1));
    Assert.assertEquals(1, AutoCompetitionStrategy.defaultParallelAutoPermits(2));
    Assert.assertEquals(2, AutoCompetitionStrategy.defaultParallelAutoPermits(4));
    Assert.assertEquals(4, AutoCompetitionStrategy.defaultParallelAutoPermits(8));
    Assert.assertEquals(4, AutoCompetitionStrategy.defaultParallelAutoPermits(16));
  }
}
