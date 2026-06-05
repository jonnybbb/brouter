package btools.router;

import java.util.Collection;

import org.junit.runners.Parameterized;

/** Loop-quality shard for {@link LoopTestRegion#ALPINE_INNSBRUCK}. See {@link LoopQualityTestBase}. */
public class LoopQualityAlpineInnsbruckTest extends LoopQualityTestBase {
  @Parameterized.Parameters(name = "{5}")
  public static Collection<Object[]> data() {
    return dataForRegion(LoopTestRegion.ALPINE_INNSBRUCK);
  }
}
