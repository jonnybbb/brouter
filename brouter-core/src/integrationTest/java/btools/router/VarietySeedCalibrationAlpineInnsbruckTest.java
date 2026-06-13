package btools.router;

import java.util.Collection;

import org.junit.runners.Parameterized;

/** Variety-seed calibration shard for {@link LoopTestRegion#ALPINE_INNSBRUCK}. See {@link VarietySeedCalibrationBase}. */
public class VarietySeedCalibrationAlpineInnsbruckTest extends VarietySeedCalibrationBase {
  @Parameterized.Parameters(name = "{5}")
  public static Collection<Object[]> data() {
    return dataForRegion(LoopTestRegion.ALPINE_INNSBRUCK);
  }
}
