package btools.router;

import java.util.Collection;

import org.junit.runners.Parameterized;

/** Variety-seed calibration shard for {@link LoopTestRegion#BASEL}. See {@link VarietySeedCalibrationBase}. */
public class VarietySeedCalibrationBaselTest extends VarietySeedCalibrationBase {
  @Parameterized.Parameters(name = "{5}")
  public static Collection<Object[]> data() {
    return dataForRegion(LoopTestRegion.BASEL);
  }
}
