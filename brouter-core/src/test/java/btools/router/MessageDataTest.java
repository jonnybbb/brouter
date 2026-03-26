package btools.router;

import org.junit.Assert;
import org.junit.Test;

public class MessageDataTest {

  @Test
  public void gradientDefaultsToZero() {
    MessageData md = new MessageData();
    Assert.assertEquals(0f, md.gradient, 0.001f);
    Assert.assertEquals(0.0, md.deltaH, 0.001);
  }

  @Test
  public void toMessageIncludesGradientColumn() {
    MessageData md = createMessage(1000, 50.0, "highway=residential");
    String msg = md.toMessage();
    Assert.assertNotNull(msg);

    String[] cols = msg.split("\t");
    // header: Lon Lat Elev Dist CostPerKm ElevCost TurnCost NodeCost InitialCost WayTags NodeTags Time Energy Gradient
    Assert.assertEquals("should have 14 columns", 14, cols.length);
    Assert.assertEquals("last column is gradient", "50", cols[13]); // 5.0% * 10 = 50
  }

  @Test
  public void toMessageGradientPositiveUphill() {
    // 100m rise over 1000m = 10%
    MessageData md = createMessage(1000, 100.0, "highway=track");
    String msg = md.toMessage();
    String[] cols = msg.split("\t");
    // 10.0% * 10 = 100
    Assert.assertEquals("100", cols[13]);
  }

  @Test
  public void toMessageGradientNegativeDownhill() {
    // 100m drop over 1000m = -10%
    MessageData md = createMessage(1000, -100.0, "highway=track");
    String msg = md.toMessage();
    String[] cols = msg.split("\t");
    // -10.0% * 10 = -100
    Assert.assertEquals("-100", cols[13]);
  }

  @Test
  public void toMessageGradientFlat() {
    MessageData md = createMessage(500, 0.0, "highway=primary");
    String msg = md.toMessage();
    String[] cols = msg.split("\t");
    Assert.assertEquals("0", cols[13]);
  }

  @Test
  public void toMessageNullWayKeyValuesReturnsNull() {
    MessageData md = new MessageData();
    md.linkdist = 100;
    md.deltaH = 5.0;
    md.gradient = 5.0f;
    Assert.assertNull(md.toMessage());
  }

  @Test
  public void addMergesGradientWeightedByDistance() {
    // Segment A: 500m, 25m rise => 5%
    MessageData a = createMessage(500, 25.0, "highway=residential");
    // Segment B: 500m, 75m rise => 15%
    MessageData b = createMessage(500, 75.0, "highway=residential");

    // add(d) accumulates d into this: b.add(a) means b absorbs a
    b.add(a);

    // combined: 1000m total, 100m rise => 10%
    Assert.assertEquals(1000, b.linkdist);
    Assert.assertEquals(100.0, b.deltaH, 0.001);
    Assert.assertEquals(10.0f, b.gradient, 0.01f);
  }

  @Test
  public void addMergesUphillAndDownhill() {
    // Segment A: 400m, 20m rise => 5% up
    MessageData a = createMessage(400, 20.0, "highway=path");
    // Segment B: 600m, -10m drop => -1.67% down
    MessageData b = createMessage(600, -10.0, "highway=path");

    b.add(a);

    // combined: 1000m, 10m net rise => 1%
    Assert.assertEquals(1000, b.linkdist);
    Assert.assertEquals(10.0, b.deltaH, 0.001);
    Assert.assertEquals(1.0f, b.gradient, 0.01f);
  }

  @Test
  public void addAccumulatesCosts() {
    MessageData a = createMessage(300, 10.0, "highway=path");
    a.linkelevationcost = 5;
    a.linkturncost = 3;
    a.linknodecost = 2;
    a.linkinitcost = 1;

    MessageData b = createMessage(200, 5.0, "highway=path");
    b.linkelevationcost = 10;
    b.linkturncost = 7;
    b.linknodecost = 4;
    b.linkinitcost = 3;

    b.add(a);

    Assert.assertEquals(500, b.linkdist);
    Assert.assertEquals(15, b.linkelevationcost);
    Assert.assertEquals(10, b.linkturncost);
    Assert.assertEquals(6, b.linknodecost);
    Assert.assertEquals(4, b.linkinitcost);
  }

  @Test
  public void addWithZeroDistanceLeavesGradientUnchanged() {
    MessageData a = new MessageData();
    a.linkdist = 0;
    a.deltaH = 0;
    a.wayKeyValues = "highway=x";

    MessageData b = new MessageData();
    b.linkdist = 0;
    b.deltaH = 0;
    b.wayKeyValues = "highway=x";

    b.add(a);

    Assert.assertEquals(0, b.linkdist);
    Assert.assertEquals(0f, b.gradient, 0.001f);
  }

  @Test
  public void copyPreservesGradientAndDeltaH() {
    MessageData md = createMessage(1000, 50.0, "highway=tertiary");
    MessageData copy = md.copy();

    Assert.assertEquals(md.gradient, copy.gradient, 0.001f);
    Assert.assertEquals(md.deltaH, copy.deltaH, 0.001);
    Assert.assertEquals(md.linkdist, copy.linkdist);
    Assert.assertEquals(md.wayKeyValues, copy.wayKeyValues);
  }

  @Test
  public void gradientRoundingInToMessage() {
    // 3.7% gradient: iGradient = (int)(3.7 * 10 + 0.5) = 37
    MessageData md = createMessage(1000, 37.0, "highway=track");
    String msg = md.toMessage();
    String[] cols = msg.split("\t");
    Assert.assertEquals("37", cols[13]);

    // -3.7% gradient: iGradient = (int)(-3.7 * 10 - 0.5) = -37
    md = createMessage(1000, -37.0, "highway=track");
    msg = md.toMessage();
    cols = msg.split("\t");
    Assert.assertEquals("-37", cols[13]);
  }

  @Test
  public void gradientSmallValue() {
    // 0.1% gradient = 1m over 1000m
    MessageData md = createMessage(1000, 1.0, "highway=motorway");
    String msg = md.toMessage();
    String[] cols = msg.split("\t");
    // 0.1% * 10 = 1
    Assert.assertEquals("1", cols[13]);
  }

  /**
   * Helper to create a MessageData with given distance, elevation change, and way tags.
   */
  private MessageData createMessage(int distMeters, double deltaH, String wayKeyValues) {
    MessageData md = new MessageData();
    md.linkdist = distMeters;
    md.deltaH = deltaH;
    md.gradient = distMeters > 0 ? (float) (deltaH / distMeters * 100.0) : 0f;
    md.wayKeyValues = wayKeyValues;
    md.costfactor = 1.0f;
    md.lon = 180000000 + 8720000; // dummy lon
    md.lat = 90000000 + 50000000; // dummy lat
    md.ele = 400; // 100m
    return md;
  }
}
