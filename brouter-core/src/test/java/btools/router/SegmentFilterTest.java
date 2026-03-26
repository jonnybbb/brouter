package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

public class SegmentFilterTest {

  private OsmTrack track;

  /**
   * Builds a track with 5 segments of varying length and gradient:
   *
   * Seg 0: 200m,  +10m rise  =>  5.0% gradient, wayTags="highway=residential"
   * Seg 1: 500m,  +50m rise  => 10.0% gradient, wayTags="highway=track"
   * Seg 2: 1500m,  0m flat   =>  0.0% gradient, wayTags="highway=primary"
   * Seg 3: 800m, -40m drop   => -5.0% gradient, wayTags="highway=secondary"
   * Seg 4: 2500m, +50m rise  =>  2.0% gradient, wayTags="highway=tertiary"
   */
  @Before
  public void setUp() {
    track = new OsmTrack();

    int[][] segments = {
      // {dist, deltaH_x100, lon_offset}
      {200, 1000, 0},        // 10m rise (deltaH stored as *100 for precision)
      {500, 5000, 1000},     // 50m rise
      {1500, 0, 2000},       // flat
      {800, -4000, 3000},    // 40m drop
      {2500, 5000, 4000},    // 50m rise
    };
    String[] wayTags = {
      "highway=residential",
      "highway=track",
      "highway=primary",
      "highway=secondary",
      "highway=tertiary",
    };
    float[] gradients = {5.0f, 10.0f, 0.0f, -5.0f, 2.0f};
    double[] deltaHs = {10.0, 50.0, 0.0, -40.0, 50.0};

    int baseLon = 180000000 + 8720000;
    int baseLat = 90000000 + 50000000;
    short baseElev = 400; // 100m

    for (int i = 0; i < segments.length; i++) {
      MessageData md = new MessageData();
      md.linkdist = segments[i][0];
      md.deltaH = deltaHs[i];
      md.gradient = gradients[i];
      md.wayKeyValues = wayTags[i];
      md.costfactor = 1.0f;
      md.lon = baseLon + segments[i][2];
      md.lat = baseLat;
      md.ele = baseElev;

      OsmPathElement pe = OsmPathElement.create(
        baseLon + segments[i][2], baseLat, baseElev, null);
      pe.message = md;
      track.nodes.add(pe);
    }
  }

  @Test
  public void aggregateMessageDataReturnsAllSegments() {
    List<MessageData> data = track.aggregateMessageData();
    Assert.assertEquals("should have 5 distinct segments", 5, data.size());
  }

  @Test
  public void aggregateMessageDataMergesConsecutiveSameWayTags() {
    // Add two more nodes with the same wayTags as existing segment to trigger merging
    OsmTrack mergeTrack = new OsmTrack();

    MessageData md1 = createMessage(300, 15.0, "highway=track");
    OsmPathElement pe1 = OsmPathElement.create(180008720, 140000000, (short) 400, null);
    pe1.message = md1;
    mergeTrack.nodes.add(pe1);

    MessageData md2 = createMessage(700, 35.0, "highway=track");
    OsmPathElement pe2 = OsmPathElement.create(180008721, 140000000, (short) 400, null);
    pe2.message = md2;
    mergeTrack.nodes.add(pe2);

    List<MessageData> data = mergeTrack.aggregateMessageData();
    Assert.assertEquals("consecutive same-tag segments should merge", 1, data.size());
    Assert.assertEquals(1000, data.get(0).linkdist);
    Assert.assertEquals(50.0, data.get(0).deltaH, 0.01);
    Assert.assertEquals(5.0f, data.get(0).gradient, 0.01f); // 50m / 1000m * 100
  }

  @Test
  public void filterByMinLength() {
    List<String> filtered = track.filterSegments(1000, -1, -1, -1);
    Assert.assertEquals("segments >= 1000m: primary(1500), tertiary(2500)", 2, filtered.size());
  }

  @Test
  public void filterByMaxLength() {
    List<String> filtered = track.filterSegments(-1, 500, -1, -1);
    Assert.assertEquals("segments <= 500m: residential(200), track(500)", 2, filtered.size());
  }

  @Test
  public void filterByMinAndMaxLength() {
    List<String> filtered = track.filterSegments(500, 1500, -1, -1);
    Assert.assertEquals("segments 500-1500m: track(500), primary(1500)", 2, filtered.size());
  }

  @Test
  public void filterByMinGradient() {
    // absolute gradient >= 5%: residential(5%), track(10%), secondary(-5%)
    List<String> filtered = track.filterSegments(-1, -1, 5.0f, -1);
    Assert.assertEquals("segments with |gradient| >= 5%", 3, filtered.size());
  }

  @Test
  public void filterByMaxGradient() {
    // absolute gradient <= 2%: primary(0%), tertiary(2%)
    List<String> filtered = track.filterSegments(-1, -1, -1, 2.0f);
    Assert.assertEquals("segments with |gradient| <= 2%", 2, filtered.size());
  }

  @Test
  public void filterByGradientRange() {
    // absolute gradient between 2% and 6%: residential(5%), secondary(5%), tertiary(2%)
    List<String> filtered = track.filterSegments(-1, -1, 2.0f, 6.0f);
    Assert.assertEquals("segments with 2% <= |gradient| <= 6%", 3, filtered.size());
  }

  @Test
  public void filterCombinedLengthAndGradient() {
    // >= 500m AND |gradient| >= 5%: track(500m, 10%), secondary(800m, 5%)
    List<String> filtered = track.filterSegments(500, -1, 5.0f, -1);
    Assert.assertEquals("segments >= 500m and |gradient| >= 5%", 2, filtered.size());
  }

  @Test
  public void filterNoMatchReturnsEmpty() {
    // >= 3000m: no segments that long
    List<String> filtered = track.filterSegments(3000, -1, -1, -1);
    Assert.assertTrue("no segments >= 3000m", filtered.isEmpty());
  }

  @Test
  public void filterAllMatchReturnsAll() {
    // very permissive: all segments qualify
    List<String> filtered = track.filterSegments(0, 10000, 0.0f, 100.0f);
    Assert.assertEquals("all segments match", 5, filtered.size());
  }

  @Test
  public void filterNoConstraintsReturnsAll() {
    // all -1 means no filter
    List<String> filtered = track.filterSegments(-1, -1, -1, -1);
    Assert.assertEquals("no filter returns all segments", 5, filtered.size());
  }

  @Test
  public void getFilteredMessagesWithoutFilterReturnsAggregated() {
    // No filter set, so getFilteredMessages() should return same as aggregateMessages()
    List<String> unfiltered = track.aggregateMessages();
    List<String> filtered = track.getFilteredMessages();
    Assert.assertEquals(unfiltered.size(), filtered.size());
    for (int i = 0; i < unfiltered.size(); i++) {
      Assert.assertEquals(unfiltered.get(i), filtered.get(i));
    }
  }

  @Test
  public void getFilteredMessagesWithFilterApplied() {
    track.segmentFilterMinLength = 1000;
    List<String> filtered = track.getFilteredMessages();
    Assert.assertEquals("only segments >= 1000m", 2, filtered.size());
  }

  @Test
  public void getFilteredMessagesWithGradientFilter() {
    track.segmentFilterMinGradient = 5.0f;
    List<String> filtered = track.getFilteredMessages();
    Assert.assertEquals("segments with |gradient| >= 5%", 3, filtered.size());
  }

  @Test
  public void hasSegmentFilterReturnsFalseByDefault() {
    Assert.assertFalse(track.hasSegmentFilter());
  }

  @Test
  public void hasSegmentFilterReturnsTrueWhenSet() {
    track.segmentFilterMinLength = 500;
    Assert.assertTrue(track.hasSegmentFilter());

    track = new OsmTrack();
    track.segmentFilterMaxGradient = 10.0f;
    Assert.assertTrue(track.hasSegmentFilter());
  }

  @Test
  public void filteredMessageFormatContainsGradient() {
    List<String> messages = track.aggregateMessages();
    for (String msg : messages) {
      String[] cols = msg.split("\t");
      Assert.assertEquals("each message should have 14 columns", 14, cols.length);
      // gradient column is the last one
      int gradientTenths = Integer.parseInt(cols[13]);
      Assert.assertTrue("gradient should be a valid number", gradientTenths >= -1000 && gradientTenths <= 1000);
    }
  }

  @Test
  public void filterSteepOnlyLongSegments() {
    // Real-world query: "find segments >= 2km with >= 2% gradient"
    track.segmentFilterMinLength = 2000;
    track.segmentFilterMinGradient = 2.0f;
    List<String> filtered = track.getFilteredMessages();
    // Only tertiary(2500m, 2%) matches
    Assert.assertEquals(1, filtered.size());
    Assert.assertTrue("should contain tertiary", filtered.get(0).contains("highway=tertiary"));
  }

  private MessageData createMessage(int distMeters, double deltaH, String wayKeyValues) {
    MessageData md = new MessageData();
    md.linkdist = distMeters;
    md.deltaH = deltaH;
    md.gradient = distMeters > 0 ? (float) (deltaH / distMeters * 100.0) : 0f;
    md.wayKeyValues = wayKeyValues;
    md.costfactor = 1.0f;
    md.lon = 180000000 + 8720000;
    md.lat = 90000000 + 50000000;
    md.ele = 400;
    return md;
  }
}
