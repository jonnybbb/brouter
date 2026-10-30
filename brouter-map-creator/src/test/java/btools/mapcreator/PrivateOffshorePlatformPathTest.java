package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.protobuf.ByteString;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.openstreetmap.osmosis.osmbinary.Fileformat;
import org.openstreetmap.osmosis.osmbinary.Osmformat;

/** Exercise admission through the real PBF parser, cutter output and elevation gate. */
public class PrivateOffshorePlatformPathTest {
  private enum SourceOrder { NORMAL, DUPLICATE_BEFORE, DUPLICATE_AFTER, DECREASING, LATE_BEFORE, LATE_AFTER }
  private static final Map<String, String> PRIVATE_PLATFORM = Map.of(
    "man_made", "offshore_platform", "access", "private");
  private static final Map<String, String> HELIDECK = Map.of(
    "location", "platform", "seamark:type", "platform", "aeroway", "helipad");

  @Rule
  public TemporaryFolder temporary = new TemporaryFolder();

  @Test
  public void privatePlatformLinkIsAbsentFromRoutingAndDoesNotNeedInventedElevation() throws Exception {
    Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, Map.of("highway", "path"), false);
    assertEquals(Set.of(200L), fixture.ways);
    assertEquals(Set.of(3L, 4L), fixture.nodes);
    assertEquals(0, fixture.audit());
    String result = Files.readString(fixture.root.resolve("gate/gate.properties"));
    assertTrue(result, result.contains("nodes=2\n"));
    assertTrue(result, result.contains("waterOnly=2\n"));
  }

  @Test
  public void explicitPublicOrConditionalAccessKeepsThePlatformLink() throws Exception {
    for (Map<String, String> access : List.of(Map.of("access", "yes"), Map.of("foot", "yes"),
      Map.of("bicycle", "designated"), Map.of("psv", "yes"),
      Map.of("access:conditional", "yes @ (Mo-Fr)"))) {
      Map<String, String> tags = new LinkedHashMap<>(access);
      tags.put("highway", "path");
      Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, tags, false);
      assertEquals(tags.toString(), Set.of(100L, 200L), fixture.ways);
      assertEquals(tags.toString(), 1, fixture.audit());
    }
  }

  @Test
  public void ferryRailwayAndWaterwayTagsPreventPlatformExclusion() throws Exception {
    for (Map<String, String> transport : List.of(Map.of("route", "ferry"),
      Map.of("railway", "rail"), Map.of("waterway", "river"))) {
      Map<String, String> tags = new LinkedHashMap<>(transport);
      tags.put("highway", "path");
      Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, tags, false);
      assertEquals(tags.toString(), Set.of(100L, 200L), fixture.ways);
      assertEquals(tags.toString(), 1, fixture.audit());
    }
  }

  @Test
  public void ordinaryPrivatePathsAndLandConnectionsRemain() throws Exception {
    for (Map<String, String> second : List.of(Map.<String, String>of(), Map.of("location", "platform"))) {
      Fixture fixture = new Fixture(PRIVATE_PLATFORM, second,
        Map.of("highway", "path", "access", "private"), false);
      assertEquals(Set.of(100L, 200L), fixture.ways);
      assertEquals(1, fixture.audit());
    }
    Fixture ordinary = new Fixture(Map.of(), Map.of(), Map.of("highway", "path", "access", "private"), false);
    assertEquals(Set.of(100L, 200L), ordinary.ways);
    assertEquals(1, ordinary.audit());
  }

  @Test
  public void publicPlatformsAndNodeAccessOverridesRemain() throws Exception {
    for (Map<String, String> first : List.of(Map.of("man_made", "offshore_platform"),
      Map.of("man_made", "offshore_platform", "access", "yes"),
      Map.of("man_made", "offshore_platform", "access", "private", "foot", "yes"))) {
      Fixture fixture = new Fixture(first, HELIDECK, Map.of("highway", "path"), false);
      assertEquals(first.toString(), Set.of(100L, 200L), fixture.ways);
      assertEquals(first.toString(), 1, fixture.audit());
    }
  }

  @Test
  public void aLandWaySharingAPlatformNodeStillRequiresElevation() throws Exception {
    Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, Map.of("highway", "path"), true);
    assertEquals(Set.of(200L, 300L), fixture.ways);
    assertEquals(Set.of(2L, 3L, 4L, 5L), fixture.nodes);
    assertEquals(1, fixture.audit());
    assertTrue(Files.readString(fixture.root.resolve("gate/failures.csv")).contains(",2,300,"));
  }

  @Test
  public void privatePlatformCanBeAtEitherEndOfThePath() throws Exception {
    Fixture fixture = new Fixture(HELIDECK, PRIVATE_PLATFORM, Map.of("highway", "footway"), false);
    assertEquals(Set.of(200L), fixture.ways);
    assertEquals(0, fixture.audit());
  }

  @Test
  public void unmarkedIntermediateNodesPreventPlatformExclusion() throws Exception {
    Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, Map.of("highway", "path"), false, true);
    assertEquals(Set.of(100L, 200L), fixture.ways);
    assertEquals(Set.of(1L, 2L, 3L, 4L, 5L), fixture.nodes);
    assertEquals(1, fixture.audit());
  }

  @Test
  public void directionalWayAndNodeAccessOverridesRemain() throws Exception {
    Fixture way = new Fixture(PRIVATE_PLATFORM, HELIDECK,
      Map.of("highway", "path", "bicycle:forward", "yes"), false);
    assertEquals(Set.of(100L, 200L), way.ways);
    assertEquals(1, way.audit());
    Fixture psv = new Fixture(PRIVATE_PLATFORM, HELIDECK,
      Map.of("highway", "path", "psv:forward", "yes"), false);
    assertEquals(Set.of(100L, 200L), psv.ways);
    assertEquals(1, psv.audit());
    Map<String, String> tags = new LinkedHashMap<>(PRIVATE_PLATFORM);
    tags.put("foot:backward", "yes");
    Fixture node = new Fixture(tags, HELIDECK, Map.of("highway", "path"), false);
    assertEquals(Set.of(100L, 200L), node.ways);
    assertEquals(1, node.audit());
  }

  @Test
  public void ambiguousNodeOrderBeforeExclusionPreservesWaysForTheGate() throws Exception {
    for (SourceOrder order : List.of(SourceOrder.DUPLICATE_BEFORE, SourceOrder.DUPLICATE_AFTER,
      SourceOrder.DECREASING, SourceOrder.LATE_BEFORE)) {
      Fixture fixture = new Fixture(PRIVATE_PLATFORM, HELIDECK, Map.of("highway", "path"), false, false, order);
      assertEquals(order.toString(), Set.of(100L, 200L), fixture.ways);
      if (order == SourceOrder.DUPLICATE_BEFORE || order == SourceOrder.DUPLICATE_AFTER) {
        try {
          fixture.audit();
          fail("duplicate source nodes must remain visible to the gate");
        } catch (IOException expected) {
          assertTrue(expected.toString(), expected.getMessage().contains("NODE_FORMAT: duplicate node"));
        }
      } else {
        assertEquals(order.toString(), 1, fixture.audit());
      }
    }
  }

  @Test
  public void lateNodesAfterExclusionFailTheCut() throws Exception {
    try {
      new Fixture(PRIVATE_PLATFORM, HELIDECK, Map.of("highway", "path"), false, false, SourceOrder.LATE_AFTER);
      fail("source ambiguity after removal must not produce successful cutter output");
    } catch (RuntimeException expected) {
      assertTrue(expected.toString(), expected.toString().contains("OFFSHORE_PLATFORM_SOURCE_ORDER"));
    }
  }

  private final class Fixture implements WayListener, NodeListener {
    final Path root = temporary.newFolder().toPath();
    final Path profiles = Path.of("../misc/profiles2");
    final Set<Long> ways = new HashSet<>();
    final Set<Long> nodes = new HashSet<>();

    Fixture(Map<String, String> first, Map<String, String> second,
            Map<String, String> pathTags, boolean sharedRoad) throws Exception {
      this(first, second, pathTags, sharedRoad, false);
    }

    Fixture(Map<String, String> first, Map<String, String> second,
            Map<String, String> pathTags, boolean sharedRoad, boolean intermediateLand) throws Exception {
      this(first, second, pathTags, sharedRoad, intermediateLand, SourceOrder.NORMAL);
    }

    Fixture(Map<String, String> first, Map<String, String> second,
            Map<String, String> pathTags, boolean sharedRoad, boolean intermediateLand, SourceOrder order) throws Exception {
      for (String directory : List.of("nodes", "ways", "nodes55", "ways55", "elevation")) {
        Files.createDirectory(root.resolve(directory));
      }
      writePbf(root.resolve("source.osm.pbf"), first, second, pathTags, sharedRoad, intermediateLand, order);
      String previous = System.getProperty("avoidMapPolling");
      System.setProperty("avoidMapPolling", "true");
      try {
        OsmFastCutter.doCut(profiles.resolve("lookups.dat").toFile(), root.resolve("nodes").toFile(),
          root.resolve("ways").toFile(), root.resolve("nodes55").toFile(),
          root.resolve("ways55").toFile(), root.resolve("border.dat").toFile(),
          root.resolve("relations.dat").toFile(), root.resolve("restrictions.dat").toFile(),
          profiles.resolve("all.brf").toFile(), profiles.resolve("trekking.brf").toFile(),
          profiles.resolve("softaccess.brf").toFile(), root.resolve("source.osm.pbf").toFile(), null, true);
      } finally {
        if (previous == null) System.clearProperty("avoidMapPolling");
        else System.setProperty("avoidMapPolling", previous);
      }
      new WayIterator(this, false).processFile(root.resolve("ways55/E0_N45.wt5").toFile());
      new NodeIterator(this, false).processFile(root.resolve("nodes55/E0_N45.n5d").toFile());
    }

    int audit() throws Exception {
      Path quarantine = root.resolve("quarantine.csv");
      Files.writeString(quarantine, "id,west,south,east,north\n");
      return RoutingElevationGate.audit(root.resolve("nodes55"), root.resolve("ways55"),
        root.resolve("elevation"), profiles.resolve("lookups.dat"), quarantine,
        root.resolve("gate"), root.resolve("source.osm.pbf"), 1000);
    }

    public boolean wayFileStart(File file) { return true; }
    public void wayFileEnd(File file) { }
    public void nextWay(WayData way) { ways.add(way.wid); }
    public void nodeFileStart(File file) { }
    public void nodeFileEnd(File file) { }
    public void nextNode(NodeData node) { nodes.add(node.nid); }
  }

  private static void writePbf(Path path, Map<String, String> first, Map<String, String> second,
                               Map<String, String> pathTags, boolean sharedRoad, boolean intermediateLand,
                               SourceOrder order) throws Exception {
    Map<String, Integer> strings = new LinkedHashMap<>();
    strings.put("", 0);
    Map<String, String> ferry = Map.of("route", "ferry");
    for (Map<String, String> tags : List.of(first, second, pathTags, ferry, Map.of("highway", "path"))) {
      for (Map.Entry<String, String> entry : tags.entrySet()) {
        strings.computeIfAbsent(entry.getKey(), key -> strings.size());
        strings.computeIfAbsent(entry.getValue(), key -> strings.size());
      }
    }
    Osmformat.StringTable.Builder table = Osmformat.StringTable.newBuilder();
    for (String value : strings.keySet()) table.addS(ByteString.copyFromUtf8(value));
    Osmformat.PrimitiveGroup.Builder group = Osmformat.PrimitiveGroup.newBuilder();
    if (order == SourceOrder.DUPLICATE_BEFORE) group.addNodes(unmarkedNode(1));
    for (int position = 1; position <= 5; position++) {
      int id = order == SourceOrder.DECREASING ? 6 - position : position;
      Osmformat.Node.Builder node = Osmformat.Node.newBuilder().setId(id)
        .setLon(10000000L + id * 10000).setLat(460000000L);
      Map<String, String> tags = id == 1 ? first : id == 2 ? second : Map.of();
      for (Map.Entry<String, String> entry : tags.entrySet()) {
        node.addKeys(strings.get(entry.getKey())).addVals(strings.get(entry.getValue()));
      }
      group.addNodes(node);
    }
    if (order == SourceOrder.DUPLICATE_AFTER) group.addNodes(unmarkedNode(1));
    addWay(group, strings, 100, pathTags, intermediateLand ? new long[]{1, 5, 2} : new long[]{1, 2});
    addWay(group, strings, 200, ferry, 3, 4);
    if (sharedRoad) addWay(group, strings, 300, Map.of("highway", "path"), 2, 5);
    try (DataOutputStream out = new DataOutputStream(Files.newOutputStream(path))) {
      writeBlob(out, "OSMHeader", Osmformat.HeaderBlock.newBuilder()
        .addRequiredFeatures("OsmSchema-V0.6").build().toByteArray());
      Osmformat.PrimitiveBlock.Builder block = Osmformat.PrimitiveBlock.newBuilder().setStringtable(table);
      if (order == SourceOrder.LATE_BEFORE) {
        Osmformat.PrimitiveGroup.Builder early = Osmformat.PrimitiveGroup.newBuilder();
        addWay(early, strings, 99, Map.of(), 1, 2);
        block.addPrimitivegroup(early);
      }
      block.addPrimitivegroup(group);
      if (order == SourceOrder.LATE_AFTER) {
        block.addPrimitivegroup(Osmformat.PrimitiveGroup.newBuilder().addNodes(unmarkedNode(6)));
      }
      writeBlob(out, "OSMData", block.build().toByteArray());
    }
  }

  private static Osmformat.Node.Builder unmarkedNode(int id) {
    return Osmformat.Node.newBuilder().setId(id).setLon(10000000L + id * 10000).setLat(460000000L);
  }

  private static void addWay(Osmformat.PrimitiveGroup.Builder group, Map<String, Integer> strings,
                             long id, Map<String, String> tags, long... nodes) {
    Osmformat.Way.Builder way = Osmformat.Way.newBuilder().setId(id);
    for (Map.Entry<String, String> entry : tags.entrySet()) {
      way.addKeys(strings.get(entry.getKey())).addVals(strings.get(entry.getValue()));
    }
    long previous = 0;
    for (long node : nodes) {
      way.addRefs(node - previous);
      previous = node;
    }
    group.addWays(way);
  }

  private static void writeBlob(DataOutputStream out, String type, byte[] bytes) throws Exception {
    byte[] payload = Fileformat.Blob.newBuilder().setRaw(ByteString.copyFrom(bytes)).build().toByteArray();
    byte[] header = Fileformat.BlobHeader.newBuilder().setType(type)
      .setDatasize(payload.length).build().toByteArray();
    out.writeInt(header.length);
    out.write(header);
    out.write(payload);
  }
}
