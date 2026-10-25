package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import btools.expressions.BExpressionContextWay;
import btools.expressions.BExpressionMetaData;
import com.google.protobuf.ByteString;
import java.io.DataOutputStream;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.openstreetmap.osmosis.osmbinary.Fileformat;
import org.openstreetmap.osmosis.osmbinary.Osmformat;

/** Preserve explicit absence before unknown tag values lose their raw meaning. */
public class OsmCutterRailwayTagTest {
  @Rule
  public TemporaryFolder temporary = new TemporaryFolder();

  @Test
  public void ferryWithExplicitNoRailwayRemainsWaterOnly() throws Exception {
    Fixture fixture = new Fixture(Map.of("route", "ferry", "railway", "no"), false);
    assertFalse(fixture.description.contains("railway="));
    assertTrue(fixture.description.contains("route=ferry"));
    assertEquals(0, fixture.audit());
    assertTrue(Files.readString(fixture.root.resolve("gate/gate.properties")).contains("waterOnly=2"));
  }

  @Test
  public void knownAndUnknownPositiveRailwaysStillRequireElevation() throws Exception {
    for (String railway : new String[]{"rail", "abandoned", "unrecognized_railway_fixture"}) {
      Fixture fixture = new Fixture(Map.of("route", "ferry", "railway", railway), false);
      assertTrue(fixture.description.contains("railway="));
      assertEquals(railway, 1, fixture.audit());
    }
  }

  @Test
  public void highwayTaggedFerryAndSharedRoadNodesStillRequireElevation() throws Exception {
    Fixture mixed = new Fixture(Map.of("route", "ferry", "railway", "no", "highway", "path"), false);
    assertTrue(mixed.description.contains("highway=path"));
    assertEquals(1, mixed.audit());
    Fixture shared = new Fixture(Map.of("route", "ferry", "railway", "no"), true);
    assertEquals(1, shared.audit());
    assertTrue(Files.readString(shared.root.resolve("gate/failures.csv")).contains(",2,200,"));
  }

  @Test
  public void negativeAccessRestrictionsArePreserved() throws Exception {
    Fixture fixture = new Fixture(Map.of("route", "ferry", "railway", "no", "access", "no"), false);
    assertFalse(fixture.description.contains("railway="));
    assertTrue(fixture.description.contains("access=no"));
  }

  private final class Fixture implements WayListener {
    final Path root = temporary.newFolder().toPath();
    final Path profiles = Path.of("../misc/profiles2");
    final Path lookups = profiles.resolve("lookups.dat");
    final BExpressionContextWay context;
    String description;

    Fixture(Map<String, String> tags, boolean sharedRoad) throws Exception {
      for (String directory : List.of("nodes", "ways", "nodes55", "ways55", "elevation")) {
        Files.createDirectory(root.resolve(directory));
      }
      writePbf(root.resolve("source.osm.pbf"), tags, sharedRoad);
      String previous = System.getProperty("avoidMapPolling");
      System.setProperty("avoidMapPolling", "true");
      try {
        OsmFastCutter.doCut(lookups.toFile(), root.resolve("nodes").toFile(),
          root.resolve("ways").toFile(), root.resolve("nodes55").toFile(),
          root.resolve("ways55").toFile(), root.resolve("border.dat").toFile(),
          root.resolve("relations.dat").toFile(), root.resolve("restrictions.dat").toFile(),
          profiles.resolve("all.brf").toFile(), profiles.resolve("trekking.brf").toFile(),
          profiles.resolve("softaccess.brf").toFile(), root.resolve("source.osm.pbf").toFile(), null);
      } finally {
        if (previous == null) System.clearProperty("avoidMapPolling");
        else System.setProperty("avoidMapPolling", previous);
      }
      BExpressionMetaData metadata = new BExpressionMetaData();
      context = new BExpressionContextWay(metadata);
      metadata.readMetaData(lookups.toFile());
      new WayIterator(this, false).processFile(root.resolve("ways55/E0_N45.wt5").toFile());
    }

    int audit() throws Exception {
      Path quarantine = root.resolve("quarantine.csv");
      Files.writeString(quarantine, "id,west,south,east,north\n");
      return RoutingElevationGate.audit(root.resolve("nodes55"), root.resolve("ways55"),
        root.resolve("elevation"), lookups, quarantine, root.resolve("gate"), 1000);
    }

    public boolean wayFileStart(File file) { return true; }
    public void wayFileEnd(File file) { }
    public void nextWay(WayData way) {
      if (way.wid == 100) description = context.getKeyValueDescription(false, way.description);
    }
  }

  private static void writePbf(Path path, Map<String, String> tags, boolean sharedRoad) throws Exception {
    Map<String, Integer> indexes = new LinkedHashMap<>();
    indexes.put("", 0);
    List<Map<String, String>> ways = new ArrayList<>();
    ways.add(tags);
    if (sharedRoad) ways.add(Map.of("highway", "path"));
    for (Map<String, String> way : ways) {
      for (Map.Entry<String, String> entry : way.entrySet()) {
        indexes.computeIfAbsent(entry.getKey(), key -> indexes.size());
        indexes.computeIfAbsent(entry.getValue(), key -> indexes.size());
      }
    }
    Osmformat.StringTable.Builder table = Osmformat.StringTable.newBuilder();
    for (String value : indexes.keySet()) table.addS(ByteString.copyFromUtf8(value));
    Osmformat.PrimitiveGroup.Builder group = Osmformat.PrimitiveGroup.newBuilder();
    for (int id = 1; id <= (sharedRoad ? 3 : 2); id++) {
      group.addNodes(Osmformat.Node.newBuilder().setId(id)
        .setLon(10000000L + id * 10000).setLat(460000000L));
    }
    for (int i = 0; i < ways.size(); i++) {
      Osmformat.Way.Builder way = Osmformat.Way.newBuilder().setId(100L + i * 100L);
      for (Map.Entry<String, String> entry : ways.get(i).entrySet()) {
        way.addKeys(indexes.get(entry.getKey())).addVals(indexes.get(entry.getValue()));
      }
      way.addRefs(1L + i).addRefs(1L);
      group.addWays(way);
    }
    try (DataOutputStream out = new DataOutputStream(Files.newOutputStream(path))) {
      writeBlob(out, "OSMHeader", Osmformat.HeaderBlock.newBuilder()
        .addRequiredFeatures("OsmSchema-V0.6").build().toByteArray());
      writeBlob(out, "OSMData", Osmformat.PrimitiveBlock.newBuilder()
        .setStringtable(table).addPrimitivegroup(group).build().toByteArray());
    }
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
