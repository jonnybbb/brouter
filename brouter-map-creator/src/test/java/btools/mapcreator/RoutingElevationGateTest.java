package btools.mapcreator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import btools.expressions.BExpressionContextWay;
import btools.expressions.BExpressionMetaData;
import btools.util.DiffCoderDataOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RoutingElevationGateTest {
  @Rule
  public TemporaryFolder tmp = new TemporaryFolder();

  private final class Fixture {
    final Path root = tmp.newFolder().toPath();
    final Path nodes = Files.createDirectory(root.resolve("nodes"));
    final Path ways = Files.createDirectory(root.resolve("ways"));
    final Path elevation = Files.createDirectory(root.resolve("elevation"));
    final Path report = root.resolve("report");
    final Path quarantine = root.resolve("quarantine.csv");
    final Path lookups = Path.of("../misc/profiles2/lookups.dat");
    final Path nodeFile = nodes.resolve("E0_N45.n5d");
    final Path wayFile = ways.resolve("E0_N45.wt5");
    final BExpressionContextWay context;

    Fixture() throws Exception {
      Files.writeString(quarantine, "id,west,south,east,north\n");
      BExpressionMetaData metadata = new BExpressionMetaData();
      context = new BExpressionContextWay(metadata);
      metadata.readMetaData(lookups.toFile());
      nodes(1);
      ways("highway=residential");
    }

    void nodes(int count) throws Exception {
      try (DiffCoderDataOutputStream out = new DiffCoderDataOutputStream(Files.newOutputStream(nodeFile))) {
        for (int i = 0; i < count; i++) {
          new NodeData(123 + i, 1 + i * 0.01, 46).writeTo(out);
        }
      }
    }

    void ways(String... descriptions) throws Exception {
      try (DataOutputStream out = new DataOutputStream(Files.newOutputStream(wayFile))) {
        for (int i = 0; i < descriptions.length; i++) {
          WayData way = new WayData(456 + i);
          way.nodes.add(123);
          int[] tags = context.createNewLookupData();
          for (String tag : descriptions[i].split(" ")) {
            String[] parts = tag.split("=");
            context.addLookupValue(parts[0], parts[1], tags);
          }
          way.description = context.encode(tags);
          way.writeTo(out);
        }
      }
    }

    void terrain(int height) throws IOException {
      PmTilesTestArchive builder = new PmTilesTestArchive().zoomRange(6, 6);
      for (int x = 30; x <= 34; x++) {
        for (int y = 20; y <= 24; y++) {
          builder.put(6, x, y, TerrainTiles.constantPng(64, height));
        }
      }
      try (PmTilesArchive archive = PmTilesArchive.open(builder.asByteSource())) {
        ConvertMapterhornTile.convertOne(archive, 6, null, elevation.toFile(),
          0, 45, 4, null, 1, false, 64);
      }
    }

    int audit() throws Exception {
      return RoutingElevationGate.audit(nodes, ways, elevation, lookups, quarantine, report, 1000);
    }

    String failures() throws IOException {
      return Files.readString(report.resolve("failures.csv"));
    }
  }

  @Test
  public void missingLandCellFailsAndNamesTheCellForConversion() throws Exception {
    Fixture fixture = new Fixture();
    assertEquals(1, fixture.audit());
    assertTrue(fixture.failures().contains("MISSING_CELL"));
    assertTrue(Files.readString(fixture.report.resolve("missing-cells.txt")).contains("srtm_37_03"));
  }

  @Test
  public void onlyExplicitWaterWaysCanExemptMissingElevation() throws Exception {
    for (String description : new String[]{"route=ferry", "waterway=river", "waterway=canal"}) {
      Fixture fixture = new Fixture();
      fixture.ways(description);
      assertEquals(description, 0, fixture.audit());
      assertTrue(Files.readString(fixture.report.resolve("water-only.csv")).contains("MISSING_CELL"));
      assertEquals("", Files.readString(fixture.report.resolve("missing-cells.txt")));
    }
  }

  @Test
  public void sharedFerryRoadNodeStillRequiresElevationInEitherWayOrder() throws Exception {
    for (boolean ferryFirst : new boolean[]{true, false}) {
      Fixture fixture = new Fixture();
      fixture.ways(ferryFirst ? "route=ferry" : "highway=residential",
        ferryFirst ? "highway=residential" : "route=ferry");
      assertEquals(1, fixture.audit());
      assertTrue(fixture.failures().contains("MISSING_CELL"));
    }
  }

  @Test
  public void unknownRoutesRailwaysAndMixedTagsRequireElevation() throws Exception {
    for (String description : new String[]{"route=hiking", "railway=rail",
        "waterway=dam", "route=ferry highway=residential", "waterway=river route=bicycle"}) {
      Fixture fixture = new Fixture();
      fixture.ways(description);
      assertEquals(description, 1, fixture.audit());
    }
  }

  @Test
  public void unmatchedProblemNodeFailsClosed() throws Exception {
    Fixture fixture = new Fixture();
    fixture.ways();
    assertEquals(1, fixture.audit());
    assertTrue(fixture.failures().contains("NO_INCIDENT_WAY"));
  }

  @Test
  public void validBelowSeaLevelLandPasses() throws Exception {
    Fixture fixture = new Fixture();
    fixture.terrain(-430);
    assertEquals(0, fixture.audit());
    assertTrue(Files.readString(fixture.report.resolve("gate.properties")).contains("status=pass"));
  }

  @Test
  public void sourceInvalidityCannotBeExemptedForLand() throws Exception {
    Fixture fixture = new Fixture();
    fixture.terrain(-32768);
    assertEquals(1, fixture.audit());
    assertTrue(fixture.failures().contains("SUSPECT_SOURCE_HEIGHT:invalid-source-support"));
  }

  @Test
  public void quarantineBlocksFiniteHeightAndInterpolationHalo() throws Exception {
    Fixture fixture = new Fixture();
    fixture.terrain(100);
    // The node is outside this region, but its interpolation support touches it.
    Files.writeString(fixture.quarantine, "id,west,south,east,north\nknown-defect,1.1,46,1.2,46.1\n");
    assertEquals(1, fixture.audit());
    assertTrue(fixture.failures().contains("SUSPECT_SOURCE_HEIGHT:known-defect"));
  }

  @Test
  public void malformedWayDescriptionFailsEvenWhenElevationIsValid() throws Exception {
    Fixture fixture = new Fixture();
    fixture.terrain(100);
    WayData way = new WayData(456);
    way.nodes.add(123);
    way.description = new byte[0];
    try (DataOutputStream output = new DataOutputStream(Files.newOutputStream(fixture.wayFile))) {
      way.writeTo(output);
    }
    try {
      fixture.audit();
      fail("malformed description must not pass merely because terrain is valid");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("WAY_FORMAT"));
      assertFalse(Files.exists(fixture.report.resolve("gate.properties")));
    }
  }

  @Test
  public void truncatedNodeAndWayRecordsFailWithoutSuccessfulReceipt() throws Exception {
    for (boolean node : new boolean[]{true, false}) {
      Fixture fixture = new Fixture();
      Path file = node ? fixture.nodeFile : fixture.wayFile;
      byte[] bytes = Files.readAllBytes(file);
      Files.write(file, Arrays.copyOf(bytes, bytes.length - 1));
      try {
        fixture.audit();
        fail("truncated cutter record must fail");
      } catch (IOException expected) {
        assertFalse(Files.exists(fixture.report.resolve("gate.properties")));
      }
    }
  }

  @Test
  public void problemCapFailsClosedWithoutSuccessfulReceipt() throws Exception {
    Fixture fixture = new Fixture();
    fixture.nodes(2);
    try {
      RoutingElevationGate.audit(fixture.nodes, fixture.ways, fixture.elevation,
        fixture.lookups, fixture.quarantine, fixture.report, 1);
      fail("problem cap must fail");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("RESOURCE_LIMIT"));
      assertFalse(Files.exists(fixture.report.resolve("gate.properties")));
    }
  }

  @Test
  public void earlierSuccessfulReportCannotBeReused() throws Exception {
    Fixture fixture = new Fixture();
    fixture.ways("route=ferry");
    assertEquals(0, fixture.audit());
    fixture.ways("highway=residential");
    try {
      fixture.audit();
      fail("must require a fresh report directory");
    } catch (java.nio.file.FileAlreadyExistsException expected) {
      // The build receives failure; it cannot confuse this attempt with the old pass.
    }
  }
}
