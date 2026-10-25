package btools.mapcreator;

import btools.expressions.BExpressionContextWay;
import btools.expressions.BExpressionMetaData;
import btools.util.DiffCoderDataInputStream;
import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PushbackInputStream;
import java.nio.channels.Channels;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Actual cutter-node elevation gate. Run on a private, hash-verified release snapshot. */
public final class RoutingElevationGate {
  public static final String VERSION = "1";
  private static final Pattern NODE_TILE = Pattern.compile("([EW])([0-9]+)_([NS])([0-9]+)\\.n5d");
  private static final Set<String> WATERWAYS = Set.of("river", "stream", "canal", "drain",
    "ditch", "tidal_channel", "fairway");
  private static final int MAX_WAY_NODES = 1_000_000;

  private RoutingElevationGate() {
  }

  public static void main(String[] args) throws Exception {
    if (args.length != 6) {
      throw new IllegalArgumentException("usage: RoutingElevationGate nodes55 waytiles55 elevation"
        + " lookups.dat quarantine.csv new-report-directory");
    }
    int status = audit(Path.of(args[0]), Path.of(args[1]), Path.of(args[2]), Path.of(args[3]),
      Path.of(args[4]), Path.of(args[5]), Integer.getInteger("elevation.maxProblems", 1_000_000));
    if (status != 0) {
      throw new IOException("ELEVATION_GATE_FAILED: inspect " + args[5]
        + "/failures.csv and convert missing-cells.txt into a new elevation release");
    }
  }

  public static int audit(Path nodes, Path ways, Path elevation, Path lookups,
                          Path quarantine, Path report, int maxProblems) throws Exception {
    if (maxProblems < 1 || maxProblems > 1_000_000) {
      throw new IOException("RESOURCE_LIMIT: maxProblems must be 1..1000000");
    }
    List<Zone> zones = readZones(quarantine);
    BExpressionMetaData metadata = new BExpressionMetaData();
    BExpressionContextWay context = new BExpressionContextWay(metadata);
    metadata.readMetaData(lookups.toFile());
    if (context.getLookupKey("highway") < 0 || context.getLookupKey("route") < 0
      || context.getLookupKey("waterway") < 0 || context.getLookupKey("railway") < 0) {
      throw new IOException("CLASSIFICATION_ERROR: required lookup keys are missing");
    }
    List<Path> files;
    try (Stream<Path> entries = Files.list(nodes)) {
      files = entries.filter(path -> path.getFileName().toString().endsWith(".n5d"))
        .sorted().collect(Collectors.toList());
    }
    if (files.isEmpty()) {
      throw new IOException("EMPTY_INPUT: no cutter node tiles in " + nodes);
    }
    // Never reuse a successful report from an earlier invocation after a new failure.
    Files.createDirectory(report);
    Set<String> required = new TreeSet<>();
    Set<String> missing = new TreeSet<>();
    List<String> cutterDigests = new ArrayList<>();
    long totalNodes = 0;
    long failedNodes = 0;
    long waterNodes = 0;
    try (BufferedWriter failures = Files.newBufferedWriter(report.resolve("failures.csv"));
         BufferedWriter exemptions = Files.newBufferedWriter(report.resolve("water-only.csv"))) {
      String header = "cell,nodeId,wayId,longitude,latitude,cause\n";
      failures.write(header);
      exemptions.write(header);
      for (Path nodeFile : files) {
        Matcher match = NODE_TILE.matcher(nodeFile.getFileName().toString());
        if (!match.matches()) {
          throw new IOException("NODE_FORMAT: invalid node tile " + nodeFile);
        }
        int longitude = Integer.parseInt(match.group(2)) * (match.group(1).equals("W") ? -1 : 1);
        int latitude = Integer.parseInt(match.group(4)) * (match.group(3).equals("S") ? -1 : 1);
        ElevationCells.requireCellCorner(longitude, latitude, nodeFile.toString());
        String cell = ElevationCells.legacyCellName(longitude, latitude).replace(".bef", "");
        Path bef = elevation.resolve(cell + ".bef");
        CellData data = Files.exists(bef, LinkOption.NOFOLLOW_LINKS) ? readCell(bef, longitude, latitude) : null;
        if (data != null) {
          required.add(cell);
        }
        Map<Long, Problem> problems = new HashMap<>();
        long tileNodes = 0;
        MessageDigest nodeDigest = MapterhornFiles.sha256Digest();
        try (PushbackInputStream input = digestInput(nodeFile, nodeDigest);
             DiffCoderDataInputStream decoded = new DiffCoderDataInputStream(input)) {
          while (hasNext(input)) {
            // EOF inside a record is fatal; NodeIterator normally treats it as end-of-file.
            NodeData node = new NodeData(decoded);
            tileNodes++;
            String actualCell = PosUnifier.genFilenameXY((node.ilon + 5000000) / 5000000,
              (654999999 - node.ilat) / 5000000 - 100);
            if (node.nid < 0 || !cell.equals(actualCell)) {
              throw new IOException("NODE_FORMAT: node " + node.nid + " belongs to " + actualCell
                + " but was found in " + nodeFile);
            }
            String cause = cause(node, data, zones);
            if (cause != null) {
              if (problems.size() == maxProblems) {
                throw new IOException("RESOURCE_LIMIT: too many suspect nodes in " + cell
                  + "; repair missing cells or source quality before retrying");
              }
              if (problems.put(node.nid, new Problem(node, cause)) != null) {
                throw new IOException("NODE_FORMAT: duplicate node " + node.nid);
              }
            }
          }
        }
        String basename = nodeFile.getFileName().toString().replace(".n5d", ".wt5");
        cutterDigests.add(MapterhornFiles.hex(nodeDigest.digest()) + "  nodes/" + nodeFile.getFileName());
        MessageDigest wayDigest = MapterhornFiles.sha256Digest();
        classify(ways.resolve(basename), context, problems, wayDigest);
        cutterDigests.add(MapterhornFiles.hex(wayDigest.digest()) + "  ways/" + basename);
        long tileFailed = 0;
        long tileWater = 0;
        for (Problem problem : problems.values()) {
          if (problem.water && !problem.land) {
            writeProblem(exemptions, cell, problem);
            tileWater++;
          } else {
            writeProblem(failures, cell, problem);
            tileFailed++;
            required.add(cell);
            if (data == null) {
              missing.add(cell);
            }
          }
        }
        failures.flush();
        exemptions.flush();
        totalNodes += tileNodes;
        failedNodes += tileFailed;
        waterNodes += tileWater;
        System.out.println("elevation-gate " + cell + " nodes=" + tileNodes
          + " failed=" + tileFailed + " waterOnly=" + tileWater);
      }
    }
    Files.write(report.resolve("cutter-inputs.sha256"), cutterDigests);
    Files.write(report.resolve("required-cells.txt"), required);
    Files.write(report.resolve("missing-cells.txt"), missing);
    if (totalNodes == 0) {
      throw new IOException("EMPTY_INPUT: cutter files contained no nodes");
    }
    Files.writeString(report.resolve("gate.properties"), "schema=1\nvalidator=" + VERSION
      + "\nstatus=" + (failedNodes == 0 ? "pass" : "fail") + "\nnodes=" + totalNodes
      + "\nfailed=" + failedNodes + "\nwaterOnly=" + waterNodes + "\n");
    System.out.println("elevation-gate: nodes=" + totalNodes + " failed=" + failedNodes
      + " waterOnly=" + waterNodes);
    return failedNodes == 0 ? 0 : 1;
  }

  private static CellData readCell(Path bef, int longitude, int latitude) throws IOException {
    ElevationCoverage.Evidence coverage = ElevationCoverage.verify(bef);
    if (!coverage.sampling.equals("full") || (coverage.geometry.ncols - 1) % 5 != 0) {
      throw new IOException("COVERAGE_GEOMETRY: routing requires a complete cell " + bef);
    }
    ElevationRaster expected = new ElevationRaster();
    ElevationCells.configureCellRaster(expected, longitude, latitude, (coverage.geometry.ncols - 1) / 5);
    if (!ElevationCells.headerEquals(expected, coverage.geometry)) {
      throw new IOException("COVERAGE_GEOMETRY: cell filename and source geometry differ " + bef);
    }
    MessageDigest digest = MapterhornFiles.sha256Digest();
    ElevationRaster raster;
    try (DigestInputStream input = new DigestInputStream(input(bef), digest)) {
      raster = new ElevationRasterCoder().decodeRaster(input);
      byte[] buffer = new byte[8192];
      while (input.read(buffer) != -1) {
        // Cover the complete file, including codec padding.
      }
    }
    if (!coverage.befSha256.equals(MapterhornFiles.hex(digest.digest()))) {
      throw new IOException("COVERAGE_HASH: cell changed between validation and use " + bef);
    }
    return new CellData(raster, coverage);
  }

  private static String cause(NodeData node, CellData cell, List<Zone> zones) {
    if (cell == null) {
      return "MISSING_CELL";
    }
    double longitude = node.ilon / 1000000.0 - 180;
    double latitude = node.ilat / 1000000.0 - 90;
    // The interpolation's four samples each aggregate a half-sample footprint.
    double margin = 1.5 * cell.raster.cellsize;
    for (Zone zone : zones) {
      if (longitude + margin >= zone.west && longitude - margin <= zone.east
        && latitude + margin >= zone.south && latitude - margin <= zone.north) {
        return "SUSPECT_SOURCE_HEIGHT:" + zone.id;
      }
    }
    int flags = cell.coverage.supportFlags(node.ilon, node.ilat);
    if ((flags & ElevationCoverage.INVALID_SOURCE) != 0) {
      return "SUSPECT_SOURCE_HEIGHT:invalid-source-support";
    }
    if (cell.raster.getElevation(node.ilon, node.ilat) == Short.MIN_VALUE) {
      return (flags & (ElevationCoverage.ABSENT_SOURCE | ElevationCoverage.OUTSIDE)) != 0
        ? "EXPECTED_SOURCE_GAP" : "UNEXPECTED_NODATA";
    }
    return null;
  }

  private static void classify(Path file, BExpressionContextWay context,
                               Map<Long, Problem> problems, MessageDigest digest) throws Exception {
    try (PushbackInputStream input = digestInput(file, digest);
         DataInputStream decoded = new DataInputStream(input)) {
      while (hasNext(input)) {
        long wayId = MapCreatorBase.readId(decoded);
        int length = decoded.readByte();
        if (wayId < 0 || length <= 0) {
          throw new IOException("WAY_FORMAT: invalid description or way ID in " + file);
        }
        byte[] description = new byte[length];
        decoded.readFully(description);
        try {
          context.decode(description);
        } catch (RuntimeException error) {
          throw new IOException("WAY_FORMAT: cannot decode description of way " + wayId, error);
        }
        Boolean water = null;
        int count = 0;
        for (long id; (id = MapCreatorBase.readId(decoded)) != -1;) {
          if (++count > MAX_WAY_NODES || id < 0) {
            throw new IOException("RESOURCE_LIMIT: invalid or oversized way " + wayId);
          }
          Problem problem = problems.get(id);
          if (problem != null) {
            if (water == null) {
              water = isWaterOnly(context.getKeyValueList(false, description));
            }
            if (water) {
              problem.water = true;
              if (!problem.land) {
                problem.wayId = wayId;
              }
            } else {
              problem.land = true;
              problem.wayId = wayId;
            }
          }
        }
        if (count == 0) {
          throw new IOException("WAY_FORMAT: empty way " + wayId);
        }
      }
    }
  }

  private static boolean isWaterOnly(List<String> tags) {
    String highway = null;
    String railway = null;
    String waterway = null;
    String route = null;
    for (int i = 0; i < tags.size(); i += 2) {
      String key = tags.get(i);
      String value = tags.get(i + 1);
      if (key.equals("highway")) highway = value;
      if (key.equals("railway")) railway = value;
      if (key.equals("waterway")) waterway = value;
      if (key.equals("route")) route = value;
    }
    if (highway != null || railway != null) {
      return false;
    }
    if (route != null) {
      return route.equals("ferry") && (waterway == null || WATERWAYS.contains(waterway));
    }
    return waterway != null && WATERWAYS.contains(waterway);
  }

  private static PushbackInputStream input(Path path) throws IOException {
    if (!Files.isRegularFile(path, LinkOption.NOFOLLOW_LINKS)) {
      throw new IOException("INPUT_MISSING: expected regular file " + path);
    }
    return new PushbackInputStream(new BufferedInputStream(Channels.newInputStream(
      Files.newByteChannel(path, StandardOpenOption.READ, LinkOption.NOFOLLOW_LINKS))), 1);
  }

  private static PushbackInputStream digestInput(Path path, MessageDigest digest) throws IOException {
    return new PushbackInputStream(new DigestInputStream(input(path), digest), 1);
  }

  private static boolean hasNext(PushbackInputStream input) throws IOException {
    int next = input.read();
    if (next == -1) return false;
    input.unread(next);
    return true;
  }

  private static void writeProblem(BufferedWriter out, String cell, Problem problem) throws IOException {
    out.write(String.format(Locale.ROOT, "%s,%d,%d,%.6f,%.6f,%s%n", cell, problem.node.nid,
      problem.wayId, problem.node.ilon / 1000000.0 - 180, problem.node.ilat / 1000000.0 - 90,
      problem.cause + (!problem.land && !problem.water ? ":NO_INCIDENT_WAY" : "")));
  }

  private static List<Zone> readZones(Path file) throws IOException {
    if (!Files.isRegularFile(file, LinkOption.NOFOLLOW_LINKS) || Files.size(file) > 1_000_000) {
      throw new IOException("QUALITY_EVIDENCE: missing or oversized quarantine manifest " + file);
    }
    List<String> lines = Files.readAllLines(file);
    if (lines.isEmpty() || !lines.get(0).equals("id,west,south,east,north")) {
      throw new IOException("QUALITY_EVIDENCE: invalid quarantine manifest header");
    }
    List<Zone> zones = new ArrayList<>();
    for (String line : lines.subList(1, lines.size())) {
      String[] values = line.split(",", -1);
      if (values.length != 5 || !values[0].matches("[A-Za-z0-9_-]+")) {
        throw new IOException("QUALITY_EVIDENCE: invalid quarantine record");
      }
      Zone zone = new Zone(values);
      if (!Double.isFinite(zone.west) || !Double.isFinite(zone.south)
        || !Double.isFinite(zone.east) || !Double.isFinite(zone.north)
        || zone.west < -180 || zone.east > 180 || zone.south < -90 || zone.north > 90
        || zone.west > zone.east || zone.south > zone.north) {
        throw new IOException("QUALITY_EVIDENCE: invalid quarantine bounds " + zone.id);
      }
      zones.add(zone);
    }
    return zones;
  }

  private static final class CellData {
    final ElevationRaster raster;
    final ElevationCoverage.Evidence coverage;
    CellData(ElevationRaster raster, ElevationCoverage.Evidence coverage) {
      this.raster = raster;
      this.coverage = coverage;
    }
  }

  private static final class Problem {
    final NodeData node;
    final String cause;
    long wayId = -1;
    boolean land;
    boolean water;
    Problem(NodeData node, String cause) {
      this.node = node;
      this.cause = cause;
    }
  }

  private static final class Zone {
    final String id;
    final double west;
    final double south;
    final double east;
    final double north;
    Zone(String[] values) {
      id = values[0];
      west = Double.parseDouble(values[1]);
      south = Double.parseDouble(values[2]);
      east = Double.parseDouble(values[3]);
      north = Double.parseDouble(values[4]);
    }
  }
}
