package btools.router;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * Real Dreieich round-trip tests for the <em>output</em> pipeline — the
 * formatters ({@link FormatGpx}/{@link FormatJson}/{@link FormatKml}) and the
 * engine's file-write path ({@code writeAdoptedTrackOutput}).
 *
 * <p>They run against the bundled fixture segment used by {@link RoundTripFixture}
 * (the build-generated Dreieich {@code E5_N50.rd5}), so they execute in CI
 * instead of being skipped — the "not ignored, Dreieich mini round-trip" test
 * afischerdev asked for on PR&nbsp;#903.
 *
 * <p><b>Regression guarded.</b> The AUTO candidate competition (and the forced
 * GREEDY/ISO_GREEDY paths) adopt a track assembled from merged greedy legs via
 * {@code new OsmTrack()}. Plain point-to-point routes get their
 * {@code messageList} populated in {@code doRouting}; the merged round-trip
 * track did not, so it reached {@link FormatGpx} with {@code messageList == null}
 * and any GPX export threw
 * <pre>NullPointerException: Cannot invoke "java.util.List.size()" because "t.messageList" is null</pre>
 * JSON/KML never read {@code messageList}, which is why the failure was
 * GPX-only and the pre-existing round-trip tests (which only inspect the track
 * object, never format it) missed it entirely.
 *
 * <p>Pre-fix, {@link #autoRoundTripGpxExportDoesNotThrow} and
 * {@link #mergedRoundTripsExportGpxWithoutNpe} fail on this exact fixture;
 * post-fix they pass. The root-cause fix populates {@code messageList} in
 * {@code finalizeAdoptedRoundTripTrack} ({@code ensureInfoMessage}); the
 * formatters were additionally hardened to tolerate a null/empty list.
 */
public class RoundTripOutputFormatTest {

  @Rule
  public TemporaryFolder outputDir = new TemporaryFolder();

  /**
   * The greedy-merge competition reliably kicks in at this radius for the
   * Dreieich fixture: a 2&nbsp;km loop to the east is large enough that AUTO
   * adopts a merged greedy track (the path that produced the null
   * {@code messageList}) rather than a {@code messageList}-carrying p2p
   * candidate. Verified empirically against the bundled segment.
   */
  private static final int MERGE_RADIUS = 2000;
  private static final int EAST = 90;

  // -------------------------------------------------------------------------
  // The regression guards: an AUTO / greedy round trip must export GPX.
  // -------------------------------------------------------------------------

  @Test
  public void autoRoundTripGpxExportDoesNotThrow() {
    Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", 2);
    assertProduced(r, "AUTO");
    assertMessageList(r.track, "AUTO");

    // Format through the same context the engine routed with — exactly the
    // call that threw before the fix.
    String gpx = new FormatGpx(r.rc).format(r.track);
    assertWellFormedGpx(gpx, "AUTO");
    // The info line the fix attaches must reach the GPX verbatim.
    Assert.assertTrue("AUTO GPX must embed the info message: " + r.track.messageList.get(0),
      gpx.contains(r.track.messageList.get(0)));
    Assert.assertTrue("AUTO round trip (timode=2) must carry turn instructions",
      r.track.voiceHints != null && !r.track.voiceHints.list.isEmpty());
  }

  @Test
  public void mergedRoundTripsExportGpxWithoutNpe() {
    // GREEDY and ISO_GREEDY both adopt a merged greedy track — the exact
    // shape whose messageList was null. Exercise both directly.
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      Result r = route(algo, EAST, MERGE_RADIUS, "trekking", 4);
      assertProduced(r, algo.name());
      assertMessageList(r.track, algo.name());
      assertWellFormedGpx(new FormatGpx(r.rc).format(r.track), algo.name());
    }
  }

  // -------------------------------------------------------------------------
  // Full-matrix coverage: every output format and every turn-instruction mode
  // must export a merged round trip without throwing.
  // -------------------------------------------------------------------------

  @Test
  public void gpxExportCoversAllTurnInstructionModes() {
    // timode 9 (BRouter style) reads messageList.get(0) for <brouter:info>;
    // every other mode iterates the whole messageList for the comment header.
    // Both reads NPE'd pre-fix, so cover the representative spread.
    for (int timode : new int[]{0, 2, 3, 4, 5, 6, 7, 9}) {
      Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", timode);
      assertProduced(r, "AUTO timode=" + timode);
      assertMessageList(r.track, "AUTO timode=" + timode);
      String gpx = new FormatGpx(r.rc).format(r.track);
      assertWellFormedGpx(gpx, "AUTO timode=" + timode);
    }
  }

  @Test
  public void roundTripExportsGpxJsonAndKml() {
    Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", 2);
    assertProduced(r, "AUTO");

    String gpx = new FormatGpx(r.rc).format(r.track);
    String json = new FormatJson(r.rc).format(r.track);
    String kml = new FormatKml(r.rc).format(r.track);

    assertWellFormedGpx(gpx, "AUTO");
    Assert.assertTrue("JSON is a FeatureCollection", json != null && json.contains("\"FeatureCollection\""));
    Assert.assertTrue("KML has a <kml> root", kml != null && kml.contains("<kml") && kml.contains("</kml>"));
  }

  // -------------------------------------------------------------------------
  // The engine's own write path (the actual production scenario afischerdev
  // ran): outfileBase set, the parent engine formats + writes the adopted
  // winner to a .gpx file. Must produce a parseable file, not log
  // "AUTO: failed to write adopted track: NullPointerException".
  // -------------------------------------------------------------------------

  @Test
  public void engineWritesParseableGpxFileForAutoRoundTrip() throws Exception {
    String base = new File(outputDir.getRoot(), "auto").getAbsolutePath();
    Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", 2, "gpx", base);
    Assert.assertNull("engine reported an error: " + r.engine.getErrorMessage(),
      r.engine.getErrorMessage());
    Assert.assertNotNull("engine produced a track", r.track);

    File gpxFile = new File(base + "0.gpx");
    Assert.assertTrue("engine wrote the adopted GPX to " + gpxFile,
      gpxFile.exists() && gpxFile.length() > 0);
    String content = new String(Files.readAllBytes(gpxFile.toPath()), StandardCharsets.UTF_8);
    assertWellFormedGpx(content, "engine-write");
    assertParseableXml(content, "engine-write");
  }

  @Test
  public void engineWritesCsvFileForAutoRoundTrip() {
    String base = new File(outputDir.getRoot(), "autocsv").getAbsolutePath();
    Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", 0, "csv", base);
    Assert.assertNull("engine reported an error: " + r.engine.getErrorMessage(),
      r.engine.getErrorMessage());
    Assert.assertNotNull("engine produced a track", r.track);

    File csvFile = new File(base + "0.csv");
    Assert.assertTrue("engine wrote the adopted CSV to " + csvFile,
      csvFile.exists() && csvFile.length() > 0);
  }

  // -------------------------------------------------------------------------
  // exportWaypoints: a round trip requested with exportWaypoints must emit the
  // route waypoints into the GPX without tripping over the adopted track's
  // matchedWaypoints, and stay well-formed.
  // -------------------------------------------------------------------------

  @Test
  public void roundTripWithExportWaypointsStaysWellFormed() {
    Result r = route(RoundTripAlgorithm.AUTO, EAST, MERGE_RADIUS, "trekking", 2, null, null, rc -> {
      rc.exportWaypoints = true;
      rc.exportCorrectedWaypoints = true;
    });
    assertProduced(r, "exportWaypoints");
    String gpx = new FormatGpx(r.rc).format(r.track);
    assertWellFormedGpx(gpx, "exportWaypoints");
    Assert.assertTrue("exportWaypoints must emit <wpt> entries", gpx.contains("<wpt"));
  }

  // -------------------------------------------------------------------------
  // Voice-hint regression, now in CI: greedy-merged round trips drop their
  // leg detour metadata unless the detoured merge carries it forward, which
  // would leave processVoiceHints with nothing to emit. The earlier guard for
  // this (greedyRoundTripEmitsVoiceHints) is segments-gated and skipped in CI;
  // this one runs on the bundled fixture.
  // -------------------------------------------------------------------------

  @Test
  public void mergedRoundTripsEmitVoiceHints() {
    for (RoundTripAlgorithm algo : new RoundTripAlgorithm[]{
        RoundTripAlgorithm.AUTO, RoundTripAlgorithm.GREEDY, RoundTripAlgorithm.ISO_GREEDY}) {
      Result r = route(algo, EAST, MERGE_RADIUS, "trekking", 4);
      assertProduced(r, algo.name());
      Assert.assertNotNull(algo + " track must carry a voice-hint list", r.track.voiceHints);
      Assert.assertFalse(algo + " round trip (timode=4) must emit voice hints",
        r.track.voiceHints.list.isEmpty());
    }
  }

  // -------------------------------------------------------------------------
  // helpers
  // -------------------------------------------------------------------------

  /** Engine + the context it routed with + its found track. */
  private static final class Result {
    final RoutingEngine engine;
    final RoutingContext rc;
    final OsmTrack track;

    Result(RoutingEngine engine, RoutingContext rc, OsmTrack track) {
      this.engine = engine;
      this.rc = rc;
      this.track = track;
    }
  }

  private Result route(RoundTripAlgorithm algo, int direction, int radius, String profile, int timode) {
    return route(algo, direction, radius, profile, timode, null, null, rc -> { });
  }

  private Result route(RoundTripAlgorithm algo, int direction, int radius, String profile, int timode,
                       String outputFormat, String outfileBase) {
    return route(algo, direction, radius, profile, timode, outputFormat, outfileBase, rc -> { });
  }

  private Result route(RoundTripAlgorithm algo, int direction, int radius, String profile, int timode,
                       String outputFormat, String outfileBase, Consumer<RoutingContext> tweak) {
    List<OsmNodeNamed> wps = new ArrayList<>();
    wps.add(RoundTripFixture.node("from", 8.72, 50.0));

    RoutingContext rc = new RoutingContext();
    rc.localFunction = RoundTripFixture.profileFile(profile).getAbsolutePath();
    rc.roundTripDistance = radius;
    rc.roundTripAlgorithm = algo;
    rc.startDirection = direction;
    rc.turnInstructionMode = timode;
    if (outputFormat != null) {
      rc.outputFormat = outputFormat;
    }
    tweak.accept(rc);

    RoutingEngine re = new RoutingEngine(outfileBase, outfileBase, RoundTripFixture.segmentDir(),
      wps, rc, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
    re.quite = true;
    re.doRun(60_000);
    return new Result(re, rc, re.getFoundTrack());
  }

  /**
   * The fixture reliably routes these AUTO/greedy loops to the east at
   * {@link #MERGE_RADIUS}; if a future fixture change stops producing them the
   * test should fail loudly (it is meant to run, not be skipped), so this
   * asserts rather than {@code Assume}s.
   */
  private static void assertProduced(Result r, String label) {
    Assert.assertNull(label + ": engine error — " + r.engine.getErrorMessage(),
      r.engine.getErrorMessage());
    Assert.assertNotNull(label + ": no track produced", r.track);
    Assert.assertTrue(label + ": degenerate track (" + r.track.nodes.size() + " nodes)",
      r.track.nodes.size() > 2);
  }

  private static void assertMessageList(OsmTrack track, String label) {
    Assert.assertNotNull(label + ": messageList must not be null (the reported NPE)",
      track.messageList);
    Assert.assertFalse(label + ": messageList must not be empty", track.messageList.isEmpty());
    Assert.assertNotNull(label + ": messageList[0] must not be null", track.messageList.get(0));
    Assert.assertFalse(label + ": messageList[0] must not be empty", track.messageList.get(0).isEmpty());
  }

  private static void assertWellFormedGpx(String gpx, String label) {
    Assert.assertNotNull(label + ": GPX is null", gpx);
    Assert.assertTrue(label + ": GPX must start with the XML declaration", gpx.startsWith("<?xml"));
    Assert.assertTrue(label + ": GPX must contain a <gpx> root", gpx.contains("<gpx"));
    Assert.assertTrue(label + ": GPX must close the <gpx> root", gpx.contains("</gpx>"));
    Assert.assertTrue(label + ": GPX must contain track points", gpx.contains("<trkpt"));
  }

  /** Parse with a real namespace-aware XML parser to prove well-formedness. */
  private static void assertParseableXml(String xml, String label) throws Exception {
    DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
    dbf.setNamespaceAware(true);
    DocumentBuilder db = dbf.newDocumentBuilder();
    db.parse(new ByteArrayInputStream(xml.getBytes(StandardCharsets.UTF_8)));
  }
}
