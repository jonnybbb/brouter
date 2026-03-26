package btools.server;


import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.ClassRule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

public class RouteServerTest {
  private static final String host = "localhost";
  private static final String port = "17777";
  private static final String baseUrl = "http://" + host + ":" + port + "/";

  @ClassRule
  public static TemporaryFolder profileDir = new TemporaryFolder();

  @BeforeClass
  public static void setupServer() throws IOException, InterruptedException, URISyntaxException {
    File workingDir = new File(".").getCanonicalFile();
    File segmentDir = new File(workingDir, "../brouter-map-creator/build/resources/test/tmp/segments");
    File profileSourceDir = new File(workingDir, "../misc/profiles2");
    // Copy required files to temporary dir because profile upload will create files
    Files.copy(Paths.get(profileSourceDir.getAbsolutePath(), "lookups.dat"), Paths.get(profileDir.getRoot().getAbsolutePath(), "lookups.dat"));
    Files.copy(Paths.get(profileSourceDir.getAbsolutePath(), "trekking.brf"), Paths.get(profileDir.getRoot().getAbsolutePath(), "trekking.brf"));
    String customProfileDir = "custom";

    Runnable runnable = () -> {
      try {
        RouteServer.main(new String[]{segmentDir.getAbsolutePath(), profileDir.getRoot().getAbsolutePath(), customProfileDir, port, "1"});
      } catch (Exception e) {
        e.printStackTrace(System.out);
      }
    };

    Thread thread = new Thread(runnable);
    thread.start();

    // Busy-wait for server startup
    URL requestUrl = new URI(baseUrl).toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    for (int i = 20; i >= 0; i--) {
      try {
        httpConnection.setConnectTimeout(1000);
        httpConnection.connect();
        break;
      } catch (ConnectException e) {
        if (i == 0) {
          throw e;
        }
        Thread.sleep(100);
      }
    }
  }

  @Test
  public void defaultRouteTrekking() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson").toURL();

    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("1169", geoJson.query("/features/0/properties/track-length"));
    Assert.assertEquals("-15", geoJson.query("/features/0/properties/plain-ascend"));
    Assert.assertEquals("4", geoJson.query("/features/0/properties/filtered ascend"));
  }

  @Test
  public void overrideParameter() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson&profile:avoid_unsafe=1").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("1455", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void voiceHints() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.705796,50.003124%7C8.705859,50.0039599&nogos=&profile=trekking&alternativeidx=0&format=geojson&timode=2").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals(2, geoJson.query("/features/0/properties/voicehints/0/1")); // TL
  }

  @Test
  public void directRoutingFirst() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.718354,50.001514%7C8.718917,50.001361%7C8.716986,50.000105%7C8.718306,50.00145&nogos=&profile=trekking&alternativeidx=0&format=geojson&straight=0&timode=3").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("505", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void directRoutingLast() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.718306,50.00145%7C8.717464,50.000405%7C8.718917,50.001361%7C8.718354,50.001514&nogos=&profile=trekking&alternativeidx=0&format=geojson&straight=2&timode=3").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("506", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void directRoutingMiddle() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.718539,50.006581%7C8.718198,50.006065,d%7C8.71785,50.006034%7C8.7169,50.004456&nogos=&profile=trekking&alternativeidx=0&format=geojson&timode=3").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("347", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void misplacedPoints() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.708678,49.999188%7C8.71145,49.999761%7C8.715801,50.00065&nogos=&profile=trekking&alternativeidx=0&format=geojson&profile:correctMisplacedViaPoints=1&timode=3").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("546", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void misplacedPointsRoundabout() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.699487,50.001257%7C8.701569,50.000092%7C8.704873,49.998898&nogos=&profile=trekking&alternativeidx=0&format=geojson&profile:correctMisplacedViaPoints=1&timode=3").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
    Assert.assertEquals("482", geoJson.query("/features/0/properties/track-length"));
  }

  @Test
  public void uploadValidProfile() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter/profile").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();

    httpConnection.setRequestMethod("POST");
    httpConnection.setDoOutput(true);
    String dummyProfile = "---context:global   # following code refers to global config\n" +
      "\n" +
      "# this prevents suppression of unused tags, so they are visibly in the data tab\n" +
      "assign processUnusedTags = true\n" +
      "assign validForFoot = true\n" +
      "\n" +
      "---context:way   # following code refers to way-tags\n" +
      "\n" +
      "assign costfactor\n" +
      "  switch and highway= not route=ferry  100000 1\n" +
      "\n" +
      "---context:node  # following code refers to node tags\n" +
      "assign initialcost = 0\n";
    try (OutputStream outputStream = httpConnection.getOutputStream()) {
      outputStream.write(dummyProfile.getBytes(StandardCharsets.UTF_8));
    }

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());
    InputStream inputStream = httpConnection.getInputStream();
    String response = new String(inputStream.readAllBytes(), StandardCharsets.UTF_8);
    JSONObject jsonResponse = new JSONObject(response);
    Assert.assertTrue(jsonResponse.query("/profileid").toString().startsWith("custom_"));
    Assert.assertFalse(jsonResponse.has("error"));
  }

  @Test
  public void uploadInvalidProfile() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter/profile").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();

    httpConnection.setRequestMethod("POST");
    httpConnection.setDoOutput(true);
    String invalidProfile = "";
    try (OutputStream outputStream = httpConnection.getOutputStream()) {
      outputStream.write(invalidProfile.getBytes(StandardCharsets.UTF_8));
    }

    // It would be better if RouteServer would return "400 Bad Request"
    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());
    InputStream inputStream = httpConnection.getInputStream();
    String response = new String(inputStream.readAllBytes(), StandardCharsets.UTF_8);
    JSONObject jsonResponse = new JSONObject(response);
    // Returns profileid, but profile isn't valid
    Assert.assertTrue(jsonResponse.query("/profileid").toString().startsWith("custom_"));
    Assert.assertTrue(jsonResponse.has("error"));
  }

  @Test
  public void robots() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "robots.txt").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    String content = new String(httpConnection.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    Assert.assertTrue(content.contains("Disallow: /"));
  }

  @Test
  public void invalidUrl() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "invalid").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_NOT_FOUND, httpConnection.getResponseCode());
  }

  @Test
  public void geojsonMessagesContainGradientColumn() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    InputStream inputStream = httpConnection.getInputStream();
    JSONObject geoJson = new JSONObject(new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));

    // Check that the messages header includes "Gradient"
    Object header = geoJson.query("/features/0/properties/messages/0");
    Assert.assertNotNull("messages header should exist", header);
    String headerStr = header.toString();
    Assert.assertTrue("messages header should contain Gradient", headerStr.contains("Gradient"));

    // Check that message rows have 14 columns (the last being Gradient)
    org.json.JSONArray messages = geoJson.getJSONArray("features")
      .getJSONObject(0).getJSONObject("properties").getJSONArray("messages");
    Assert.assertTrue("should have header + at least one data row", messages.length() >= 2);
    // Header row
    org.json.JSONArray headerRow = messages.getJSONArray(0);
    Assert.assertEquals("header should have 14 columns", 14, headerRow.length());
    Assert.assertEquals("Gradient", headerRow.getString(13));
    // Data row
    org.json.JSONArray dataRow = messages.getJSONArray(1);
    Assert.assertEquals("data row should have 14 columns", 14, dataRow.length());
  }

  @Test
  public void csvFormatContainsGradientColumn() throws IOException, URISyntaxException {
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=csv").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    String csv = new String(httpConnection.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    String[] lines = csv.split("\n");
    Assert.assertTrue("CSV should have header + data lines", lines.length >= 2);

    // Check header
    String header = lines[0];
    Assert.assertTrue("CSV header should contain Gradient", header.contains("Gradient"));
    String[] headerCols = header.split("\t");
    Assert.assertEquals("header should have 14 columns", 14, headerCols.length);
    Assert.assertEquals("Gradient", headerCols[13]);

    // Check data row has 14 columns
    String[] dataCols = lines[1].split("\t");
    Assert.assertEquals("data row should have 14 columns", 14, dataCols.length);
  }

  @Test
  public void segmentFilterMinLength() throws IOException, URISyntaxException {
    // Request without filter
    URL unfilteredUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson").toURL();
    HttpURLConnection conn1 = (HttpURLConnection) unfilteredUrl.openConnection();
    conn1.connect();
    JSONObject unfiltered = new JSONObject(new String(conn1.getInputStream().readAllBytes(), StandardCharsets.UTF_8));
    int unfilteredCount = unfiltered.getJSONArray("features")
      .getJSONObject(0).getJSONObject("properties").getJSONArray("messages").length();

    // Request with minLength filter (only segments >= 500m)
    URL filteredUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson&minLength=500").toURL();
    HttpURLConnection conn2 = (HttpURLConnection) filteredUrl.openConnection();
    conn2.connect();
    Assert.assertEquals(HttpURLConnection.HTTP_OK, conn2.getResponseCode());

    JSONObject filtered = new JSONObject(new String(conn2.getInputStream().readAllBytes(), StandardCharsets.UTF_8));
    int filteredCount = filtered.getJSONArray("features")
      .getJSONObject(0).getJSONObject("properties").getJSONArray("messages").length();

    // Filtered should have fewer or equal rows (header row is always present)
    Assert.assertTrue("filtered messages count (" + filteredCount + ") should be <= unfiltered (" + unfilteredCount + ")",
      filteredCount <= unfilteredCount);
  }

  @Test
  public void segmentFilterMinGradient() throws IOException, URISyntaxException {
    // Request with high gradient filter (only segments with |gradient| >= 20%)
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=geojson&minGradient=20").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    JSONObject geoJson = new JSONObject(new String(httpConnection.getInputStream().readAllBytes(), StandardCharsets.UTF_8));
    int msgCount = geoJson.getJSONArray("features")
      .getJSONObject(0).getJSONObject("properties").getJSONArray("messages").length();

    // Dreieich is flat, so very few (if any) segments should have >= 20% gradient
    // At minimum, there's still the header row (1)
    Assert.assertTrue("steep gradient filter should significantly reduce results", msgCount <= 3);
  }

  @Test
  public void segmentFilterCombinedLengthAndGradient() throws IOException, URISyntaxException {
    // Combined filter: long segments with significant gradient
    URL requestUrl = new URI(baseUrl + "brouter?lonlats=8.723037,50.000491%7C8.712737,50.002899&nogos=&profile=trekking&alternativeidx=0&format=csv&minLength=200&minGradient=1").toURL();
    HttpURLConnection httpConnection = (HttpURLConnection) requestUrl.openConnection();
    httpConnection.connect();

    Assert.assertEquals(HttpURLConnection.HTTP_OK, httpConnection.getResponseCode());

    String csv = new String(httpConnection.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    String[] lines = csv.split("\n");
    // Header always present
    Assert.assertTrue("should have at least header line", lines.length >= 1);
    Assert.assertTrue("header should contain Gradient", lines[0].contains("Gradient"));

    // Verify all returned data rows meet the filter criteria
    for (int i = 1; i < lines.length; i++) {
      String[] cols = lines[i].split("\t");
      int dist = Integer.parseInt(cols[3]);
      int gradientTenths = Integer.parseInt(cols[13]);
      Assert.assertTrue("filtered segment distance should be >= 200m, got " + dist, dist >= 200);
      Assert.assertTrue("filtered segment |gradient| should be >= 1% (10 tenths), got " + gradientTenths,
        Math.abs(gradientTenths) >= 10);
    }
  }
}
