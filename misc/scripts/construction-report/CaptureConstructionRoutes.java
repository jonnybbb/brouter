package btools.router;
import btools.router.roundtrip.LoopCostOracle;
import java.io.*;
import java.nio.file.*;
import java.util.*;
/** Report-only capture of exact regenerated finished routes; no routing mutations. */
public class CaptureConstructionRoutes {
  public static void main(String[] args) throws Exception {
    Path out = Paths.get(args[0]); Files.createDirectories(out);
    List<String> lines = Files.readAllLines(Paths.get("brouter-core/build/construction-diagnosis/final-case-results.csv"));
    for (String line : lines.subList(1, lines.size())) {
      String label = line.split(",")[0];
      FullABEvaluationMatrixTest.CellSpec spec = FullABEvaluationMatrixTest.buildCellMatrix().stream()
        .filter(c -> c.label.equals(label)).findFirst().get();
      long started = System.nanoTime();
      try {
        OsmNodeNamed start = new OsmNodeNamed(); start.name = "start"; start.ilon = spec.region.ilon; start.ilat = spec.region.ilat;
        RoutingContext rc = new RoutingContext();
        rc.localFunction = new File("misc/profiles2/" + spec.profileName).getAbsolutePath();
        rc.roundTripDistance = spec.searchRadius; rc.startDirection = (int) spec.direction;
        rc.roundTripAlgorithm = spec.algorithm; rc.roundTripStrictQuality = false;
        RoutingEngine engine = new RoutingEngine(null, null, new File("segments4"), Arrays.asList(start), rc, RoutingEngine.BROUTER_ENGINEMODE_ROUNDTRIP);
        engine.quite = true; engine.doRun(60000L);
        long requestMs = (System.nanoTime() - started) / 1000000;
        if (engine.getErrorMessage() != null) throw new IllegalStateException(engine.getErrorMessage());
        OsmTrack track = engine.getFoundTrack();
        if (track == null) throw new IllegalStateException("No finished track");
        long signature = LoopCostOracle.geometrySignature(track);
        int cost = LoopCostOracle.priceCost(engine.roundTripOps(), track, track.getMatchedWaypoints());
        String failure = engine.getLastPricingFailure();
        int repeated = LoopCostOracle.priceCost(engine.roundTripOps(), track, track.getMatchedWaypoints());
        if (signature != LoopCostOracle.geometrySignature(track)) throw new IllegalStateException("Pricing mutated geometry");
        Files.writeString(out.resolve(label + ".gpx"), new FormatGpx(rc).format(track));
        StringBuilder json = new StringBuilder("{\"label\":\"").append(label).append("\",\"requestMs\":").append(requestMs)
          .append(",\"cost\":").append(cost).append(",\"repeatCost\":").append(repeated).append(",\"signature\":\"").append(signature)
          .append("\",\"pricingFailure\":\"").append(failure == null ? "" : failure.replace("\"", "'"))
          .append("\",\"points\":[");
        for (int i = 0; i < track.nodes.size(); i++) {
          OsmPathElement p = track.nodes.get(i); if (i > 0) json.append(',');
          json.append('[').append(p.getILon()).append(',').append(p.getILat()).append(',')
            .append(p.getSElev() == Short.MIN_VALUE ? "null" : Double.toString(p.getSElev() / 4.0)).append(']');
        }
        json.append("],\"metrics\":{"); boolean first = true;
        for (Map.Entry<String, Double> e : RouteRideAnalysis.measure(track, 2 * Math.PI * spec.searchRadius).values().entrySet()) {
          if (!first) json.append(','); first = false;
          json.append('"').append(e.getKey()).append("\":").append(Double.isFinite(e.getValue()) ? e.getValue() : "null");
        }
        json.append("}}"); Files.writeString(out.resolve(label + ".json"), json.toString());
        System.out.println("CAPTURE " + label + " points=" + track.nodes.size() + " cost=" + cost + " repeated=" + repeated + " requestMs=" + requestMs);
      } catch (Exception e) {
        Files.writeString(out.resolve(label + ".error.txt"), e.toString());
        System.out.println("FAILED " + label + " " + e);
      }
    }
  }
}
