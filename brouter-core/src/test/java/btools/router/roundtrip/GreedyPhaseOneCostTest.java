package btools.router.roundtrip;

import btools.router.OsmTrack;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

/** Unit tests for the phase-1 compiled-leg cost median. */
public class GreedyPhaseOneCostTest {

  @Test
  public void medianOfCompiledLegCostIgnoresCandidatesWithoutALeg() {
    List<RoundTripCandidateProvider.CandidatePoint> cps = new ArrayList<>();
    cps.add(leg(1000, 1000));   // 1.0
    cps.add(leg(1000, 3000));   // 3.0
    cps.add(leg(1000, 2000));   // 2.0
    cps.add(new RoundTripCandidateProvider.CandidatePoint()); // iso pick, no leg
    assertEquals(2.0, GreedyRoundTripPlanner.medianCompiledLegCostPerMeter(cps), 1e-9);
    cps.add(leg(1000, 4000));   // 4.0 → even count → mean of 2.0 and 3.0
    assertEquals(2.5, GreedyRoundTripPlanner.medianCompiledLegCostPerMeter(cps), 1e-9);
    assertEquals(-1, GreedyRoundTripPlanner.medianCompiledLegCostPerMeter(new ArrayList<>()), 1e-9);
  }

  private static RoundTripCandidateProvider.CandidatePoint leg(int distance, int cost) {
    RoundTripCandidateProvider.CandidatePoint cp = new RoundTripCandidateProvider.CandidatePoint();
    cp.routedTrack = new OsmTrack();
    cp.routedTrack.distance = distance;
    cp.routedTrack.cost = cost;
    return cp;
  }
}
