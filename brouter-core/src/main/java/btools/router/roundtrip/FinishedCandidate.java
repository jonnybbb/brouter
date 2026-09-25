package btools.router.roundtrip;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;
import btools.router.OsmTrack;

/**
 * Result of full candidate finalization (§4.7).
 */
public final class FinishedCandidate {
  private final FinalizationOutcome outcome;
  private final String reason;
  private final OsmTrack track;
  private final List<MatchedWaypoint> matchedWaypoints;
  private final RoundTripQualityResult qualityVerdict;
  private final double oracleCostPerMeter;
  private final double rcs;

  public FinishedCandidate(FinalizationOutcome outcome, String reason, OsmTrack track,
                           List<MatchedWaypoint> matchedWaypoints, RoundTripQualityResult qualityVerdict,
                           double oracleCostPerMeter, double rcs) {
    this.outcome = outcome;
    this.reason = reason;
    this.track = track;
    if (matchedWaypoints != null) {
      List<MatchedWaypoint> copy = new ArrayList<>(matchedWaypoints.size());
      for (MatchedWaypoint wp : matchedWaypoints) {
        copy.add(RefineSkeleton.copyWaypoint(wp));
      }
      this.matchedWaypoints = Collections.unmodifiableList(copy);
    } else {
      this.matchedWaypoints = null;
    }
    this.qualityVerdict = qualityVerdict;
    this.oracleCostPerMeter = oracleCostPerMeter;
    this.rcs = rcs;
  }

  public FinalizationOutcome getOutcome() {
    return outcome;
  }

  public boolean isSuccess() {
    return outcome == FinalizationOutcome.SUCCESS;
  }

  public String getReason() {
    return reason;
  }

  public OsmTrack getTrack() {
    return track;
  }

  public List<MatchedWaypoint> getMatchedWaypoints() {
    return matchedWaypoints;
  }

  public RoundTripQualityResult getQualityVerdict() {
    return qualityVerdict;
  }

  public double getOracleCostPerMeter() {
    return oracleCostPerMeter;
  }

  public double getRcs() {
    return rcs;
  }

  /** Create a FinishedCandidate representing the original tier baseline loop (§4.5). */
  public static FinishedCandidate fromBaseline(OsmTrack track, List<MatchedWaypoint> matchedWaypoints,
                                                RoundTripQualityResult qualityVerdict,
                                                double oracleCostPerMeter, double rcs) {
    return new FinishedCandidate(FinalizationOutcome.SUCCESS, "baseline", track,
      matchedWaypoints, qualityVerdict, oracleCostPerMeter, rcs);
  }
}
