package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.MatchedWaypoint;

/**
 * Result of a greedy round-trip planning attempt.
 * Contains the route, quality metrics, and diagnostic metadata.
 */
public class RoundTripResult {

  private OsmTrack track;
  private List<OsmNodeNamed> loopWaypoints;
  private List<MatchedWaypoint> matchedWaypoints;
  private int totalDistanceMeters;
  private double reusedEdgeRatio;
  private boolean withinTolerance;
  private int attemptsUsed;
  private int subRoutesChosen;
  private final List<String> diagnostics = new ArrayList<>();
  private String fallbackReason;
  private List<OsmTrack> legTracks; // per-leg sub-route tracks from greedy planner
  // Spec §10 telemetry — compute-budget audit signals.
  private int candidatesGenerated;
  private int candidatesRouted;
  private int returnChecksPerformed;
  private long runtimeMillis;
  // Auto-quality-redesign §132 telemetry: routed candidates broken down by
  // candidate source (iso-derived vs radial). The greedy planner identifies
  // source via the existing `costFromStart != NO_ISO_COST` sentinel.
  // "Routed" counts every candidate that the planner ran through Dijkstra;
  // "accepted" counts only those that became part of the final loop.
  // Low-iso-usage classification should use ACCEPTED legs, not routed.
  private int routedIsoCandidates;
  private int routedRadialCandidates;
  private int acceptedIsoLegs;
  private int acceptedRadialLegs;

  public OsmTrack getTrack() {
    return track;
  }

  public void setTrack(OsmTrack track) {
    this.track = track;
  }

  public List<OsmNodeNamed> getLoopWaypoints() {
    return loopWaypoints;
  }

  public void setLoopWaypoints(List<OsmNodeNamed> loopWaypoints) {
    this.loopWaypoints = loopWaypoints;
  }

  public List<MatchedWaypoint> getMatchedWaypoints() {
    return matchedWaypoints;
  }

  public void setMatchedWaypoints(List<MatchedWaypoint> matchedWaypoints) {
    this.matchedWaypoints = matchedWaypoints;
  }

  public int getTotalDistanceMeters() {
    return totalDistanceMeters;
  }

  public void setTotalDistanceMeters(int totalDistanceMeters) {
    this.totalDistanceMeters = totalDistanceMeters;
  }

  public double getReusedEdgeRatio() {
    return reusedEdgeRatio;
  }

  public void setReusedEdgeRatio(double reusedEdgeRatio) {
    this.reusedEdgeRatio = reusedEdgeRatio;
  }

  public boolean isWithinTolerance() {
    return withinTolerance;
  }

  public void setWithinTolerance(boolean withinTolerance) {
    this.withinTolerance = withinTolerance;
  }

  public int getAttemptsUsed() {
    return attemptsUsed;
  }

  public void setAttemptsUsed(int attemptsUsed) {
    this.attemptsUsed = attemptsUsed;
  }

  public int getSubRoutesChosen() {
    return subRoutesChosen;
  }

  public void setSubRoutesChosen(int subRoutesChosen) {
    this.subRoutesChosen = subRoutesChosen;
  }

  public List<String> getDiagnostics() {
    return diagnostics;
  }

  public void addDiagnostic(String message) {
    diagnostics.add(message);
  }

  public String getFallbackReason() {
    return fallbackReason;
  }

  public void setFallbackReason(String fallbackReason) {
    this.fallbackReason = fallbackReason;
  }

  public List<OsmTrack> getLegTracks() {
    return legTracks;
  }

  public void setLegTracks(List<OsmTrack> legTracks) {
    this.legTracks = legTracks;
  }

  /** Number of candidate points produced by the candidate provider across all steps. */
  public int getCandidatesGenerated() {
    return candidatesGenerated;
  }

  public void setCandidatesGenerated(int candidatesGenerated) {
    this.candidatesGenerated = candidatesGenerated;
  }

  /** Number of candidate-leg sub-routes actually computed by Dijkstra. */
  public int getCandidatesRouted() {
    return candidatesRouted;
  }

  public void setCandidatesRouted(int candidatesRouted) {
    this.candidatesRouted = candidatesRouted;
  }

  /** Number of return-to-start feasibility Dijkstras performed. */
  public int getReturnChecksPerformed() {
    return returnChecksPerformed;
  }

  public void setReturnChecksPerformed(int returnChecksPerformed) {
    this.returnChecksPerformed = returnChecksPerformed;
  }

  /** Wall-clock duration of the planning attempt, milliseconds. */
  public long getRuntimeMillis() {
    return runtimeMillis;
  }

  public void setRuntimeMillis(long runtimeMillis) {
    this.runtimeMillis = runtimeMillis;
  }

  /** Number of iso-derived candidates the planner Dijkstra-routed. */
  public int getRoutedIsoCandidates() { return routedIsoCandidates; }
  public void setRoutedIsoCandidates(int v) { this.routedIsoCandidates = v; }

  /** Number of radial (geometric) candidates the planner Dijkstra-routed. */
  public int getRoutedRadialCandidates() { return routedRadialCandidates; }
  public void setRoutedRadialCandidates(int v) { this.routedRadialCandidates = v; }

  /** Number of iso-derived candidates that became legs in the final loop. */
  public int getAcceptedIsoLegs() { return acceptedIsoLegs; }
  public void setAcceptedIsoLegs(int v) { this.acceptedIsoLegs = v; }

  /** Number of radial candidates that became legs in the final loop. */
  public int getAcceptedRadialLegs() { return acceptedRadialLegs; }
  public void setAcceptedRadialLegs(int v) { this.acceptedRadialLegs = v; }
}
