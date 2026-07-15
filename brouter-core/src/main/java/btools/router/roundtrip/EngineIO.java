package btools.router.roundtrip;

import btools.router.OsmTrack;

/**
 * The logging/output slice of the engine seam. One of the four role
 * interfaces composed by {@link RoundTripEngineOps}.
 */
public interface EngineIO {

  /** Engine log line; a no-op in {@code quite} child engines (AUTO candidates). */
  void logInfo(String msg);

  /** Engine exception logging. */
  void logException(Throwable t);

  /** Engine throwable logging (compact form). */
  void logThrowable(Throwable t);

  /** Write the adopted track to the engine's configured output (engine IO). */
  void writeAdoptedTrackOutput(OsmTrack track);

  /** Merge duplicate round-trip voice hints (needs VoiceHint package internals). */
  void consolidateRoundTripVoiceHints(OsmTrack track);
}
