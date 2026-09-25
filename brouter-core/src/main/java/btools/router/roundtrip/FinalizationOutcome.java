package btools.router.roundtrip;

/**
 * Outcome states for candidate finalization (§4.5).
 */
public enum FinalizationOutcome {
  /** Candidate successfully detailed, cleaned, gated, and priced. */
  SUCCESS,

  /** Finalization operation exceeded the request or stage deadline. */
  TIMEOUT,

  /** Finalization operation encountered an irrecoverable error or rejection. */
  FAILURE,

  /** Request was cancelled by watchdog or thread interruption. */
  CANCELLED
}
