/**
 * Container for a voice hint
 * (both input- and result data for voice hint processing)
 *
 * @author ab
 */
package btools.router;

import java.util.ArrayList;
import java.util.List;

public class VoiceHintList {

  static final int TRANS_MODE_NONE = 0;
  static final int TRANS_MODE_FOOT = 1;
  static final int TRANS_MODE_BIKE = 2;
  static final int TRANS_MODE_CAR  = 3;

  private int transportMode = TRANS_MODE_BIKE;
  int turnInstructionMode;
  List<VoiceHint> list = new ArrayList<>();

  /**
   * Refuse to emit a turn-instruction format with no hints. A real multi-node
   * route always yields at least START and END hints; an empty list when a
   * turn-instruction mode was requested means the route never produced turn
   * data (e.g. a round trip whose detour metadata was lost). The format
   * routines call this so the invalid state is intercepted loudly rather than
   * silently written as an empty turn-instruction block.
   */
  public void assertNonEmptyIfRequested(int trackNodeCount) {
    if (turnInstructionMode > 0 && list.isEmpty() && trackNodeCount > 1) {
      throw new IllegalArgumentException(
        "turn-instruction mode " + turnInstructionMode + " requested but the route "
          + "produced no voice hints (" + trackNodeCount + " nodes) — refusing to "
          + "emit an empty turn-instruction list");
    }
  }

  public void setTransportMode(boolean isCar, boolean isBike) {
    transportMode = isCar ? TRANS_MODE_CAR : (isBike ? TRANS_MODE_BIKE : TRANS_MODE_FOOT);
  }

  public void setTransportMode(int mode) {
    transportMode = mode;
  }

  public String getTransportMode() {
    String ret;
    switch (transportMode) {
      case TRANS_MODE_FOOT:
        ret = "foot";
        break;
      case TRANS_MODE_CAR:
        ret = "car";
        break;
      case TRANS_MODE_BIKE:
      default:
        ret = "bike";
        break;
    }
    return ret;
  }

  public int transportMode() {
    return transportMode;
  }

  public int getLocusRouteType() {
    if (transportMode == TRANS_MODE_CAR) {
      return 0;
    }
    if (transportMode == TRANS_MODE_BIKE) {
      return 5;
    }
    return 3; // foot
  }
}
