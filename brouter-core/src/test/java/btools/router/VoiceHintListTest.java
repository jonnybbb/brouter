package btools.router;

import org.junit.Test;

/**
 * Guard for afischerdev's review note: a turn-instruction format must not be
 * emitted with an empty voice-hint list. {@link VoiceHintList#assertNonEmptyIfRequested}
 * intercepts that invalid state instead of silently writing an empty
 * turn-instruction block.
 */
public class VoiceHintListTest {

  @Test(expected = IllegalArgumentException.class)
  public void rejectsEmptyHintsWhenModeRequested() {
    VoiceHintList vhl = new VoiceHintList();
    vhl.turnInstructionMode = 4;
    vhl.assertNonEmptyIfRequested(100); // turn mode requested, empty list, real track
  }

  @Test
  public void allowsEmptyWhenNoModeRequested() {
    VoiceHintList vhl = new VoiceHintList();
    vhl.turnInstructionMode = 0;
    vhl.assertNonEmptyIfRequested(100); // no turn instructions requested → fine
  }

  @Test
  public void allowsEmptyForTrivialTrack() {
    VoiceHintList vhl = new VoiceHintList();
    vhl.turnInstructionMode = 4;
    vhl.assertNonEmptyIfRequested(1); // single-node/degenerate track → not invalid
  }

  @Test
  public void allowsNonEmptyHints() {
    VoiceHintList vhl = new VoiceHintList();
    vhl.turnInstructionMode = 4;
    vhl.list.add(new VoiceHint());
    vhl.assertNonEmptyIfRequested(100); // has hints → fine
  }
}
