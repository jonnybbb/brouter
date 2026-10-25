package btools.router;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import java.util.Map;

import org.junit.Test;

/** The rule's snapshot/restore must hand later suites exactly the properties the fork started with. */
public class LoopPropertiesRuleTest {

  @Test
  public void restorePutsBackChangedClearedAndAddedProperties() {
    String changed = "loop.rule.test.changed";
    String cleared = "loop.rule.test.cleared";
    String added = "roundtrip.rule.test.added";
    String untracked = "other.rule.test.untracked";
    System.setProperty(changed, "fork");
    System.setProperty(cleared, "fork");
    System.clearProperty(added);
    System.setProperty(untracked, "before");
    try {
      Map<String, String> before = LoopPropertiesRule.snapshot();

      System.setProperty(changed, "test");
      System.clearProperty(cleared);
      System.setProperty(added, "test");
      System.setProperty(untracked, "test");
      LoopPropertiesRule.restore(before);

      assertEquals("changed value restored", "fork", System.getProperty(changed));
      assertEquals("cleared value restored", "fork", System.getProperty(cleared));
      assertNull("added property removed", System.getProperty(added));
      assertEquals("untracked prefixes are left alone", "test", System.getProperty(untracked));
    } finally {
      System.clearProperty(changed);
      System.clearProperty(cleared);
      System.clearProperty(added);
      System.clearProperty(untracked);
    }
  }
}
