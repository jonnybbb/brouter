package btools.router;

import java.util.HashMap;
import java.util.Map;

import org.junit.rules.TestRule;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

/**
 * Snapshots every {@code loop.*}, {@code golden.*}, {@code perf.*},
 * {@code mtb.*} and {@code roundtrip.*} system property — the toggles
 * {@code brouter-core/build.gradle} forwards to the fork — before a test and
 * restores that exact set afterwards: values put back, additions removed.
 *
 * <p>Gradle passes these properties to the integrationTest fork, and the
 * suites that run later in the same JVM depend on them: {@link LoopTestSegments}
 * reads {@code loop.segments.nodownload} / {@code noupdate} on every fetch to
 * keep the shared segment dir pinned, and the evaluation harness reads the
 * {@code loop.refine.*} toggles. A test that sets one of them for an assertion
 * must leave behind the value the fork started with, not a cleared property:
 * a cleared {@code loop.segments.noupdate} re-enables freshness downloads for
 * every suite that follows and swaps tiles under the golden and evaluation
 * runs. Any test that touches these properties uses this rule instead of a
 * hand-written save/restore.
 *
 * <pre>
 *   &#64;Rule
 *   public final LoopPropertiesRule loopProperties = new LoopPropertiesRule();
 * </pre>
 */
public final class LoopPropertiesRule implements TestRule {

  private static final String[] PREFIXES = {"loop.", "golden.", "perf.", "mtb.", "roundtrip."};

  @Override
  public Statement apply(Statement base, Description description) {
    return new Statement() {
      @Override
      public void evaluate() throws Throwable {
        Map<String, String> before = snapshot();
        try {
          base.evaluate();
        } finally {
          restore(before);
        }
      }
    };
  }

  static Map<String, String> snapshot() {
    Map<String, String> values = new HashMap<>();
    for (String key : System.getProperties().stringPropertyNames()) {
      if (isTracked(key)) {
        values.put(key, System.getProperty(key));
      }
    }
    return values;
  }

  static void restore(Map<String, String> before) {
    for (String key : System.getProperties().stringPropertyNames()) {
      if (isTracked(key) && !before.containsKey(key)) {
        System.clearProperty(key);
      }
    }
    for (Map.Entry<String, String> e : before.entrySet()) {
      System.setProperty(e.getKey(), e.getValue());
    }
  }

  private static boolean isTracked(String key) {
    for (String prefix : PREFIXES) {
      if (key.startsWith(prefix)) {
        return true;
      }
    }
    return false;
  }
}
