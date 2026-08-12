package btools.router;

import org.junit.Assert;
import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The profile-parameter checksum must identify WHICH key holds WHICH value.
 *
 * <p>ProfileCache reuses a parsed profile whenever this checksum matches, so a checksum that
 * cannot tell two parameter sets apart serves the second request the first one's route. That was
 * a live defect: the checksum summed {@code key.hashCode() + value.hashCode()} per entry, which is
 * invariant under permuting the values between keys.
 *
 * <p>Observed as one request echoing the previous one whenever their parameter value multisets
 * coincided, with any differing request in between clearing it.
 */
public class ProfileParameterCacheKeyTest {

  private static long checksumOf(Map<String, String> params) {
    RoutingContext rc = new RoutingContext();
    rc.keyValues = params;
    return rc.getKeyValueChecksum();
  }

  private static Map<String, String> params(String... kv) {
    Map<String, String> m = new LinkedHashMap<>();
    for (int i = 0; i < kv.length; i += 2) {
      m.put(kv[i], kv[i + 1]);
    }
    return m;
  }

  /**
   * The exact production shape that motivated the fix. Both of these are routes a real rider can
   * request, and under the old checksum both produced 306203190.
   */
  @Test
  public void swappingValuesBetweenTwoKeysChangesTheChecksum() {
    long a = checksumOf(params("avoid_arterials", "6", "avoid_residential", "12"));
    long b = checksumOf(params("avoid_arterials", "12", "avoid_residential", "6"));

    Assert.assertNotEquals(
      "swapping two parameter values must change the cache key, or the second request is "
        + "served the first request's route", a, b);
  }

  /**
   * Guards the shape of the fix, not just this instance. Any per-entry combination of the form
   * {@code a*key + b*value} still sums to the same total under a permutation of the values, so a
   * three-way rotation has to be checked too — it is the case a naive "multiply the key by 31"
   * fix would still get wrong.
   */
  @Test
  public void rotatingValuesAmongThreeKeysChangesTheChecksum() {
    long abc = checksumOf(params("uphillcost_override", "1", "avoid_arterials", "2", "avoid_service", "3"));
    long bca = checksumOf(params("uphillcost_override", "2", "avoid_arterials", "3", "avoid_service", "1"));
    long cab = checksumOf(params("uphillcost_override", "3", "avoid_arterials", "1", "avoid_service", "2"));

    Assert.assertNotEquals("rotation must be visible to the cache key", abc, bca);
    Assert.assertNotEquals("rotation must be visible to the cache key", bca, cab);
    Assert.assertNotEquals("rotation must be visible to the cache key", abc, cab);
  }

  /** Concatenation must not alias: {@code a=b} plus {@code c=} is not {@code a=bc} plus {@code c=}. */
  @Test
  public void pairBoundariesAreNotAmbiguous() {
    Assert.assertNotEquals(
      checksumOf(params("a", "b", "c", "")),
      checksumOf(params("a", "bc", "c", "")));
  }

  /** Same parameters, different insertion order: the key must be stable, or nothing ever caches. */
  @Test
  public void iterationOrderDoesNotAffectTheChecksum() {
    long forward = checksumOf(params("avoid_arterials", "6", "avoid_residential", "12"));
    long reverse = checksumOf(params("avoid_residential", "12", "avoid_arterials", "6"));

    Assert.assertEquals(
      "reordering the same parameters must NOT change the key — otherwise identical requests "
        + "miss the cache and every one reparses the profile", forward, reverse);
  }

  /** No parameters at all is a legitimate request, and must not blow up or alias onto a real one. */
  @Test
  public void emptyAndNullParametersAreStableAndDistinctFromRealOnes() {
    Assert.assertEquals(0L, checksumOf(params()));

    RoutingContext nullKv = new RoutingContext();
    nullKv.keyValues = null;
    Assert.assertEquals(0L, nullKv.getKeyValueChecksum());

    Assert.assertNotEquals(0L, checksumOf(params("uphillcost_override", "600")));
  }

  /**
   * String.hashCode collides readily on the numeric strings profile parameters are made of —
   * "72762.0753" and "206.069547" share one. A checksum built from those hashes inherits the
   * collision; one built from the text does not.
   */
  @Test
  public void valuesWithCollidingStringHashesRemainDistinct() {
    Assert.assertEquals("precondition: these two strings share a String.hashCode",
      "72762.0753".hashCode(), "206.069547".hashCode());

    Assert.assertNotEquals(
      checksumOf(params("uphillcost_override", "72762.0753")),
      checksumOf(params("uphillcost_override", "206.069547")));
  }
}
