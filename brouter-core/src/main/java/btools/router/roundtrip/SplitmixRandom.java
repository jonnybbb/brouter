package btools.router.roundtrip;

/**
 * Deterministic 64-bit SplitMix pseudo-random number generator (§2.8, §4.4).
 * Completely thread-isolated and Android API 23 compliant.
 */
public final class SplitmixRandom {

  private long state;
  private double nextNextGaussian;
  private boolean haveNextNextGaussian = false;

  public SplitmixRandom(long seed) {
    this.state = seed;
  }

  /**
   * Advance state and return next pseudo-random 64-bit long.
   */
  public long nextLong() {
    long z = (state += 0x9e3779b97f4a7c15L);
    z = (z ^ (z >>> 30)) * 0xbf58476d1ce4e5b9L;
    z = (z ^ (z >>> 27)) * 0x94d049bb133111ebL;
    return z ^ (z >>> 31);
  }

  /**
   * Return a pseudo-random integer uniformly distributed in [0, bound).
   */
  public int nextInt(int bound) {
    if (bound <= 0) {
      throw new IllegalArgumentException("bound must be positive");
    }
    int r = (int) (nextLong() >>> 33);
    int m = bound - 1;
    if ((bound & m) == 0) {
      return (int) ((bound * (long) r) >> 31);
    }
    int u = r;
    while (u - (r = u % bound) + m < 0) {
      u = (int) (nextLong() >>> 33);
    }
    return r;
  }

  /**
   * Return a pseudo-random double uniformly distributed in [0.0, 1.0).
   */
  public double nextDouble() {
    return (nextLong() >>> 11) * 0x1.0p-53;
  }

  /**
   * Return next Gaussian distributed value with mean 0.0 and standard deviation 1.0
   * using Box-Muller transform (§4.4).
   */
  public double nextGaussian() {
    if (haveNextNextGaussian) {
      haveNextNextGaussian = false;
      return nextNextGaussian;
    }
    double u1 = nextDouble();
    while (u1 <= 1e-15) {
      u1 = nextDouble();
    }
    double u2 = nextDouble();
    double radius = Math.sqrt(-2.0 * Math.log(u1));
    double theta = 2.0 * Math.PI * u2;
    nextNextGaussian = radius * Math.sin(theta);
    haveNextNextGaussian = true;
    return radius * Math.cos(theta);
  }
}
