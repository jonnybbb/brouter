package btools.router;

import java.util.Locale;

public enum DirectionPreference {
  N(0), NE(45), E(90), SE(135), S(180), SW(225), W(270), NW(315), ANY(-1);

  public final double bearing;

  DirectionPreference(double bearing) {
    this.bearing = bearing;
  }

  public static DirectionPreference fromString(String s) {
    if (s == null) return ANY;
    try {
      return valueOf(s.toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException e) {
      return ANY;
    }
  }
}
