package btools.codec;

/**
 * a waypoint matcher gets way geometries
 * from the decoder to find the closest
 * matches to the waypoints
 */
public interface WaypointMatcher {
  /**
   * @param wayDescription the way's tag-value description bitmap (may be null);
   *                       lets matches carry the matched way's tags for
   *                       profile-aware snap scoring downstream
   */
  boolean start(int ilonStart, int ilatStart, int ilonTarget, int ilatTarget, boolean useAsStartWay, byte[] wayDescription);

  void transferNode(int ilon, int ilat);

  void end();

  boolean hasMatch(int lon, int lat);
}
