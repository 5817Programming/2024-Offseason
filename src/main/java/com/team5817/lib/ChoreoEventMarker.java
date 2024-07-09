package com.team5817.lib;

import com.team254.lib.geometry.Translation2d;

import org.json.simple.JSONObject;

/** Position along the path that will trigger a command when reached */
public class ChoreoEventMarker {
  private final double timestamp;
  private final double minimumTriggerDistance;
  private final String name;

  public Translation2d markerPos;

  /**
   * Create a new event marker
   *
   * @param timestamp The waypoint relative position of the marker
   * @param command The command that should be triggered at this marker
   * @param minimumTriggerDistance The minimum distance the robot must be within for this marker to
   *     be triggered
   */
  public ChoreoEventMarker(double timestamp, double minimumTriggerDistance, String name) {
    this.timestamp = timestamp;
    this.minimumTriggerDistance = minimumTriggerDistance;
    this.name = name;
  }

  /**
   * Create a new event marker
   *
   * @param timestamp The waypoint relative position of the marker
   * @param command The command that should be triggered at this marker
   */
  public ChoreoEventMarker(double timestamp, String name) {
    this(timestamp, 0.5, name);
  }

  /**
   * Create an event marker from json
   *
   * @param markerJson {@link org.json.simple.JSONObject} representing an event marker
   * @return The event marker defined by the given json object
   */
  public static ChoreoEventMarker fromJson(JSONObject markerJson) {
    double timestamp = ((Number) markerJson.get("timestamp")).doubleValue();
    JSONObject cmd = ((JSONObject) markerJson.get("command"));
    JSONObject data = ((JSONObject) cmd.get("name"));
    String name = ((String) data.get("name")).toString();
    return new ChoreoEventMarker(timestamp, name);
  }


  public String getName(){
    return name;
  }

  
  public double getTimestamp() {
    return timestamp;
  }

  /**
   * Get the minimum trigger distance for this marker
   *
   * @return The minimum trigger distance in meters
   */
  public double getMinimumTriggerDistance() {
    return minimumTriggerDistance;
  }

  

}
