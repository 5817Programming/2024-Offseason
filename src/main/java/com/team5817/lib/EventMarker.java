package com.team5817.lib;

import com.pathplanner.lib.auto.CommandUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import org.json.simple.JSONObject;

/** Position along the path that will trigger a command when reached */
public class EventMarker {
  private final double waypointRelativePos;
  private final double minimumTriggerDistance;
  private final String name;

  public Translation2d markerPos;
  private Translation2d lastRobotPos;

  /**
   * Create a new event marker
   *
   * @param waypointRelativePos The waypoint relative position of the marker
   * @param command The command that should be triggered at this marker
   * @param minimumTriggerDistance The minimum distance the robot must be within for this marker to
   *     be triggered
   */
  public EventMarker(double waypointRelativePos, Command command, double minimumTriggerDistance, String name) {
    this.waypointRelativePos = waypointRelativePos;
    this.minimumTriggerDistance = minimumTriggerDistance;
    this.name = name;
    this.lastRobotPos = null;
  }

  /**
   * Create a new event marker
   *
   * @param waypointRelativePos The waypoint relative position of the marker
   * @param command The command that should be triggered at this marker
   */
  public EventMarker(double waypointRelativePos, Command command, String name) {
    this(waypointRelativePos, command, 0.5, name);
  }

  /**
   * Create an event marker from json
   *
   * @param markerJson {@link org.json.simple.JSONObject} representing an event marker
   * @return The event marker defined by the given json object
   */
  public static EventMarker fromJson(JSONObject markerJson) {
    double pos = ((Number) markerJson.get("waypointRelativePos")).doubleValue();
    String name = ((String) markerJson.get("name")).toString();
    Command cmd = CommandUtil.commandFromJson((JSONObject) markerJson.get("command"), false);
    return new EventMarker(pos, cmd, name);
  }

  /**
   * Reset the current robot position
   *
   * @param robotPose The current pose of the robot
   */
  public void reset(Pose2d robotPose) {
    lastRobotPos = robotPose.getTranslation();
  }

  public String getName(){
    return name;
  }

  /**
   * Get if this event marker should be triggered
   *
   * @param robotPose Current pose of the robot
   * @return True if this marker should be triggered
   */
  public boolean shouldTrigger(Pose2d robotPose) {
    if (lastRobotPos == null || markerPos == null) {
      lastRobotPos = robotPose.getTranslation();
      return false;
    }

    double distanceToMarker = robotPose.getTranslation().distance(markerPos);
    boolean trigger =
        distanceToMarker <= minimumTriggerDistance
            && lastRobotPos.distance(markerPos) < distanceToMarker;
    lastRobotPos = robotPose.getTranslation();
    return trigger;
  }

  /**
   * Get the command associated with this marker
   *
   * @return Command for this marker
   */

  /**
   * Get the waypoint relative position of this marker
   *
   * @return Waypoint relative position of this marker
   */
  public double getWaypointRelativePos() {
    return waypointRelativePos;
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
