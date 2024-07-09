// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Autos;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathSegment;
import com.team5817.frc.subsystems.RobotState;
import com.team5817.frc.subsystems.SuperStructure;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import com.team5817.lib.ChoreoEventMarker;
import com.team5817.lib.EventMarker;
import com.team5817.lib.PathStateGenerator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;

/** 
 * This is an abstract class
 * abstract classes tell its child methods it needs to have or it will raise an errror
 * we have to implement a method called auto
 * this class will also give its child the stop auto method because all autos will need it
 */
public abstract class AutoBase {
    public PathStateGenerator mPathStateGenerator;
    public RobotState mRobotState;
    public SuperStructure s;
    public List<Translation2d> stopPoses = new ArrayList<>();
    public List<Double> stopTimestamps = new ArrayList<>();
    public AutoBase(){
        mPathStateGenerator = PathStateGenerator.getInstance();
        mRobotState = RobotState.getInstance();
        s = SuperStructure.getInstance();
    }
    public abstract void auto();

    public void runAuto(){
        auto();
    }

    public void stopAuto(){
        SuperStructure.getInstance().clearQueues();
        SwerveDrive.getInstance().setState(SwerveDrive.State.MANUAL);
    }


    public void registerChoreoEvents(String pathName){
        String name = "choreo/" + pathName + ".traj";
        PathPlannerTrajectory path = PathPlannerPath.fromChoreoTrajectory(pathName).getTrajectory(new ChassisSpeeds(), new Rotation2d());
        List<ChoreoEventMarker> eventMarkers = new ArrayList<>();
    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(Filesystem.getDeployDirectory(), name)))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    for (var markerJson : (JSONArray) json.get("eventMarkers")) {
      eventMarkers.add(ChoreoEventMarker.fromJson((JSONObject) markerJson));
    }
    for (ChoreoEventMarker m : eventMarkers) {
        double timestamp = m.getTimestamp();
        m.markerPos = new Translation2d(path.sample(timestamp).getTargetHolonomicPose().getTranslation());
        Logger.recordOutput("Marker Pose " + eventMarkers.indexOf(m), Pose2d.fromTranslation(m.markerPos).toWPI());

        if(m.getName().equals("Shoot")){
            s.waitForMarkerState(m);
            s.printState("Shooting");
            s.shootState(false);
            s.resumeTrajectoryState();
            stopPoses.add(m.markerPos);
            stopTimestamps.add(m.getTimestamp());
    }
        else if(m.getName().equals("Intake Shoot")){
            s.waitForMarkerState(m);
            s.printState("Intaking + Shooting");
            s.intakeShootState(.6);
            s.resumeTrajectoryState();
          }
        else if(m.getName().equals("Stop")){
            stopPoses.add(m.markerPos);
            stopTimestamps.add(m.getTimestamp());
        }
        else if(m.getName().equals("Intake")){
            s.waitForMarkerState(m);
            s.setContinuousShootState(false);
            s.printState("Intaking");
            s.intakeState(1.5);
            s.setContinuousShootState(true);
        }
        else{
            System.out.println("Invalid event name: "+ m.getName());
        }
    }
    } catch (Exception e) {
      e.printStackTrace();
    }
        
    
    }
    public void registerPathEvents(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        List<EventMarker> eventMarkers = new ArrayList<>();
        var allPoints = path.getAllPathPoints();
        try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    for (var markerJson : (JSONArray) json.get("eventMarkers")) {
      eventMarkers.add(EventMarker.fromJson((JSONObject) markerJson));
    }
    for (EventMarker m : eventMarkers) {
        int pointIndex = (int) Math.round(m.getWaypointRelativePos() / PathSegment.RESOLUTION);
        m.markerPos = new Translation2d(allPoints.get(pointIndex).position);
        Logger.recordOutput("Marker Pose " + eventMarkers.indexOf(m), Pose2d.fromTranslation(m.markerPos).toWPI());

        if(m.getName().equals("Shoot")){
            s.waitForPositionState(m.markerPos);
            s.printState("Shooting");
            s.shootState(false);
            s.resumeTrajectoryState();
            stopPoses.add(m.markerPos);
            stopTimestamps.add(0.0);
        }
        else if(m.getName().equals("Intake Shoot")){
            s.waitForPositionState(m.markerPos);
            s.printState("Intaking + Shooting");
            s.intakeShootState(.6);
            s.resumeTrajectoryState();
          }
        else if(m.getName().equals("Stop")){
            stopPoses.add(m.markerPos);
            stopTimestamps.add(0.0);
        }
        else if(m.getName().equals("Intake")){
            s.waitForPositionState(m.markerPos);
            s.setContinuousShootState(false);
            s.printState("Intaking");
            s.intakeState(1.5);
            s.setContinuousShootState(true);
        }
        else{
            System.out.println("Invalid event name: "+ m.getName());
        }
    }
    } catch (Exception e) {
      e.printStackTrace();
    }
        
    
    }
    public void updateAuto(){
        if(!stopPoses.isEmpty()){
                Translation2d markerPose = stopPoses.get(0);
                Translation2d currentPose = new Translation2d(PathStateGenerator.getInstance().getDesiredState(true).positionMeters);
                double pathTimesamp = PathStateGenerator.getInstance().getTime();
                if(stopTimestamps.get(0) < pathTimesamp+.2){//hubert I don't think you did this right
                    if(markerPose.translateBy(currentPose.inverse()).norm() < 0.1){
                        mPathStateGenerator.stopTimer();
                        stopPoses.remove(0);
                    }
               
            
    }}

}
}
