package com.team5817.lib;


import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathStateGenerator{
  private final Timer timer = new Timer();
  public static PathStateGenerator instance = null;
  private PathPlannerTrajectory transformedTrajectory;

  double desiredRotation = 0;
  double speed = 1;
  Pose2d currentPose;
  boolean useEvents = false;
  boolean ran = false;

  int EventIndex = 0;

  public PathStateGenerator() {
  }

  public static PathStateGenerator getInstance() {// if doesnt have an instance of swerve will make a new one
      if (instance == null)
          instance = new PathStateGenerator();
      return instance;
  }


  public void startTimer() {
    this.timer.start();
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    resetTimer();
    
    this.transformedTrajectory = trajectory;
  }

  public Pose2d sample(double timestamp){
    return new Pose2d(transformedTrajectory.sample(timestamp).getTargetHolonomicPose());
  }

  public State getDesiredState(boolean useAllianceColor) {
    return getDesiredState(useAllianceColor,timer.get());
  }
  public State getDesiredState(boolean useAllianceColor,double time) {
    double currentTime = this.timer.get();
    State desiredState = transformedTrajectory.sample(currentTime);
    desiredState = transformStateForAlliance(desiredState, DriverStation.getAlliance().get());


    Logger.recordOutput("desiredPose", desiredState.getTargetHolonomicPose());
    return desiredState;

  }

  public double reflect(double x) {
    return 16.5-x;
  }

  public boolean alliance() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  public Pose2d getInitial(PathPlannerTrajectory trajectory,double Rotation, boolean useAllianceColor) {
    double initX = trajectory.getInitialState().positionMeters.getX();
    double initY = trajectory.getInitialState().positionMeters.getY();
    double initRot = Rotation;
    if (alliance() && useAllianceColor) {
      return new Pose2d(new Translation2d(reflect(initX), initY),
          Rotation2d.fromDegrees(initRot).flip());
    } else {
      return new Pose2d(new Translation2d(initX, initY),
         Rotation2d.fromDegrees(initRot));
    }
  }

  public void stopTimer(){
    timer.stop();
  }

  public double getrotation() {
    return desiredRotation;
  }

  public void resetTimer() {
    timer.reset();
    timer.stop();
  }

  public boolean isFinished() {

    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds()+ .2);

  }


  // public double percentageDone() {
  //   return (this.timer.get() / (transformedTrajectory.getTotalTimeSeconds() + .2));
  // }

  public double getTime(){
    return this.timer.get();
  }
  public static State transformStateForAlliance(
    State state, DriverStation.Alliance alliance) {
  if (alliance == DriverStation.Alliance.Red) {
    // Create a new state so that we don't overwrite the original
    State transformedState = new State();

    Translation2d transformedTranslation =
        new Translation2d(16.5-state.positionMeters.getX(), state.positionMeters.getY());
    Rotation2d transformedHeading = new Rotation2d(state.heading.times(-1));
    Rotation2d transformedHolonomicRotation = new Rotation2d(state.targetHolonomicRotation.times(-1));

    transformedState.timeSeconds = state.timeSeconds;
    transformedState.velocityMps = state.velocityMps;
    transformedState.accelerationMpsSq = state.accelerationMpsSq;
    transformedState.positionMeters = new Pose2d(transformedTranslation, transformedHeading).toWPI().getTranslation();
    transformedState.accelerationMpsSq = -state.accelerationMpsSq;
    transformedState.targetHolonomicRotation = transformedHolonomicRotation.toWPI();
    transformedState.holonomicAngularVelocityRps = Optional.of(-state.holonomicAngularVelocityRps.get());
    transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
    transformedState.heading = transformedHeading.toWPI();
    
    return transformedState;
  } else {
    return state;
  }
}
}
