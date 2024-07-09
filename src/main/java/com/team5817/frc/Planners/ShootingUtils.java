// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Planners;


import com.team5817.frc.Constants;
import com.team5817.frc.subsystems.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;


/** Add your docs here. */
public class ShootingUtils {
    

    public enum NoteState {
        NEW,
        MEDIUM,
        OLD
    }

    public static class ShootingParameters{
        public double effectiveDistance;
        public double compensatedDistance;

        public double uncompensatedDesiredPivotAngle;
        public double compensatedDesiredPivotAngle;

        public double uncompensatedDesiredShooterSpeed;
        public double compensatedDesiredShooterSpeed;



        public ShootingParameters(
           double effectiveDistance,
           double compensatedDistance,
           double compensatedDesiredPivotAngle,
           double uncompensatedDesiredPivotAngle,

           double compensatedDesiredShooterSpeed,
           double uncompensatedDesiredShooterSpeed

        ){
         this.effectiveDistance = effectiveDistance;
         this.compensatedDistance = compensatedDistance;
         this.compensatedDesiredPivotAngle = compensatedDesiredPivotAngle;
         this.uncompensatedDesiredPivotAngle = uncompensatedDesiredPivotAngle;
         this.compensatedDesiredShooterSpeed = compensatedDesiredShooterSpeed;
         this.uncompensatedDesiredShooterSpeed = uncompensatedDesiredShooterSpeed;


        }
    }

    public static ShootingParameters getShootingParameters(
        Pose2d currentPose, 
        Pose2d targetPose,
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity){
         Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(lookahead_time+.004).rotateBy(currentPose.getRotation());
        Pose2d compensatedShooterToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(poseAtTimeFrame.getTranslation().inverse()));


        double compensatedDistance = compensatedShooterToTarget.getTranslation().norm();

        
        double desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;


        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;


        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed
);
    }
    public static ShootingParameters getShootingParameters(
        double pivotOffset,
        Pose2d currentPose, 
        Pose2d targetPose,
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity,
        boolean manual){
         Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPose(lookahead_time*1.3);
        Translation2d futureOdomToTargetPoint = poseAtTimeFrame.getTranslation().translateBy(targetPose.getTranslation().inverse());


        double compensatedDistance = futureOdomToTargetPoint.norm();

        
        double compensatedPivotAngle;
        if(manual)
            compensatedPivotAngle = 55;
        else{
            compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value+pivotOffset+2.52;
        }
        double desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
       
        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed

            );
    }



    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getPivotMap(boolean lob){
        System.out.println(lob);
        if
        (lob)
            return Constants.PivotConstants.LobAngleMap;
        return Constants.PivotConstants.SpeakerAngleMap;
    }
 public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getVelocityMap(boolean lob){
        if(lob)
            return Constants.ShooterConstants.LOB_VELOCITY_TREE_MAP;
        return Constants.ShooterConstants.SPEAKER_VELOCITY_TREE_MAP;
    }
}