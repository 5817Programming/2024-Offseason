// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Planners;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc.Constants;
import com.team5817.frc.subsystems.RobotState;
import com.team5817.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.team5817.lib.swerve.HeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

/** Add your docs here. */
public class AimingPlanner {

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker;
    private AimingRequest mAimingRequest;
    private boolean isAimed = false;

    public enum AimingRequest {
        SPEAKER,
        LOB,
    }

    public AimingPlanner() {
    }

    public AimingRequest getAimingRequest() {
        return mAimingRequest;
    }

    public Pose2d updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d visionPoseComponent,
            AimingRequest request, Optional<VisionUpdate> visionUpdate, HeadingController headingController,
            Twist2d currentVelocity) {
        Pose2d targetPose = new Pose2d();
        mAimingRequest = request;
        switch (mAimingRequest) {
            case SPEAKER:
                mFieldToSpeaker = Constants.getSpeakerAimingPose();
                break;
            case LOB:
                mFieldToSpeaker = Constants.getLobPose();
                break;
        }
        Logger.recordOutput("Shooting/Mode", mAimingRequest);
        double estimatedTimeFrame = 0;
        Pose2d odomToTargetPoint = visionPoseComponent.inverse().transformBy(mFieldToSpeaker);
        double travelDistance = odomToTargetPoint.transformBy(currentOdomToRobot).getTranslation().norm();
        estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame);

        Logger.recordOutput("Shooting/Future Pose", poseAtTimeFrame.toWPI());
        Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(odomToTargetPoint).inverse();
        Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle().inverse();
        targetPose = new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation);
        headingController.setTargetHeading(targetPose.getRotation().inverse());
        double rotationOutput = headingController.updateRotationCorrection(currentOdomToRobot.getRotation().inverse().rotateBy(-4),
                timeStamp);
        isAimed = headingController.atTarget();
        targetPose = new Pose2d(
                targetPose.getTranslation(),
                Rotation2d.fromDegrees(rotationOutput * 1.75));

        return targetPose;
    }

    public boolean isAimed() {
        return isAimed;
    }

}
