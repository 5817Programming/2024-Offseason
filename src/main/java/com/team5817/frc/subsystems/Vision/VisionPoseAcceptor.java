package com.team5817.frc.subsystems.Vision;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;

public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(double timestamp, Pose2d visionFieldToVehicle, Twist2d robotVelocity) {


        boolean rotatingTooFast = Math.abs(robotVelocity.dtheta) >= 1.0;
        if (!rotatingTooFast) {
            return true;
        } else {
            return false;
        }
    }

}