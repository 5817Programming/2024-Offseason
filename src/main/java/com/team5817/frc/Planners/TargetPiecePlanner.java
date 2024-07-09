// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Planners;

import java.util.Optional;

import com.team5817.frc.subsystems.Vision.ObjectLimeLight.VisionObjectUpdate;
import com.team254.lib.geometry.Rotation2d;


/** Add your docs here. */
public class TargetPiecePlanner {

    public double updateAiming(double timeStamp,Optional<VisionObjectUpdate> visionupdate, HeadingController headingController, Rotation2d currentHeading){
        if(visionupdate.isEmpty()){
            System.out.println("visionempty");
            return 0;
        }
        double objectYDegreesCamera = visionupdate.get().getCameraToTarget().x();
        Rotation2d targetHeading = currentHeading.rotateBy(objectYDegreesCamera);
        headingController.setTargetHeading(targetHeading.inverse());
        double output = headingController.getRotationCorrection(currentHeading.inverse(), timeStamp);   
        return output;
    }


}
