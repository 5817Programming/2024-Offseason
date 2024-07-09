// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.lib.swerve;

import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.team5817.frc.Constants;
import com.team5817.frc.Constants.SwerveConstants;
import com.team5817.frc.subsystems.Swerve.SwerveDriveModule;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;


/** Add your docs here. */
public class SwerveKinematics {

    public SwerveKinematics() {
        updateModuleRotationVectors();
    }
    
    private final int kNumberOfModules = SwerveConstants.kModulePositions.size();
    private List<Translation2d> moduleRotationVectors;
    private List<Rotation2d> moduleRotations = new ArrayList<>(kNumberOfModules);



    
    public void updateModuleRotationVectors() {
        int numberOfModules = kNumberOfModules;
        double rotateVectorDirection = 360.0 / numberOfModules;
        List<Translation2d> vectorList = new ArrayList<>(numberOfModules);
        for(int i = 0; i < numberOfModules; i++) {
            vectorList.add(Constants.kModulePositions.get(i).rotateBy(Rotation2d.fromDegrees(rotateVectorDirection)));
            moduleRotations.add(Constants.kModulePositions.get(i).getAngle());
        }
        moduleRotationVectors = vectorList;
    }
    /**
     * 
     * @param translationVector -A vector that that is usually represented by the translational stick on the controller
     * @param rotationalMagnitude -The amount of rotation of the swerve drive represented by the rotational stick on the controller
     * @param robotPosition -The Position of the robot, with respect to the Rotation
     * @param robotCentric -Should the determine it's heading based off of the robot. False = Field Centric(i.e, when you push forward, 
     *                      the robot will always move in the same direction, no matter it's heading)
     */
    public List<Translation2d> updateDriveVectors(Translation2d translationVector, double rotationalMagnitude, Pose2d robotPosition, boolean robotCentric, boolean Velocity) {
        List<Translation2d> driveVectors = new ArrayList<>(kNumberOfModules);
        if(!robotCentric)
            translationVector = translationVector.rotateBy(robotPosition.getRotation().inverse()); //Rotates by the translation vector by inverse rotation of the robot 
        for(int i = 0; i < kNumberOfModules; i++) {
            driveVectors.add(translationVector.translateBy(moduleRotationVectors.get(i).scale(rotationalMagnitude))); //Rotates the translation vector of each of the modules, by the rotation value
        }

        double maxMagnitude = 1.0;
        for (Translation2d m : driveVectors) {
            double moduleVectorMagnitude = m.norm();
            if(moduleVectorMagnitude > maxMagnitude)
                maxMagnitude = moduleVectorMagnitude;
        }
        for (int i =0; i< kNumberOfModules; i++) {
            Translation2d driveVector = driveVectors.get(i);
            driveVectors.set(i, driveVector.scale(1.0/maxMagnitude*(Velocity?Constants.SwerveMaxspeedMPS:1)));
        }
        
        return driveVectors;
        
    }

     public ChassisSpeeds toChassisSpeedWheelConstraints(List<SwerveDriveModule> modules) {
        if (modules.size() != kNumberOfModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var constraintsMatrix = new SimpleMatrix(kNumberOfModules * 2, 3);
        for (int i = 0; i < kNumberOfModules; i++) {
            SwerveModuleState module = modules.get(i).getSwerveModuleState();

            Rotation2d beta =
                    module.angle.rotateBy(moduleRotations.get(i).inverse()).rotateBy(Rotation2d.fromRadians(Math.PI / 2.0));

            //System.out.println(module);
            constraintsMatrix.setRow(i*2, 0,
                    module.angle.cos(),
                    module.angle.sin(),
                    -Constants.kModulePositions.get(i).norm()*beta.cos());
            constraintsMatrix.setRow(i*2 + 1, 0,
                    -module.angle.sin(),
                    module.angle.cos(),
                    Constants.kModulePositions.get(i).norm()*beta.sin());
        }
        //System.out.println(constraintsMatrix);

        var psuedoInv = constraintsMatrix.pseudoInverse();

        var enforcedConstraints = new SimpleMatrix(kNumberOfModules*2, 1);
        for (int i = 0; i < kNumberOfModules; i++) {
            enforcedConstraints.setRow(i*2, 0, modules.get(i).getSwerveModuleState().speedMetersPerSecond);
            enforcedConstraints.setRow(i*2 + 1, 0, 0);
        }
        //System.out.println(enforcedConstraints);

        var chassisSpeedsVector = psuedoInv.mult(enforcedConstraints);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }
}
