// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.lib.swerve;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc.Constants;
import com.team5817.frc.subsystems.Swerve.SwerveDriveModule;
import com.team5817.frc.subsystems.gyros.Gyro;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveOdometry {
    private Pose2d m_poseMeters;
    private SwerveKinematics m_kinematics;
    private ChassisSpeeds m_velocity;
    private double m_previousTimestamp = -1;


    public SwerveOdometry(SwerveKinematics kinematics,Pose2d initalPose, double... previousDistances){
        m_kinematics = kinematics;
        m_poseMeters = initalPose;
        m_velocity = new ChassisSpeeds();
    }

    public SwerveOdometry(SwerveKinematics kinematics, Pose2d initalPose){
        this(kinematics, initalPose, new double[4]);
    }

    public ChassisSpeeds getVelocity(){
        return m_velocity;
    }
    public SwerveModulePosition[] getModulePositions(List<SwerveDriveModule> modules){
        SwerveModulePosition[] positions = {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
        for(int i= 0; i<modules.size();i++){
            positions[i] = new SwerveModulePosition(modules.get(i).getDistanceTraveled(),Rotation2d.fromDegrees(modules.get(i).getModuleAngle()).toWPI());
        }
        return positions;
    }

        SwerveModulePosition[] lastModulePositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };


    public Pose2d updateWithSwerveModuleStates(Gyro gyro, List<SwerveDriveModule> modules, double timestamp){
            double x = 0.0;
            double y = 0.0;      
            double averageDistance = 0.0;
            double[] distances = new double[4];
            for (SwerveDriveModule m : modules) {
                m.updatePose(Rotation2d.fromDegrees(gyro.getAngle()));
                double distance = m.getEstimatedRobotPose().getTranslation().translateBy(m_poseMeters.getTranslation().inverse())
                        .norm();
                distances[0] = distance;
                averageDistance += distance;
            }
            averageDistance /= modules.size();

            m_velocity = m_kinematics.toChassisSpeedWheelConstraints(modules);
            int minDevianceIndex = 0;
            double minDeviance = Units.inchesToMeters(100);
            List<SwerveDriveModule> modulesToUse = new ArrayList<>();
            for (SwerveDriveModule m : modules) {
                double deviance = Math.abs(distances[0] - averageDistance);
                if (deviance < minDeviance) {
                    minDeviance = deviance;
                    minDevianceIndex = 0;
                }
                if(deviance <= 1000){
					modulesToUse.add(m);
				}
            }
      
            if (modulesToUse.isEmpty()) {
                modulesToUse.add(modules.get(minDevianceIndex));
            }
            
            for (SwerveDriveModule m : modulesToUse) {
                x += m.getEstimatedRobotPose().getTranslation().x();
                y += m.getEstimatedRobotPose().getTranslation().y();
            }
            Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), Rotation2d.fromDegrees(gyro.getAngle()));
            Translation2d deltaPos = updatedPose.getTranslation().translateBy(m_poseMeters.getTranslation().inverse());
            m_velocity.vxMetersPerSecond = deltaPos.scale(1 / (timestamp - m_previousTimestamp)).x();
            m_velocity.vyMetersPerSecond = deltaPos.scale(1 / (timestamp - m_previousTimestamp)).y();
      
            m_poseMeters= updatedPose;
      
            modules.forEach((m) -> m.resetPose(m_poseMeters));
            m_previousTimestamp = timestamp;

            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = getModulePositions(modules);
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }
            SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(Constants.ModulePositions);
            Twist2d twist = new Twist2d(swerveDriveKinematics.toTwist2d(moduleDeltas));
                
            gyro.update(Rotation2d.fromRadians(twist.dtheta*100));
            Logger.recordOutput("dtheta", twist.dtheta*100);
            return m_poseMeters;
    }

    public Pose2d getPoseMeters(){
        return m_poseMeters;
    }
    
    public void resetPosition(Pose2d pose, List<SwerveDriveModule> modules){
        m_poseMeters = pose;
        modules.forEach((m) -> m.resetPose(pose));
    }
}
