// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root dir[]ectory of this project.

package com.team5817.frc.subsystems.Swerve;


import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc.Constants;
import com.team5817.frc.Options;
import com.team5817.frc.Ports;
import com.team5817.frc.Planners.AimingPlanner;
import com.team5817.frc.Planners.AutoAlignMotionPlanner;
import com.team5817.frc.Planners.DriveMotionPlanner;
import com.team5817.frc.Planners.TargetPiecePlanner;
import com.team5817.frc.Planners.AimingPlanner.AimingRequest;
import com.team5817.frc.subsystems.RobotState;
import com.team5817.frc.subsystems.Subsystem;
import com.team5817.frc.subsystems.Requests.Request;
import com.team5817.frc.subsystems.Swerve.SwerveDriveModule.ModuleStatus;
import com.team5817.frc.subsystems.Vision.ObjectLimeLight;
import com.team5817.frc.subsystems.Vision.OdometryLimeLight;
import com.team5817.frc.subsystems.gyros.Gyro;
import com.team5817.frc.subsystems.gyros.Pigeon;
import com.team5817.lib.Conversions;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team5817.lib.swerve.ChassisSpeeds;
import com.team5817.lib.swerve.HeadingController;
import com.team5817.lib.swerve.SwerveKinematics;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class SwerveDrive extends Subsystem {
    public static SwerveDrive instance = null;

    public static SwerveDrive getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;

    Translation2d aimingVector = new Translation2d();
    Translation2d translationVector = new Translation2d();
    public double rotationScalar = 0;
    double speedPercent = 1;
    double rotationalVel;


    Pigeon gyro;
    OdometryLimeLight odometryVision;
    ObjectLimeLight objectVision;
    Pose2d drivingPose;

    Pose2d poseMeters = new Pose2d();
    
    List<SwerveDriveModule> positionModules;
    Translation2d currentVelocity = new Translation2d();

    List<Translation2d> moduleVectors;

    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    double desiredRotationScalar;
    double distanceTraveled;

    double lastUpdateTimestamp = 0;



    SwerveKinematics inverseKinematics = new SwerveKinematics();
    AutoAlignMotionPlanner mAutoAlignMotionPlanner;
    DriveMotionPlanner mDriveMotionPlanner;
    AimingPlanner mAimingPlanner;
    TargetPiecePlanner mTargetPiecePlanner;
    RobotState robotState;
    HeadingController headingController = new HeadingController();


    double currentSpeed = 0;
    double bestDistance;
    boolean lob = false;
    boolean stateHasChanged = false;
    boolean modeHasChanged = false;
    double lastTimeStamp = 0;


    public SwerveDrive() {

        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
                Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,
                Constants.mFrontRightPosition);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
                Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,
                Constants.mFrontLeftPosition);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
                Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,
                Constants.mRearLeftPosition);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
                Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,
                Constants.mRearRightPosition);
        modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        // sets which ways the modules turn
        frontRightModule.invertDriveMotor(true);
        frontLeftModule.invertDriveMotor(false);
        rearLeftModule.invertDriveMotor(false);
        rearRightModule.invertDriveMotor(true);
        positionModules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);
        distanceTraveled = 0;

        gyro = Pigeon.getInstance();
        odometryVision = OdometryLimeLight.getInstance();
        objectVision = ObjectLimeLight.getInstance();
        robotState = RobotState.getInstance();
        mDriveMotionPlanner =  DriveMotionPlanner.getInstance();
        mAutoAlignMotionPlanner = new AutoAlignMotionPlanner();
        mAimingPlanner = new AimingPlanner();
        mTargetPiecePlanner = new TargetPiecePlanner();

    }

    public enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        TARGETOBJECT,
        AIMING,
        ALIGNMENT,
        SNAP,
    }

    public enum TrajectoryMode{
        TRACKING,
        FOLLOWING,
    }

    private State currentState = State.MANUAL;
    private TrajectoryMode currentMode = TrajectoryMode.FOLLOWING;

    public State getState() {
        return this.currentState;
    }

    public void setState(State desiredState) {
        if(desiredState != currentState)
            stateHasChanged = true;
        currentState = desiredState;
    }

    public void setMode(TrajectoryMode desiredMode){
        if(desiredMode != currentMode)
            modeHasChanged = true;
        currentMode = desiredMode;
    }
    public SwerveKinematics getKinematics(){
        return inverseKinematics;
    }

    public void resetModulePose(Pose2d newpose){
        modules.forEach((m) -> m.resetPose(newpose));
    }

    public void setSpeedPercent(double percent) {
        speedPercent = (1 - (percent * 0.3));
    }

    public void sendInput(double x, double y, double rotation) {
        translationVector = new Translation2d(x, y).scale(speedPercent);
        if (Math.abs(rotation) <= rotationDeadband) {
            rotation = 0;
        }
        if (rotation == 0 && rotationScalar != 0) {
            headingController.disable();
        }
        rotationScalar = rotation;
        final double scaleValue = 1.5;
        double inputMagnitude = translationVector.norm();
        inputMagnitude = Math.pow(inputMagnitude, scaleValue);
        inputMagnitude = Util.deadband(translationDeadband, inputMagnitude);
        if (translationVector.norm() <= translationDeadband) {
            translationVector = new Translation2d();
        }
        if(DriverStation.getAlliance().get().equals(Alliance.Blue))
            translationVector = translationVector.inverse();
        rotationScalar *= 0.025;
        if (translationVector.norm() <= translationDeadband && Math.abs(rotation) <= rotationDeadband) {
            this.commandModuleDrivePowers(0);
        } else {
            this.update();

        }
        this.update();

    }

    public void setAlignment(Pose2d pose){
        mAutoAlignMotionPlanner.setTargetPoint(pose);
        robotState.setDisplaySetpointPose(pose);
    }

    public void setLob(boolean newlob){
        this.lob = newlob;
    }

    public void commandModule(List<Translation2d> moduleVectors) {
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());

            }
        }
    }

    public void commandModuleVelocitys(List<Translation2d> moduleVectors){
         this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveVelocity(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveVelocity(moduleVectors.get(i).norm());

            }
        }
    }
    public void commandModuleVelocitysPercentages(List<Translation2d> moduleVectors){
         this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setVelocityPercent(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setVelocityPercent(moduleVectors.get(i).norm());

            }
        }
    }
    public void commandModuleDrivePowers(double power) {
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }

    public void resetPose(Pose2d newPose) {
        modules.forEach((m) -> m.resetPose(newPose));
    }

    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }
    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }
    public Gyro getGyro(){
        return gyro;
    }

    public List<SwerveDriveModule> getModules(){
        return modules;
    }
    
    public boolean isStatusOK(){
        for(SwerveDriveModule m:modules){
            if(m.getModuleStatus().equals(ModuleStatus.OK))
                return false;
        }
        return true;
    }

    @Override
    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        poseMeters = robotState.getKalmanPose(timeStamp);
        drivingPose = Pose2d.fromRotation(getRobotHeading());
        double rotationCorrection;
        switch (currentState) {
            case MANUAL:
                if(stateHasChanged){
                    headingController.setTargetHeading(getRobotHeading().inverse().flip());
                }
                rotationCorrection = headingController.updateRotationCorrection(drivingPose.getRotation().inverse().flip(),
                        timeStamp);
                if (translationVector.norm() == 0 || rotationScalar != 0) {
                    rotationCorrection = 0;
                }
                commandModule(inverseKinematics.updateDriveVectors(translationVector,
                        rotationScalar, drivingPose, robotCentric,false));
                break;

            case ALIGNMENT:
                if (stateHasChanged) {
                    mAutoAlignMotionPlanner.reset();
                }
                ChassisSpeeds targetChassisSpeeds = updateAutoAlign();
                commandModuleVelocitys(inverseKinematics.updateDriveVectors(new Translation2d(
                    -targetChassisSpeeds.vxMetersPerSecond,
                    targetChassisSpeeds.vyMetersPerSecond),
                    targetChassisSpeeds.omegaRadiansPerSecond*8,
                    poseMeters,
                    false,
                    true
                    ));
                break;

            case TRAJECTORY:
                if(DriverStation.isTeleop() && translationVector.norm() != 0){
                    setState(State.MANUAL);
                }
                mDriveMotionPlanner.updateTrajectory();
                switch (currentMode) {
                    case FOLLOWING:
                        translationVector = mDriveMotionPlanner.getTranslation2dToFollow(timeStamp);
                        headingController.setTargetHeading(mDriveMotionPlanner.getTargetHeading());

                        break;
                    case TRACKING:
                        if(modeHasChanged)
                            mDriveMotionPlanner.resetNoteTracking();
                        Pose2d targetPose = new Pose2d();
                        translationVector = targetPose.getTranslation();
                        headingController.setTargetHeading(targetPose.getRotation().inverse());
                        break;
                }
                        rotationCorrection = headingController.getRotationCorrection(getRobotHeading().inverse().flip(), timeStamp);
                        desiredRotationScalar = rotationCorrection;
                        commandModuleVelocitys(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, poseMeters,
                                robotCentric,true));
                break;
            case TARGETOBJECT:
                rotationCorrection = mTargetPiecePlanner.updateAiming(timeStamp, objectVision.getLatestVisionUpdate(), headingController, getRobotHeading());
                commandModule(
                        inverseKinematics.updateDriveVectors(translationVector.scale(.5),
                        rotationCorrection*.7+rotationScalar, drivingPose, robotCentric,false));
                break;
            case AIMING:
            Pose2d demandedAngle;
                    if(!lob)
                    demandedAngle = mAimingPlanner.updateAiming(
                        timeStamp,
                        RobotState.getInstance().getLatestPoseFromOdom().getValue(),
                        Pose2d.fromTranslation(RobotState.getInstance().getLatestVisionPoseComponent()),
                        AimingRequest.SPEAKER,
                        odometryVision.getLatestVisionUpdate(),
                        headingController,
                        robotState.getSmoothedVelocity());
                    else
                       demandedAngle = mAimingPlanner.updateAiming(
                        timeStamp,
                        RobotState.getInstance().getLatestPoseFromOdom().getValue(),
                        Pose2d.fromTranslation(RobotState.getInstance().getLatestVisionPoseComponent()),
                        AimingRequest.LOB,
                        odometryVision.getLatestVisionUpdate(),
                        headingController,
                        robotState.getSmoothedVelocity());
                commandModule(
                        inverseKinematics.updateDriveVectors(translationVector.scale(.3),
                        demandedAngle.getRotation().getDegrees()+rotationScalar, drivingPose, robotCentric,false));
                break;

            case OFF:
                commandModule(inverseKinematics.updateDriveVectors(new Translation2d(), 0, drivingPose, robotCentric,false));
                break;

            case SNAP:
                headingController.setTargetHeading(mDriveMotionPlanner.getTargetHeading());
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                commandModule(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, poseMeters,
                        robotCentric,false));
                break;

        }
        stateHasChanged = false;
    }


    public ChassisSpeeds updateAutoAlign(){
        final double now = Timer.getFPGATimestamp();
        var fieldToOdometry = robotState.getAbsoluteVisionPoseComponent(now);
        var odomToVehicle = robotState.getPoseFromOdom(now);
        ChassisSpeeds output = mAutoAlignMotionPlanner.updateAutoAlign(now, odomToVehicle, Pose2d.fromTranslation(fieldToOdometry), robotState.getMeasuredVelocity(),headingController, getRobotHeading());
        return output;
    }

    public void snap(double r) {
        setState(State.SNAP);
        headingController.setTargetHeading(Rotation2d.fromDegrees(r));
    }

    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(180);
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
        headingController.setTargetHeading(Rotation2d.fromDegrees(180));
            gyro.setAngle(0);
    }

    }

    @Override
    public void readPeriodicInputs() {
        modules.forEach((m) -> {
            m.readPeriodicInputs();
        });
    }

    @Override
    public void writePeriodicOutputs() {
        modules.forEach((m) -> {
            m.writePeriodicOutputs();
        });
    }

    public void resetGryo(double angle) {
        headingController.setTargetHeading(Rotation2d.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(0);
        resetPose(new Pose2d(new Translation2d(1.9, 4.52), Rotation2d.fromDegrees(0)));
    }

    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    public Request setStateRequest(State state) {
        return new Request() {
            @Override
            public void act() {
                setState(state);
            }
        };
    }

    public Request setModeRequest(TrajectoryMode mode) {
        return new Request() {
            @Override
            public void act() {
                setMode(mode);
            }
        };
    }

    public Request isAimedRequest(){
        return new Request() {
            @Override
            public boolean isFinished(){
                return mAimingPlanner.isAimed();
            }
        };
    }

    public Request isAlignedRequest(){
        return new Request() {
            @Override
            public boolean isFinished(){
                return mAutoAlignMotionPlanner.getAutoAlignmentCompleted();
            }
        };
    }
    public synchronized ChassisSpeeds getSetPoint(){
        var desiredThrottleSpeed = translationVector.x() * Constants.SwerveMaxspeedMPS;
        var desiredStrafeSpeed = translationVector.y() * Constants.SwerveMaxspeedMPS;
        var desiredRotationSpeed = rotationScalar * Conversions.falconToMPS(Constants.kSwerveRotationMaxSpeed, Constants.kWheelCircumference, Options.driveRatio);
        return ChassisSpeeds.fromRobotRelativeSpeeds(desiredThrottleSpeed, desiredStrafeSpeed, desiredRotationSpeed);
    }


    public void snapToPoint(Pose2d targetPoint){
        if(mAutoAlignMotionPlanner != null){
            if(currentState != State.ALIGNMENT){
                mAutoAlignMotionPlanner.reset();
                setState(State.ALIGNMENT);
            }
        }
        mAutoAlignMotionPlanner.setTargetPoint(targetPoint);
        robotState.setDisplaySetpointPose(targetPoint);
    }

    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Swerve/State", getState());
        modules.forEach((m) -> m.outputTelemetry());
        Logger.recordOutput("Swerve/Heading", getRobotHeading().getDegrees());
        Logger.recordOutput("Swerve/Desired Heading", headingController.getTargetHeading());
        }


    @Override
    public void stop() {// stops everything
        setState(State.MANUAL);
        translationVector = new Translation2d();
        rotationScalar = 0;

        update();
        commandModuleDrivePowers(0);

    }

}
