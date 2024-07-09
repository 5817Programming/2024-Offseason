// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc.Constants;
import com.team5817.frc.Options;
import com.team5817.frc.Ports;
import com.team5817.frc.subsystems.Subsystem;
import com.team5817.frc.subsystems.encoders.CANEncoder;
import com.team5817.frc.subsystems.encoders.Encoder;
import com.team5817.frc.subsystems.encoders.MagEncoder;
import com.team5817.lib.Conversions;
import com.team5817.lib.TalonConfigs;
import com.team5817.lib.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.lib.swerve.SwerveModuleState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class SwerveDriveModule extends Subsystem {

    TalonFX rotationMotor, driveMotor;
    Encoder rotationMagEncoder;
    String name;
    public int moduleID;
    double encoderOffset;

    private double previousEncDistance = 0;
	private Translation2d position = new Translation2d();
    private Translation2d mstartingPosition;

	private Pose2d estimatedRobotPose = new Pose2d();
	boolean standardCarpetDirection = true;
    
    Translation2d modulePosition;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

    boolean rotationEncoderFlipped;

    PeriodicIO mPeriodicIO = new PeriodicIO();
    

    /**
     * 
     * @param rotationMotorPort  -The Drive Motor Port
     * @param driveMotorPort     -The Drive Motor Port
     * @param moduleID           -The ID of the module
     * @param encoderStartingPos -The starting encoder position(used for zeroing
     *                           purposes)
     * @param modulePoseInches         -The position of the module relative to the robot's
     *                           center
     * @param flipEncoder        -Is the encoder going in the right direction?
     *                           (clockwise = increasing, counter-clockwise =
     *                           decreasing)
     */
    DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), 0.99, .025);
    
    public SwerveDriveModule(int rotationMotorPort, int driveMotorPort, int moduleID, double encoderStartingPos,
            Translation2d modulePoseInches, boolean flipEncoder ,Translation2d moduleposemeters){
        this.rotationMotor = new TalonFX(rotationMotorPort, Constants.isCompbot? "Minivore": "");
        this.driveMotor = new TalonFX(driveMotorPort, Constants.isCompbot? "Minivore": "");
        this.moduleID = moduleID;
        this.name = "Module " + moduleID;
        this.encoderOffset = encoderStartingPos;
        this.modulePosition = modulePoseInches;
        this.mstartingPosition =moduleposemeters;
        

        this.rotationEncoderFlipped = flipEncoder;
        
        if (Options.encoderType == "Mag Encoder") {
            rotationMagEncoder = new MagEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        } 
        else if(Options.encoderType == "CANCoder") {
         rotationMagEncoder = new CANEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        }
        configMotors();
    }
    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
    TalonFXConfiguration rotationConfigs = new TalonFXConfiguration();

    public void configMotors(){
        driveConfigs = TalonConfigs.swerveDriveConfig();
        rotationConfigs = TalonConfigs.swerveRotationConfig();

        driveMotor.getConfigurator().apply(driveConfigs);
        rotationMotor.getConfigurator().apply(rotationConfigs);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setPosition(0);

    }

    public enum ControlMode{
        PercentOuput,
        MotionMagic,
        Velocity
      }

    public void invertDriveMotor(boolean invert) {
        driveMotor.setInverted(invert);
    }

    public void invertRotationMotor(Boolean invertType) {
        rotationMotor.setInverted(invertType);
    }

    public void invertRotationMotor(boolean invert) {
        rotationMotor.setInverted(invert);
    }

    public void setDriveMotorNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    public void setModuleAngle(double desiredAngle) {
        desiredAngle = Util.placeInAppropriate0To360Scope(getModuleAngle(), desiredAngle);// optimization
        double angleRotations = degreesToRotations(desiredAngle);
        mPeriodicIO.rotationDemand = angleRotations;
    }

    public double getModuleAngle() {
        return rotationsToDegrees(mPeriodicIO.rotationPosition);
    }

    public double getModuleAbsolutePosition() {
        return rotationMagEncoder.getOutput() * ((this.rotationEncoderFlipped) ? -1 : 1) * 360.0;
    }

    public boolean isMagEncoderConnected() {
        return rotationMagEncoder.isConnected();
    }


    public double getDistanceTraveled(){
        return mPeriodicIO.distanceTraveled;
    }
    public void resetEncoders() {
        driveMotor.setPosition(0);

    }
    public synchronized void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(mstartingPosition)).getTranslation();
		position = modulePosition;
	}

    public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle =Rotation2d.fromDegrees( getModuleAngle());
		return normalizedAngle.rotateBy(robotHeading);
	}
	
    public double encUnitsToInches(double encUnits){
		return encUnits/Constants.kSwerveEncUnitsPerInch;
	}
    public void setVelocityPercent(double percent){
        mPeriodicIO.driveDemand = percent*Constants.SwerveMaxspeedMPS;
        mPeriodicIO.driveControlMode = ControlMode.Velocity;
    }
    
    public void setDriveOpenLoop(double percentOuput) {
        mPeriodicIO.driveControlMode = ControlMode.PercentOuput;
        mPeriodicIO.driveDemand = percentOuput;
    }

    public void setDriveVelocity(double velocity) {
        mPeriodicIO.driveControlMode = ControlMode.Velocity;
        mPeriodicIO.driveDemand = (velocity/Constants.kWheelCircumference)*Options.driveRatio;
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(Conversions.falconToMPS(
            mPeriodicIO.driveVelocity,
            Constants.kWheelCircumference,
            Options.driveRatio),
            mPeriodicIO.distanceTraveled ,
            Rotation2d.fromDegrees(rotationsToDegrees(mPeriodicIO.rotationPosition)
            ));
    }
    
    public synchronized void updatePose(Rotation2d robotHeading){
		double currentEncDistance = Conversions.falconToMeters(mPeriodicIO.drivePosition, Constants.kWheelCircumference, Options.driveRatio);
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(-currentWheelAngle.cos()*deltaEncDistance, 
				currentWheelAngle.sin()*deltaEncDistance);


		double xScrubFactor = Constants.kXScrubFactorN;
		double yScrubFactor = Constants.kYScrubFactorN;
        if(Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)){
            xScrubFactor = Constants.kXScrubFactorP;

        }
        if(Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)){
            yScrubFactor = Constants.kYScrubFactorP;
        }
	
		deltaPosition = new Translation2d(deltaPosition.x() * xScrubFactor,
			deltaPosition.y() * yScrubFactor);
        mPeriodicIO.deltaDistanceTraveled = deltaPosition.norm();
        mPeriodicIO.distanceTraveled += deltaPosition.norm();
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(mstartingPosition).inverse());
		position = updatedPosition;
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
        Logger.recordOutput("Swerve/Module "+moduleID+"/Position", Pose2d.fromTranslation(updatedPosition).toWPI());

	}

    public double degreesToRotations(double degrees){
        return (degrees/360)*Options.rotationRatio;
    }

    public double rotationsToDegrees(double rotations){
        return (rotations*360)/Options.rotationRatio;
    }

    public void resetModulePositionToAbsolute() {
                driveMotor.setPosition(0);
                double offset = getModuleAbsolutePosition() - encoderOffset;
                rotationMotor.setPosition(degreesToRotations(offset));
    }

    public enum ModuleStatus {
        OK, ABSOLUTE_ENCODER_ERROR, DRIVE_MOTOR_ERROR, ROTATION_MOTOR_ERROR;
    }

   
    public ModuleStatus getModuleStatus(){
        return mPeriodicIO.status;
    }
    public void updateModuleStatus() {
        if (!isMagEncoderConnected())
            mPeriodicIO.status = ModuleStatus.ABSOLUTE_ENCODER_ERROR;
        else if (driveMotor.isAlive())
            mPeriodicIO.status = ModuleStatus.DRIVE_MOTOR_ERROR;
        else if (rotationMotor.isAlive())
            mPeriodicIO.status = ModuleStatus.ROTATION_MOTOR_ERROR;
        mPeriodicIO.status = ModuleStatus.OK;
    }

    @Override
    public void writePeriodicOutputs() {
        switch (Constants.currentMode) {
            case REAL:
                updateModuleStatus();
                mPeriodicIO.rotationPosition = rotationMotor.getPosition().getValue();
                mPeriodicIO.drivePosition = driveMotor.getPosition().getValue();
                mPeriodicIO.driveVelocity = driveMotor.getVelocity().getValue();
                break;
            case SIM:
                mPeriodicIO.rotationPosition = mPeriodicIO.rotationDemand;
                mPeriodicIO.drivePosition = driveSim.getAngularPositionRotations();
                mPeriodicIO.driveVelocity = driveSim.getAngularVelocityRPM()*60;
                driveSim.setState(driveSim.getAngularPositionRad(), driveSim.getAngularVelocityRadPerSec()*0.99);
            default:
                break;
        }
        
    }
    double lastTimeStamp = 0;
      @Override
  public void readPeriodicInputs() {
    double timestamp = Timer.getFPGATimestamp();
    driveSim.update(timestamp-lastTimeStamp);
    switch (mPeriodicIO.driveControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.driveDemand, driveMotor);
          driveSim.setInputVoltage(12*mPeriodicIO.driveDemand);
        break;
        default:
            break;
    }
    switch (mPeriodicIO.rotationControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.rotationDemand, rotationMotor);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.rotationDemand, rotationMotor);
        default:
          runMotionMagic(mPeriodicIO.rotationDemand, rotationMotor);
        break;
    }
    lastTimeStamp = timestamp;
  }

  public void runPercentOutput(double percent, TalonFX motor){
    motor.setControl(new DutyCycleOut(percent, true, false, false, false));
  }
    public void runMotionMagic(double position, TalonFX motor){
    motor.setControl(new MotionMagicVoltage(position).withEnableFOC(false ));
  }

  public void runVelocity(double velocity, TalonFX motor){
    motor.setControl(new MotionMagicVelocityVoltage(velocity).withSlot(0).withEnableFOC(true));

  }
    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Swerve/"+this.name + "/Angle Demand", rotationsToDegrees(mPeriodicIO.rotationDemand));
        Logger.recordOutput("Swerve/"+this.name + "/Angle", rotationsToDegrees(mPeriodicIO.rotationPosition));
        Logger.recordOutput("Swerve/"+this.name + "/Absolute Position", getModuleAbsolutePosition());
        Logger.recordOutput("Swerve/"+this.name + "/Drive Motor Demand", mPeriodicIO.driveDemand);
        Logger.recordOutput("Swerve/"+this.name + "/Status", mPeriodicIO.status);
        Logger.recordOutput("Swerve/"+this.name + "/Drive Position", mPeriodicIO.drivePosition);
        Logger.recordOutput("Swerve/"+this.name + "/Drive velocity", mPeriodicIO.driveVelocity);
        Logger.recordOutput("Swerve/"+this.name + "/Drive Control Mode", mPeriodicIO.driveControlMode);
        Logger.recordOutput("Swerve/"+this.name + "/Rotation Control Mode", mPeriodicIO.rotationControlMode);
    }

    @Override
    public void stop() {
        setModuleAngle(getModuleAngle());
        setDriveOpenLoop(0.0);
    }

    public static class PeriodicIO {
        double deltaDistanceTraveled = 0;;
        double rotationPosition = 0;
        double drivePosition = 0;
        double driveVelocity = 0;
        double distanceTraveled = 0;
        ModuleStatus status = ModuleStatus.OK;
        ControlMode rotationControlMode = ControlMode.MotionMagic;
        ControlMode driveControlMode = ControlMode.PercentOuput;
        double rotationDemand = 0.0;
        double driveDemand = 0.0;
    }

}
