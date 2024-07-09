// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc;

import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.team5817.frc.Autos.AutoBase;
import com.team5817.frc.Autos.M6;
import com.team5817.frc.Autos.Shoot;
import com.team5817.frc.Controls.Controls;
import com.team5817.frc.loops.Looper;
import com.team5817.frc.subsystems.Climb;
import com.team5817.frc.subsystems.Indexer;
import com.team5817.frc.subsystems.Intake;
import com.team5817.frc.subsystems.Lights;
import com.team5817.frc.subsystems.Pivot;
import com.team5817.frc.subsystems.RobotState;
import com.team5817.frc.subsystems.RobotStateEstimator;
import com.team5817.frc.subsystems.Shooter;
import com.team5817.frc.subsystems.SuperStructure;
import com.team5817.frc.subsystems.SuperStructure.SuperState;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import com.team5817.frc.subsystems.Vision.ObjectLimeLight;
import com.team5817.frc.subsystems.Vision.OdometryLimeLight;
import com.team5817.frc.subsystems.gyros.Gyro;
import com.team5817.lib.PathStateGenerator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;



public class Robot extends LoggedRobot {

  Controls controls;
  SubsystemManager subsystemManager;
  SuperStructure s = SuperStructure.getInstance();
  SwerveDrive swerve;
  double yaw;
  OdometryLimeLight vision;
  Gyro pigeon;
  AutoBase auto = new M6();
  public LoggedDashboardChooser<AutoBase> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
	// enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
  @Override
  public void robotInit() {
    autos.put("Middle 6", new M6());


    autos.put("1", new Shoot());

    DriverStation.startDataLog(DataLogManager.getLog());

    RobotState.getInstance().resetKalmanFilters(Timer.getFPGATimestamp());
    for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
      String N = entry.getKey();
      AutoBase A = entry.getValue();
      autoChooser.addOption(N, A);
    }

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    swerve = SwerveDrive.getInstance();
    controls = Controls.getInstance();
    vision = OdometryLimeLight.getInstance();
    swerve.zeroModules();
    subsystemManager = SubsystemManager.getInstance();

    subsystemManager.setSubsystems(
        SwerveDrive.getInstance(),
        SuperStructure.getInstance(), 
        OdometryLimeLight.getInstance(),
        RobotStateEstimator.getInstance(),
        ObjectLimeLight.getInstance(),
        Shooter.getInstance(),
        Indexer.getInstance(),
        Intake.getInstance(),
        Pivot.getInstance(),
        Climb.getInstance(),
        Lights.getInstance()
        );
        subsystemManager.registerEnabledLoops(mEnabledLooper);
        subsystemManager.registerDisabledLoops(mDisabledLooper);

    }

  @Override
  public void robotPeriodic() {
    subsystemManager.outputLoopTimes();
    Logger.recordOutput("timestamp", Timer.getFPGATimestamp());
  }



  @Override
  public void autonomousInit() {
    auto = autoChooser.get();
    swerve = SwerveDrive.getInstance();
    swerve.fieldzeroSwerve();
    swerve.zeroModules();
    SuperStructure.getInstance().setState(SuperState.AUTO);
    Pivot.getInstance().conformToState(Pivot.State.MAXUP);
    auto.runAuto();
    Indexer.getInstance().setPiece(true);
    mDisabledLooper.stop();
		mEnabledLooper.start();
    }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { 
    auto.updateAuto();
  }

  /** This function is called once when teleop is enabled. */  
  @Override
  public void teleopInit() {
    swerve = SwerveDrive.getInstance();
    // swerve.fieldzeroSwerve();
    swerve.zeroModules();
    RobotStateEstimator.getInstance().resetOdometry(new Pose2d(15.18,5.48,Rotation2d.kIdentity));
    mDisabledLooper.stop();
		mEnabledLooper.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controls.update();
  }

  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {
    SuperStructure.getInstance().clearQueues();
    PathStateGenerator.getInstance().resetTimer();
    mDisabledLooper.stop();
		mEnabledLooper.start();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    mDisabledLooper.stop();
		mEnabledLooper.start();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    

  }
}
