// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.Vision;

import java.util.Optional;


import com.team5817.frc.Constants;
import com.team5817.frc.Constants.LimelightConstants;
import com.team5817.frc.loops.ILooper;
import com.team5817.frc.loops.Loop;
import com.team5817.frc.subsystems.Subsystem;
import com.team5817.frc.subsystems.Requests.Request;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


public class ObjectLimeLight extends Subsystem {

  public static ObjectLimeLight instance = null;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-down");
  PeriodicIO mPeriodicIO = new PeriodicIO();

  private boolean mOutputsHaveChanged = true;


  public static ObjectLimeLight getInstance() {
    if (instance == null)
      instance = new ObjectLimeLight();
    return instance;
  }

  private ObjectLimeLight() {

}
  public static class VisionObjectUpdate {
    private double timestamp;
    private Translation2d cameraToTarget;

    public VisionObjectUpdate(double timestamp, Translation2d cameraToTarget) {
      this.timestamp = timestamp;
      this.cameraToTarget = cameraToTarget;
    }

    public double getTimestamp() {
      return timestamp;
    }

    public Translation2d getCameraToTarget() {
      return cameraToTarget;
    }
  }
  double timestamp = 0;
  private void readInputsAndAddVisionUpdate() {
    timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.tx = table.getEntry("tx").getDouble(0);
    mPeriodicIO.ty = table.getEntry("ty").getDouble(0);
    mPeriodicIO.imageCaptureLatency = table.getEntry("cl").getDouble(LimelightConstants.IMAGE_CAPTURE_LATENCY);
    mPeriodicIO.latency = table.getEntry("tl").getDouble(0) / 1000.0 + mPeriodicIO.imageCaptureLatency / 1000.0;
    mPeriodicIO.givenPipeline = (int) table.getEntry("getpipe").getDouble(0);
    mPeriodicIO.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    Translation2d cameraToTarget = new Translation2d(mPeriodicIO.tx, mPeriodicIO.ty);
    if (mPeriodicIO.seesTarget) {
        mPeriodicIO.visionUpdate = Optional.of(new VisionObjectUpdate(timestamp, cameraToTarget));
    }
    else{
        mPeriodicIO.visionUpdate = Optional.empty();
    }
  }
  public Optional<VisionObjectUpdate> getLatestVisionUpdate(){
    return mPeriodicIO.visionUpdate;
  }



  public void setPipeline(Integer pipeline) {
    mPeriodicIO.pipeline = pipeline;
    mOutputsHaveChanged = true;
  }

  public Request pipleLineRequest(int pipeline) {
    return new Request() {
      public void act() {
        setPipeline(pipeline);
      }
    };
  }

  public static class PeriodicIO {

    public double imageCaptureLatency = 0;
    public double latency = 0;
    public int pipeline = 0;
    public double stream = 0;
    public double snapshot = 0;
    public boolean seesTarget = false;
    public double tx = 0;
    public double ty = 0;
    public int givenPipeline = 0;
    public Optional<VisionObjectUpdate> visionUpdate = null;

  }

  @Override
  public void writePeriodicOutputs() {
    if (mOutputsHaveChanged) {
      table.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
    }
  }

@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
            readInputsAndAddVisionUpdate();
      }

			@Override
			public void onStop(double timestamp) {
			}
		});
	}
  

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void stop() {

  }

}
