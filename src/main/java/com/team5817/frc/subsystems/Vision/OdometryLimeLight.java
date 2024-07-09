// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.Vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import com.team5817.frc.Constants;
import com.team5817.frc.Constants.LimelightConstants;
import com.team5817.frc.Field.AprilTag;
import com.team5817.frc.Field.FieldLayout;
import com.team5817.frc.loops.ILooper;
import com.team5817.frc.subsystems.RobotState;
import com.team5817.frc.subsystems.Subsystem;
import com.team5817.frc.subsystems.Requests.Request;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import com.team5817.lib.Vision.LimelightHelpers;
import com.team5817.lib.Vision.TargetInfo;
import com.team5817.lib.Vision.UndistortMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import static org.opencv.core.CvType.CV_64FC1;

public class OdometryLimeLight extends Subsystem {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
  public static OdometryLimeLight instance = null;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-up");
  PeriodicIO mPeriodicIO = new PeriodicIO();

  private Mat mCameraMatrix = new Mat(3, 3, CV_64FC1);
  private Mat mDistortionCoeffients = new Mat(1, 5, CV_64FC1);

  private boolean mOutputsHaveChanged = true;

  private static HashMap<Integer, AprilTag> mTagMap = FieldLayout.Red.kAprilTagMap;

  public static OdometryLimeLight getInstance() {
    if (instance == null)
      instance = new OdometryLimeLight();
    return instance;
  }

  private OdometryLimeLight() {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        mCameraMatrix.put(i, j, LimelightConstants.UNDISTORT_CONSTANTS.getCameraMatrix()[i][j]);
      }
    }
    for (int i = 0; i < 5; i++) {
      mDistortionCoeffients.put(0, i, LimelightConstants.UNDISTORT_CONSTANTS.getCameraDistortion()[i]);
    }
  }

  public static class VisionUpdate {
    private double timestamp;
    private Pose2d pose;
    private int tagId;


    public VisionUpdate(double timestamp, Pose2d pose) {
      this.timestamp = timestamp;
      this.pose = pose;

      
    }

    public double getTimestamp() {
      return timestamp;
    }

    public Pose2d getPose() {
      return pose;
    }


    public int getId() {
      return tagId;
    }
  }

  private void readInputsAndAddVisionUpdate() {
    final double timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.imageCaptureLatency = table.getEntry("cl").getDouble(LimelightConstants.IMAGE_CAPTURE_LATENCY);
    mPeriodicIO.latency = table.getEntry("tl").getDouble(0) / 1000.0 + mPeriodicIO.imageCaptureLatency / 1000.0;
    mPeriodicIO.givenPipeline = (int) table.getEntry("getpipe").getDouble(0);
    mPeriodicIO.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    mPeriodicIO.tagId = (int) table.getEntry("tid").getNumber(-1).doubleValue();
    mPeriodicIO.tx = table.getEntry("tx").getDouble(0);
    mPeriodicIO.ty = table.getEntry("ty").getDouble(0);
    mPeriodicIO.corners = table.getEntry("tcornxy").getNumberArray(new Number[] { 0, 0, 0, 0, 0 });
    mPeriodicIO.ta = table.getEntry("ta").getDouble(0);
    LimelightHelpers.SetRobotOrientation("limelight-up", 180-SwerveDrive.getInstance().getRobotHeading().getDegrees(), 0, 0, 0, 0, 0);
    Pose2d mt2 = new Pose2d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-up").pose);
    


    if (mPeriodicIO.seesTarget) {
      if (mt2 != Pose2d.identity()) {
        mPeriodicIO.visionUpdate = Optional 
            .of(new VisionUpdate(timestamp - mPeriodicIO.latency, mt2));
        RobotState.getInstance().addVisionUpdate(
            mPeriodicIO.visionUpdate.get());

      } else {
        mPeriodicIO.visionUpdate = Optional.empty();
      }
    } else {
      mPeriodicIO.visionUpdate = Optional.empty();
    }
  }

  public Optional<VisionUpdate> getLatestVisionUpdate() {
    return mPeriodicIO.visionUpdate;
  }

  public synchronized Translation2d getCameraToTargetTranslation() {
    // Get all Corners Normalized
    List<TargetInfo> targetPoints = getTarget();
    if (targetPoints == null || targetPoints.size() < 4) {
      return null;
    }
    // Project Each Corner into XYZ Space
    Translation2d cameraToTagTranslation = Translation2d.identity();
    List<Translation2d> cornerTranslations = new ArrayList<>(targetPoints.size());
    for (int i = 0; i < targetPoints.size(); i++) {
      Translation2d cameraToCorner;

      // Add 3 Inches to the Height of Top Corners
      if (i < 2) {
        cameraToCorner = getCameraToPointTranslation(targetPoints.get(i), true);
      } else {
        cameraToCorner = getCameraToPointTranslation(targetPoints.get(i), false);
      }
      if (cameraToCorner == null) {
        return null;
      }
      cornerTranslations.add(cameraToCorner);
      cameraToTagTranslation = cameraToTagTranslation.translateBy(cameraToCorner);
    }

    // Divide by 4 to get the average Camera to Goal Translation
    cameraToTagTranslation = cameraToTagTranslation.scale(0.25);

    return cameraToTagTranslation;

  }

  public synchronized Translation2d getCameraToPointTranslation(TargetInfo target, boolean isTopCorner) {
    // Compensate for camera pitch
    Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ())
        .rotateBy(Rotation2d.fromDegrees(LimelightConstants.kCameraPitch.getDegrees()));
    double x = xz_plane_translation.x();
    double y = target.getY();
    double z = xz_plane_translation.y();

    double offset = isTopCorner ? Units.inchesToMeters(3.25) : -Units.inchesToMeters(3.25);
    // find intersection with the goal
    double differential_height = mTagMap.get(target.getTagId()).getHeight()
        - Units.inchesToMeters(LimelightConstants.kCameraHeightMeters) + offset;
    if ((z > 0.0) == (differential_height > 0.0)) {
      double scaling = differential_height / z;
      double distance = Math.hypot(x, y) * scaling;
      Rotation2d angle = new Rotation2d(x, y, true);
      return new Translation2d(distance * angle.cos(), distance * angle.sin());
    }
    return null;
  }

  private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

  public List<TargetInfo> getTarget() {
    // Get corners
    List<Translation2d> corners = getCorners(mPeriodicIO.corners);

    if (corners.size() < 4 || !mTagMap.containsKey(mPeriodicIO.tagId)) {
      return null;
    }

    // Sort by y, list will have "highest in image" corner first
    corners.sort(ySort);
    ArrayList<TargetInfo> targetInfos = new ArrayList<>();

    for (Translation2d corner : corners) {
      targetInfos.add(getRawTargetInfo(new Translation2d(corner.x(), corner.y()), getTagId()));

    }

    return targetInfos;
  }

  private static List<Translation2d> getCorners(Number[] tcornxy) {
    // Check if there is a non even number of corners
    if (tcornxy.length % 2 != 0) {
      return List.of();
    }

    ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
    for (int i = 0; i < tcornxy.length; i += 2) {
      corners.add(new Translation2d(tcornxy[i].doubleValue(), tcornxy[i + 1].doubleValue()));
    }

    return corners;
  }

  public synchronized TargetInfo getRawTargetInfo(Translation2d desiredTargetPixel, int tagId) {
    if (desiredTargetPixel == null) {
      return null;
    } else {
      double[] undistortedNormalizedPixelValues;
      UndistortMap undistortMap = LimelightConstants.UNDISTORTMAP;

      undistortedNormalizedPixelValues = undistortMap.pixelToUndistortedNormalized((int) desiredTargetPixel.x(),
          (int) desiredTargetPixel.y());
      double y_pixels = undistortedNormalizedPixelValues[0];
      double z_pixels = undistortedNormalizedPixelValues[1];

      double nY = -(y_pixels - mCameraMatrix.get(0, 2)[0]);// -(y_pixels * 2.0 - 1.0);
      double nZ = -(z_pixels - mCameraMatrix.get(1, 2)[0]);// -(z_pixels * 2.0 - 1.0);

      double y = nY / mCameraMatrix.get(0, 0)[0];
      double z = nZ / mCameraMatrix.get(1, 1)[0];

      return new TargetInfo(y, z, tagId);
    }
  }

  public int getTagId() {
    return mPeriodicIO.tagId;
  }

  public Request hasTargetRequest() {
    return new Request() {
      @Override
      public boolean isFinished() {
        return mPeriodicIO.seesTarget;
      }
    };
  }

  public double getPivotAngle() {
    return 0;
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
    public Number[] corners;
    public int pipeline = 0;
    public double stream = 0;
    public double snapshot = 0;
    public int tagId = 0;
    public boolean seesTarget = false;
    public int givenPipeline = 0;
    public Optional<VisionUpdate> visionUpdate = Optional.empty();
    public double ta = 0;
    public double tx = 0;
    public double ty = 0;
  }

  @Override
  public void writePeriodicOutputs() {
    if (mOutputsHaveChanged) {
      table.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
    }
  }

  @Override
  public void registerEnabledLoops(ILooper looper) {

    readInputsAndAddVisionUpdate();
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void stop() {

  }

}
