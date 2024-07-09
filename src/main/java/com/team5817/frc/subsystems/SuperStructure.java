// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.team5817.frc.Constants;
import com.team5817.frc.Constants.ShooterConstants;
import com.team5817.frc.Planners.DriveMotionPlanner;
import com.team5817.frc.Planners.ShootingUtils;
import com.team5817.frc.Planners.ShootingUtils.ShootingParameters;
import com.team5817.frc.subsystems.Lights.Color;
import com.team5817.frc.subsystems.Requests.Request;
import com.team5817.frc.subsystems.Requests.RequestList;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import com.team5817.frc.subsystems.Swerve.SwerveDrive.TrajectoryMode;
import com.team5817.frc.subsystems.Vision.OdometryLimeLight;
import com.team5817.lib.ChoreoEventMarker;
import com.team5817.lib.PathStateGenerator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    protected Intake mIntake;
    protected OdometryLimeLight vision;
    protected DriveMotionPlanner mDriveMotionPlanner;
    protected DriveMotionPlanner dMotionPlanner;
    protected RobotState mRobotState;
    protected Indexer mIndexer;
    protected Pivot mPivot;
    protected Shooter mShooter;
    protected SwerveDrive mDrive;
    protected Lights mLights;
    protected PathStateGenerator mPathStateGenerator;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        mPathStateGenerator = PathStateGenerator.getInstance();
        vision = OdometryLimeLight.getInstance();
        mDriveMotionPlanner = DriveMotionPlanner.getInstance();
        mRobotState = RobotState.getInstance();
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();
        mLights = Lights.getInstance();
        mPivot = Pivot.getInstance();
        mShooter = Shooter.getInstance();
        mDrive = SwerveDrive.getInstance();
        queuedRequests = new ArrayList<>();

    }

    public static SuperStructure instance = null;

    public static SuperStructure getInstance() {
        if (instance == null)
            instance = new SuperStructure();
        return instance;
    }

    private RequestList activeRequests;
    Request currentRequest;

    private boolean newRequests;
    private boolean activeRequestsComplete = true;
    private String currentRequestLog;
    private boolean stateChanged = false;
    private SuperState currentState = SuperState.OFF;
    private Mode currentMode = Mode.SHOOTING;
    private boolean modeChanged = false;
    private boolean manual = false;
    private boolean pieceAim = true;
    private double pivotOffset = 0;

    private boolean continuousShoot = true;

    private boolean requestsCompleted() {
        return activeRequestsComplete;
    }

    boolean lockElevator = false;
    boolean cube = false;
    boolean isIntaking = false;

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsComplete = false;
    }

    public enum SuperState {
        INTAKING,
        SCORE,
        OFF,
        IDLE,
        AUTO,
        OUTTAKING
    }

    public enum Mode {
        SHOOTING,
        AMP,
        FIRING,
        AMPOVERIDE
    }

    public void setState(SuperState state) {
        if (currentState != state)
            stateChanged = true;
        currentState = state;
    }

    public void setMode(Mode mode) {
        if (currentMode != mode)
            modeChanged = true;
        currentMode = mode;
    }

    private void setQueueRequests(RequestList r) {
        queuedRequests.clear();
        queuedRequests.add(r);
    }

    private void setQueueRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList r : requests) {
            queuedRequests.add(r);
        }
    }

    private void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueueRequests(new RequestList());
    }

    private void request(RequestList r) {
        setActiveRequests(r);
        setQueueRequests(new RequestList());
    }

    public void addActiveRequests(Request r) {
        activeRequests.add(r);
        newRequests = true;
        activeRequestsComplete = false;
    }

    public void addForemostActiveRequest(Request request) {
        activeRequests.addToForefront(request);
        newRequests = true;
        activeRequestsComplete = false;
        activeRequestsComplete = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }

    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueueRequests(new RequestList(Arrays.asList(request), false));
    }

    public synchronized void clearQueues() {
        queuedRequests.clear();
        activeRequests = new RequestList();
        activeRequestsComplete = true;
    }

    public void replaceQueue(RequestList list) {
        setQueueRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueueRequests(lists);
    }

    public void processState(double timestamp) {
        Logger.recordOutput("SuperStructure/Mode", currentMode);
        Logger.recordOutput("SuperStructure/SuperState", currentState);
        switch (currentState) {
            case SCORE:
                switch (currentMode) {
                    case FIRING:
                        if (modeChanged) {
                            shootState(true);
                        }
                        prepareShooterSetpoints(timestamp, manual);
                        break;
                    case SHOOTING:
                        prepareShooterSetpoints(timestamp, manual);
                        mDrive.setLob(!inShootZone(timestamp));
                        mDrive.setState(SwerveDrive.State.AIMING);
                        if (stateChanged)
                            indicationState();
                        break;

                    case AMP:
                        mShooter.setSpin(1);
                        if (modeChanged || stateChanged) {
                            scoreAmpState();
                        }
                        mIndexer.setPiece(false);

                        break;
                    case AMPOVERIDE:
                        mShooter.setSpin(1);
                        if (modeChanged || stateChanged) {
                            overrideAmpState();
                        }
                        mIndexer.setPiece(false);;
                }

                break;

            case INTAKING:
                if (stateChanged && !DriverStation.isAutonomous()) {
                    intakeState();
                    if (pieceAim)
                        mDrive.setState(SwerveDrive.State.TARGETOBJECT);
                    else
                        mDrive.setState(SwerveDrive.State.MANUAL);
                }
                break;
            case OUTTAKING:
                mIndexer.conformToState(Indexer.State.OUTTAKING);
                mIndexer.setHasPieceRequest(false).act();
                mIntake.conformToState(Intake.State.OUTTAKING);
                break;
            case IDLE:
                if (stateChanged)
                    clearQueues();
                if (mIndexer.hasPiece())
                    mLights.setColor(Color.HASPIECE);
                else if (mDrive.isStatusOK()&&currentMode!=Mode.AMP)
                    mLights.setColor(Color.IDLE);
                else if(currentMode == Mode.AMP)
                    mLights.setColor(Color.AMP);
                else
                    mLights.setColor(Color.ERROR);
                mDrive.setState(SwerveDrive.State.MANUAL);
                mIntake.conformToState(Intake.State.OFF);
                mIndexer.conformToState(Indexer.State.OFF);
                if(currentMode == Mode.SHOOTING || currentMode == Mode.FIRING){
                    if(inShootZone(timestamp) && mIndexer.hasPiece()){
                        ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp));
                        mShooter.conformToState(Shooter.State.PARTIALRAMP);
                        mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
                    }
                    else{
                        mPivot.conformToState(Pivot.State.INTAKING);
                        mShooter.conformToState(Shooter.State.IDLE);
                    }
                }else{
                    if(inShootZone(timestamp))
                        mPivot.conformToState(Pivot.State.AMP);
                    else
                        mPivot.conformToState(Pivot.State.INTAKING);
                }
                break;
            case OFF:
                mPivot.stop();
                mDrive.stop();
                mIndexer.stop();
                mShooter.stop();
                mIntake.stop();
                break;
            case AUTO:
                if (continuousShoot)
                    prepareShooterSetpoints(timestamp);
                break;

        }
        stateChanged = false;
        modeChanged = false;
    }

    public boolean inShootZone(double timestamp) {
        Pose2d currentPose = mRobotState.getKalmanPose(timestamp);
        if (DriverStation.getAlliance().get().equals(Alliance.Red))
            return currentPose.getTranslation().x() > 8.25;
        return currentPose.getTranslation().x() < 8.25;
    }

    public boolean inAmpZone(double timestamp) {
        return mRobotState.getKalmanPose(timestamp).getTranslation()
                .translateBy(Constants.FieldConstants.getAmpPose().inverse().getTranslation()).norm() < 4;
    }

    public void indicationState() {
        request(new RequestList(Arrays.asList(
                mShooter.atTargetRequest(),
                mLights.setColorRequest(Color.LOCKED)), false));
    }

    public void setPieceAim(boolean disable) {
        if (disable)
            pieceAim = false;
        else
            pieceAim = true;
    }

    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        synchronized (SuperStructure.this) {

            if (!activeRequestsComplete) {
                if (newRequests) {
                    if (activeRequests.isParallel()) {
                        boolean allActivated = true;
                        for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
                                .hasNext();) {
                            Request request = iterator.next();
                            boolean allowed = request.allowed();
                            allActivated &= allowed;
                            if (allowed)
                                request.act();
                        }
                        newRequests = !allActivated;
                    } else {
                        if (activeRequests.isEmpty()) {
                            activeRequestsComplete = true;
                            return;
                        }
                        currentRequest = activeRequests.remove();
                        currentRequest.act();
                        currentRequest.initialize();
                        newRequests = false;
                    }
                }
                if (activeRequests.isParallel()) {
                    boolean done = true;
                    for (Request request : activeRequests.getRequests()) {
                        done &= request.isFinished();
                    }
                    activeRequestsComplete = done;
                } else if (currentRequest.isFinished()) {
                    if (activeRequests.isEmpty()) {
                        activeRequestsComplete = true;
                    } else if (activeRequests.getRequests().get(0).allowed()) {
                        newRequests = true;
                        activeRequestsComplete = false;
                    }
                } else {
                    currentRequest.act();
                }
            } else {
                if (!queuedRequests.isEmpty()) {
                    setActiveRequests(queuedRequests.remove(0));
                }
            }

        }
        processState(timeStamp);
    }

    public void prepareShooterSetpoints(double timestamp) {
        ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp));
        mShooter.conformToState(Shooter.State.SHOOTING);
        mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle-1);
        mShooter.setSpin(Constants.ShooterConstants.SPIN);
    }

    public void prepareShooterSetpoints(double timestamp, boolean manual) {
        ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp), manual,
                !inShootZone(timestamp));
        if(inShootZone(timestamp)){
        mShooter.conformToState(Shooter.State.SHOOTING);
        mShooter.setSpin(ShooterConstants.SPIN);
        }
        else{
        mShooter.setPercent(.6);
        mShooter.setSpin(1.2);
        }
        mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
        mIndexer.setPiece(false);
    }

    public void setContinuousShootState(boolean newContinuousShoot) {
        queue(new Request() {
            @Override
            public void act() {
                continuousShoot = newContinuousShoot;
            }
        });
    }

    public void prepareShooterSetpoints() {
        ShootingParameters shootingParameters = getShootingParams(
                mRobotState.getKalmanPose(Timer.getFPGATimestamp()));
        mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
        mShooter.conformToState(Shooter.State.SHOOTING);
        mShooter.setSpin(ShooterConstants.SPIN);

    }

    public void preparePivotState() {
        queue(new Request() {
            @Override
            public void act() {
                ShootingParameters params = getShootingParams(mRobotState.getLatestKalmanPose());
                mPivot.conformToState(params.compensatedDesiredPivotAngle);
            }

        });
    }

    public Request preparePivotRequest() {
        return new Request() {
            @Override
            public void act() {
                ShootingParameters params = getShootingParams(mRobotState.getLatestKalmanPose());
                mPivot.conformToState(params.compensatedDesiredPivotAngle);
            }
        };
    }

    public ShootingParameters getShootingParams(Pose2d currentPose) {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;
        double kShotTime = Constants.ShooterConstants.kShotTime;

        Pose2d speakerPose = Constants.getSpeakerPivotPose();

        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = ShootingUtils
                .getPivotMap(false);
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityMap = ShootingUtils
                .getVelocityMap(false);

        ShootingParameters shootingParameters = ShootingUtils.getShootingParameters(
                0,
                currentPose,
                speakerPose,
                kShotTime,
                pivotMap,
                velocityMap,
                shotTimeMap,
                mRobotState.getPredictedVelocity(),
                false);// TODO change to measured when it works
        return shootingParameters;
    }


    public ShootingParameters getShootingParams(Pose2d currentPose, boolean manual, boolean lob) {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;
        double kShotTime = Constants.ShooterConstants.kShotTime;

        Pose2d speakerPose = Constants.getSpeakerPivotPose();
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = ShootingUtils
                .getPivotMap(lob);
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityMap = ShootingUtils
                .getVelocityMap(lob);

        ShootingParameters shootingParameters = ShootingUtils.getShootingParameters(
                pivotOffset,
                currentPose,
                speakerPose,
                kShotTime,
                pivotMap,
                velocityMap,
                shotTimeMap,
                mRobotState.getPredictedVelocity(),
                manual);// TODO change to measured when it works
        return shootingParameters;
    }

    public void intakePercent(double percentage) {
        RequestList request = new RequestList(Arrays.asList(
                mIntake.setIntakePercentRequest(percentage)), false);
        request(request);
    }

    public void stopTrajectoryState() {
        Request stopRequest = new Request() {
            @Override
            public void act() {
                mPathStateGenerator.stopTimer();
            }
        };
        queue(stopRequest);
    }

    public void resumeTrajectoryState() {
        Request stopRequest = new Request() {
            @Override
            public void act() {
                mPathStateGenerator.startTimer();
            }
        };
        queue(stopRequest);
    }

    public void setPivotState(double position) {
        queue(mPivot.stateRequest(position));
    }

    public void trajectoryState(PathPlannerTrajectory trajectory, double initRotation) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("trajectory")), true);
        RequestList queue = new RequestList(Arrays.asList(
                mDriveMotionPlanner.setTrajectoryRequest(trajectory, initRotation, true),
                mDrive.setStateRequest(SwerveDrive.State.TRAJECTORY),
                mDriveMotionPlanner.startPathRequest(true)), false);
        queue(request);
        queue(queue);
    }

    public void onTheFlyTrajectoryState(Pose2d endPose, double timestamp) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("OnTheFly"),
                mDriveMotionPlanner.generatePathRequest(
                        mRobotState.getKalmanPose(timestamp),
                        endPose,
                        ChassisSpeeds.fromTwist2d(mRobotState.getPredictedVelocity(), mDrive.getRobotHeading()),
                        false),
                mDrive.setStateRequest(SwerveDrive.State.TRAJECTORY),
                mDriveMotionPlanner.startPathRequest(false)), false);

        request(request);
    }

    public void intakeState() {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("Intaking"),
                mIndexer.setHasPieceRequest(false),
                mLights.setColorRequest(Color.INTAKING),
                mPivot.atTargetRequest(),
                mIntake.stateRequest(Intake.State.INTAKING),
                mIndexer.stateRequest(Indexer.State.RECIEVING),
                mIndexer.hasPieceRequest(false),
                waitRequest(.03),
                mLights.setColorRequest(Color.INTAKED),
                mIndexer.stateRequest(Indexer.State.OFF),
                mIntake.stateRequest(Intake.State.OFF)), false);
        request(request);
    }

    public void intakeState(double timeout) {
        RequestList request = new RequestList(Arrays.asList(
                    logCurrentRequest("Intaking"),
                    mDrive.setModeRequest(TrajectoryMode.TRACKING),
                    mLights.setColorRequest(Color.INTAKING),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(Indexer.State.RECIEVING),
                    mIndexer.hasPieceRequest(timeout),
                    mDrive.setModeRequest(TrajectoryMode.FOLLOWING),
                    mLights.setColorRequest(Color.INTAKED),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIntake.stateRequest(Intake.State.OFF),
                    mIndexer.setHasPieceRequest(true))
                    , false);
        queue(request);

    }
    public void intakeShootState(double timeout) {
        RequestList request;
        request = new RequestList(Arrays.asList(
                    logCurrentRequest("Intaking"),
                    mDrive.setModeRequest(TrajectoryMode.TRACKING),
                    mIndexer.setHasPieceRequest(true),
                    mLights.setColorRequest(Color.INTAKING),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(-1),
                    waitRequest(timeout),
                    mLights.setColorRequest(Color.SHOOTING),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIntake.stateRequest(Intake.State.OFF),
                    mIndexer.setHasPieceRequest(false))
                    , false);
        queue(request);
    }
        public void shooterPercentState(double percent) {
        queue(mShooter.setPercentRequest(percent));
    }
    public void waitForEventState(double timestamp) {
        queue(mDriveMotionPlanner.waitForTrajectoryRequest(timestamp));
    }

    public void setManual(boolean Override) {
        manual = Override;
    }

    public void waitForPositionState(Translation2d other) {
        queue(
                new Request() {
                    @Override
                    public boolean isFinished() {
                        Logger.recordOutput("Auto/Current Event Marker", Pose2d.fromTranslation(other.reflect()).toWPI());
                        double timeStamp = Timer.getFPGATimestamp();
                        if (DriverStation.getAlliance().get().equals(Alliance.Blue))
                            return other.translateBy(
                                    mRobotState.getKalmanPose(timeStamp).getTranslation().inverse())
                                    .norm() < 0.7;
                        return other.reflect().translateBy(
                                mRobotState.getKalmanPose(timeStamp).getTranslation().inverse())
                                .norm() < 0.7;

                    }
                });
    }

    public void waitForMarkerState(ChoreoEventMarker m) {
        queue(
                new Request() {
                    @Override
                    public boolean isFinished() {
                        Translation2d other = m.markerPos;
                        Logger.recordOutput("Auto/Current Event Marker", Pose2d.fromTranslation(other.reflect()).toWPI());
                        double timeStamp = Timer.getFPGATimestamp();
                        if(mPathStateGenerator.getTime() > m.getTimestamp())
                            return false;
                        if (DriverStation.getAlliance().get().equals(Alliance.Blue))
                            return other.translateBy(
                                    mRobotState.getKalmanPose(timeStamp).getTranslation().inverse())
                                    .norm() < 0.7;
                        return other.reflect().translateBy(
                                mRobotState.getKalmanPose(timeStamp).getTranslation().inverse())
                                .norm() < 0.7;

                    }
                });
    }
    public void printState(String string) {
        queue(new Request() {
            @Override
            public void initialize() {
                System.out.println(string);
            }
        });
    }

    public void shootState(boolean Override) {
        if (Override) {
            RequestList queue = new RequestList(Arrays.asList(
                    mShooter.atTargetRequest(),
                    mPivot.atTargetRequest(),
                    mDrive.isAimedRequest(),
                    mLights.setColorRequest(Color.LOCKED),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(Indexer.State.TRANSFERING),
                    mLights.setColorRequest(Color.SHOOTING),
                    mIndexer.hasNoPieceRequest(0.4),
                    mShooter.stateRequest(Shooter.State.IDLE),
                    mIndexer.setHasPieceRequest(false)), false);
            request(queue);
        } else {
            RequestList queue = new RequestList(Arrays.asList(
                    mLights.setColorRequest(Color.AIMING),
                    // mPivot.atTargetRequest(),
                    mLights.setColorRequest(Color.SHOOTING),
                    mIndexer.stateRequest(Indexer.State.TRANSFERING),
                    waitRequest(0.4),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIndexer.setHasPieceRequest(false)), false);
            queue(queue);
        }
    }

    public Request logCurrentRequest(String newLog) {
        return new Request() {
            @Override
            public void act() {
                currentRequestLog = newLog;
            }
        };
    }

    public String getCurrentLoggedRequest() {
        return currentRequestLog;
    }

    public void scoreAmpState() {
        RequestList request = new RequestList(Arrays.asList(
                mPivot.stateRequest(Pivot.State.AMP),
                mPivot.atTargetRequest(),
                mArm.stateRequest(Arm.State.AMP),
                mArm.atTargetRequest(),
                mShooter.stateRequest(Shooter.State.AMP),
                mShooter.atTargetRequest(.4),
                mLights.setColorRequest(Color.AMPRAMPED),
                mIndexer.stateRequest(Indexer.State.TRANSFERING),
                waitRequest(.5),
                mShooter.stateRequest(Shooter.State.IDLE),
                mArm.stateRequest(Arm.State.MAXDOWN)), false);
        request(request);

    }
    public void overrideAmpState() {
        RequestList request = new RequestList(Arrays.asList(
                mPivot.stateRequest(Pivot.State.AMP),
                mArm.stateRequest(Arm.State.AMP),
                mShooter.stateRequest(Shooter.State.AMP),
                mIndexer.stateRequest(Indexer.State.TRANSFERING),
                waitRequest(.5),
                mShooter.stateRequest(Shooter.State.IDLE),
                mArm.stateRequest(Arm.State.MAXDOWN)), false);
        request(request);

    }
    public void offsetPivot(double offset) {
        pivotOffset += offset;
        Logger.recordOutput("Pivot Offset", pivotOffset);
    }

    public void waitState(double waitTime, boolean Override) {
        if (Override)
            request(waitRequest(waitTime));
        else
            queue(waitRequest(waitTime));
    }

    public Request waitRequest(double waitTime) {
        return new Request() {
            Timer timer = new Timer();

            @Override
            public boolean isFinished() {
                if (timer.hasElapsed(waitTime)) {
                    timer.reset();
                    return true;
                } else
                    return timer.hasElapsed(waitTime);
            }

            @Override
            public void act() {
                timer.start();
            }
        };
    }

    public Request eitherRequest(Request request1, Request request2) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return request1.isFinished() || request2.isFinished();
            }
        };
    }

    public Request neverRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {

                return false;
            }

            @Override
            public void act() {
            }
        };
    }

    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Auto/Countinuous Shoot", continuousShoot);
        Logger.recordOutput("SuperStructure/RequestsCompleted", requestsCompleted());
        Logger.recordOutput("SuperStructure/CurrentRequest", currentRequestLog);
    }

    @Override
    public void stop() {

    }
}
