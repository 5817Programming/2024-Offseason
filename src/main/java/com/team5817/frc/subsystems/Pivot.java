 package com.team5817.frc.subsystems;

  import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc.Constants;
import com.team5817.frc.Ports;
 import com.team5817.frc.subsystems.Requests.Request;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


  public class Pivot extends Subsystem {
    private PivotIOAutoLogged mPeriodicIO = new PivotIOAutoLogged();
    private TalonFX pivotMotor1 = new TalonFX(Ports.Pivot1,"Minivore");
    private TalonFX pivotMotor2 = new TalonFX(Ports.Pivot2,"Minivore");

    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();


    public static Pivot instance = null;

    public static Pivot getInstance() {
      if (instance == null)
        instance = new Pivot();
      return instance;
    }
    Mechanism2d mech = new Mechanism2d(.6, 1.25);
    MechanismRoot2d root = mech.getRoot("Pivot", 0, 0);
    /** Creates a new pivot. */
    MechanismLigament2d joint;
    public Pivot() {
      joint = root.append(new MechanismLigament2d("Shooter", .7,18, 10, new Color8Bit(Color.kAliceBlue)));
      configMotors();
      resetToAbsolute();
    }

    public enum ControlMode {
      MotionMagic,
      Percent,
    }

    public enum State {
      AMP(PivotConstants.AMP),
      TRANSFER(PivotConstants.SPEAKER),
      TRAP(PivotConstants.SPEAKER),
      MAXUP(PivotConstants.MAX_UP),
      MAXDOWN(PivotConstants.MAX_DOWN),
      INTAKING(PivotConstants.INTAKING);

      double output = 0;
      State(double output){
        this.output = output;
      }
    }

    public void resetToAbsolute(){
      double currentAngle =getAbsolutePosition()/360;
      pivotMotor1.setPosition(currentAngle);

      mPeriodicIO.rotationDemand = currentAngle;
    }

    public void setRamp(double rampTime) {
      pivotMotor1.getConfigurator().refresh(pivotConfig);
      pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
      pivotMotor1.getConfigurator().apply(pivotConfig);
    }

    public double getAbsolutePosition(){
      return mPeriodicIO.rotationPosition*360;
    }

    public void configMotors() {
      pivotConfig = Constants.HoodConstants.
      pivotMotor1.getConfigurator().apply(pivotConfig);
      pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
      pivotMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setMotionMagic(double position){
      double degToRot = position/360;
      mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
      mPeriodicIO.rotationDemand = degToRot;
    }

    public void setPivotPercent(double percentage) {
      mPeriodicIO.rotationControlMode = ControlMode.Percent;
      mPeriodicIO.rotationDemand = percentage;
    }

    public boolean atTarget(){
      System.out.println(Math.abs(mPeriodicIO.rotationDemand -mPeriodicIO.rotationPosition));
      return Math.abs(mPeriodicIO.rotationDemand -mPeriodicIO.rotationPosition) <2;
    }

    public void conformToState(State state) {
      setMotionMagic(state.output);
    }
    public void conformToState(double Override) {
      setMotionMagic(Override);
    }

    public void motionMagic(){
      pivotMotor1.setControl(new MotionMagicVoltage(mPeriodicIO.rotationDemand));
      pivotMotor2.setControl(new Follower(Ports.Pivot1, false));

    }

    public void setPercent(){
      pivotMotor1.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));
    }

    public Request stateRequest(State state) {
      return new Request() {

        @Override
        public void act() {
          conformToState(state);
        }
      };
    }

    public Request stateRequest(Double position) {
      return new Request() {

        @Override
        public void act() {
          conformToState(position);
        }
      };
    }

    public Request setpivotPercentRequest(double percentage) {
      return new Request() {


        @Override
        public void act() {
          setPivotPercent(percentage);
        }

      };

    }

    public Request atTargetRequest(){
      return new Request() {
        @Override
          public boolean isFinished() {
              return   atTarget();
          }
      };
    }

    public double getStatorCurrent() {
      return mPeriodicIO.statorCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
      if (Constants.currentMode == Constants.Mode.SIM) {
        mPeriodicIO.rotationPosition = mPeriodicIO.rotationDemand;
      }
      else{
        mPeriodicIO.rotationPosition = pivotMotor1.getPosition().getValueAsDouble();
        mPeriodicIO.statorCurrent = pivotMotor1.getStatorCurrent().getValue();
        mPeriodicIO.velocity = pivotMotor1.getVelocity().getValue();
      }
    }

    @Override
    public void readPeriodicInputs() {
      switch (mPeriodicIO.rotationControlMode) {
        case MotionMagic:
          motionMagic();
          break;
        case Percent:
          setPercent();
          break;
      }
    }

    @Override
    public void outputTelemetry() {
      joint.setAngle(getAbsolutePosition()+23);
      Logger.recordOutput("Pivot/Mechanism", mech);
      Logger.processInputs("Pivot", mPeriodicIO);
    }

    @Override
    public void stop() {
      setpivotPercentRequest(0);
    }
    @AutoLog
    public static class PivotIO {
      ControlMode rotationControlMode = ControlMode.MotionMagic;
      double rotationPosition = 0;
      double velocity = 0;
      double statorCurrent = 0;

      double rotationDemand = -.07;
    }
  }
  
