 package com.team5817.frc.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc.Constants.ElevatorConstants;
 import com.team5817.frc.subsystems.Requests.Request;
import com.team5817.frc.Constants;
import com.team5817.frc.Ports;


  public class Climb extends Subsystem {
    private ClimbIOAutoLogged mPeriodicIO = new ClimbIOAutoLogged();
    private TalonFX elevatorMotor1 = new TalonFX(Ports.elevatorMotor1,"Minivore");
    private TalonFX elevatorMotor2 = new TalonFX(Ports.elevatorMotor2,"Minivore");
    
    private TalonFXConfiguration elevator1Config = Constants.ElevatorConstants.;//TODO

    private State currentState;



    public static Climb instance = null;

    public static Climb getInstance() {
      if (instance == null)
        instance = new Climb();
      return instance;
    }

    /** Creates a new pivot. */
    public Climb() {
      elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
      elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
      elevatorMotor1.setPosition(0);
      elevatorMotor2.setPosition(0);
      configMotors();
      
    }

    public enum ControlMode {
      MotionMagic,
      Percent,
    }

    public enum State {
      Down(ElevatorConstants.MAX_DOWN),
      UP(ElevatorConstants.MAX_UP);

      double output = 0;

      State(double output){
        this.output = output;
      }
    }

    public void setRamp(double rampTime) {
      elevatorMotor1.getConfigurator().refresh(elevator1Config);
      elevator1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
      elevatorMotor1.getConfigurator().apply(elevator1Config);
    }

    public void configMotors() {
      elevator1Config = ElevatorConstants;
      elevatorMotor1.getConfigurator().apply(elevator1Config);
      elevatorMotor2.getConfigurator().apply(elevator1Config);
      elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
      elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

    }

    public void setMotionMagic(double position){
      mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
      mPeriodicIO.rotationDemand = position;
    }

    public void setPivotPercent(double percentage) {
      mPeriodicIO.rotationDemand = percentage;
    }


    public boolean atTarget(){
      return Math.abs(currentState.output) - Math.abs(mPeriodicIO.rotationPosition) <1;
    }

    public void conformToState(State state) {
      setMotionMagic(state.output);
    }
    public void conformToState(double Override) {
      setMotionMagic(Override);
    }

    public void motionMagic(){
      elevatorMotor1.setControl(new MotionMagicDutyCycle(mPeriodicIO.rotationDemand));
      elevatorMotor2.setControl(new Follower(Ports.elevatorMotor1, true));
    }   

    public void setPercent(){
      elevatorMotor1.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));
      elevatorMotor2.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));

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
              return  atTarget();
          }
      };
    }

    public double getStatorCurrent() {
      return mPeriodicIO.statorCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
      mPeriodicIO.rotationPosition = elevatorMotor1.getPosition().getValueAsDouble();
      mPeriodicIO.velocity = elevatorMotor1.getVelocity().getValueAsDouble();
      mPeriodicIO.statorCurrent = elevatorMotor1.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void readPeriodicInputs() {
      motionMagic();
    }

    @Override
    public void outputTelemetry() {
      Logger.processInputs("Climber", mPeriodicIO);
      

    }

    @Override
    public void stop() {
      setpivotPercentRequest(0);
    }
    @AutoLog
    static class ClimbIO{
      ControlMode rotationControlMode = ControlMode.MotionMagic;
      double rotationPosition = 0;
      double velocity = 0;
      double statorCurrent = 0;

      double rotationDemand = 0;
    }
  }
