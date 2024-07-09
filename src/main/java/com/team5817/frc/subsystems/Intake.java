 package com.team5817.frc.subsystems;
 


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.DutyCycleOut;
  import com.ctre.phoenix6.hardware.TalonFX;
 import com.team5817.frc.Ports;
 import com.team5817.frc.subsystems.Requests.Request;
 import com.team5817.lib.TalonConfigs;


  public class Intake extends Subsystem {
    private IntakeIOAutoLogged mPeriodicIO = new IntakeIOAutoLogged();
    private TalonFX intakeMotor = new TalonFX(Ports.Intake, "Minivore");
    private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();


    public static Intake instance = null;

    public static Intake getInstance() { 
      if (instance == null)
        instance = new Intake();
      return instance;
    }

    /** Creates a new intake. */
    public Intake() {
      configMotors();

    }



    public enum State {
      INTAKING(-1),
      OUTTAKING(0.7),
      OFF(0), 
      PARTIALRAMP(-0);

      double output = 0;

      State(double output) {
        this.output = output;
      }
    }

    public void setRamp(double rampTime) {
      intakeMotor.getConfigurator().refresh(intakeConfig);
      intakeConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
      intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void configMotors() {
      intakeConfig = TalonConfigs.intakeConfigs();
      intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void intakePercent(double Percentage) {
      mPeriodicIO.driveDemand = Percentage;
    }


    public void conformToState(State state) {
      intakePercent(state.output);
    }


    public Request stateRequest(State state) {
      return new Request() {

        @Override
        public void act() {
          conformToState(state);
        }
      };
    }

    public Request setIntakePercentRequest(double percentage) {
      return new Request() {

        @Override
        public void act() {

          intakePercent(percentage);
        }

      };

    }


    public double getStatorCurrent() {
      return mPeriodicIO.statorCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
      mPeriodicIO.drivePosition = intakeMotor.getPosition().getValueAsDouble();
      mPeriodicIO.velocity = intakeMotor.getVelocity().getValueAsDouble();
      mPeriodicIO.statorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void readPeriodicInputs() {
      intakeMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
    }

    @Override
    public void outputTelemetry() {
     Logger.processInputs("Intake", mPeriodicIO);


    }

    @Override
    public void stop() {
      setIntakePercentRequest(0);
    }
    @AutoLog
    public static class IntakeIO { 
      double drivePosition = 0;
      double velocity = 0;
      double statorCurrent = 0;
      double driveDemand = 0.0;
    }
  }
