package com.team5817.frc.subsystems;



import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
 import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team5817.frc.Ports;
import com.team5817.frc.subsystems.Requests.Request;

import edu.wpi.first.wpilibj.Timer;

 public class Indexer extends Subsystem {

   private IndexerIOAutoLogged mPeriodicIO = new IndexerIOAutoLogged();
   private TalonFX indexer = new TalonFX(Ports.INDEXER);
   private boolean lastBeam = false;
   private BeamBreak beamBreak;
   private TalonFXConfiguration indexerConfig;


   public static Indexer instance = null;

   
   public static Indexer getInstance() { 
     if (instance == null)
       instance = new Indexer();
     return instance;
   }

   /** Creates a new intake. */
   public Indexer() {
     configMotors();
     beamBreak = new BeamBreak(0);
   }
   public enum State{
    OFF(0),
    RECIEVING(-.25),
    TRANSFERING(-1),
    REVERSE_TRANSFER(-.5), 
    OUTTAKING(0.4);

    double output = 0;
    State(double output){
        this.output = output;
    }

   }

   public void setRamp(double rampTime) {
     indexer.getConfigurator().refresh(indexerConfig);
     indexerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
     indexer.getConfigurator().apply(indexerConfig);
   }

   public void configMotors() {
     indexerConfig = TalonConfigs.indexerConfigs();
     indexer.getConfigurator().apply(indexerConfig);
   }

   public void setPercent(double Percentage) {
     mPeriodicIO.driveDemand = Percentage;
   }


   public Request stateRequest(State state){
    return new Request() {
        @Override
        public void act(){
            conformToState(state);
        }
    };
   }
   public void conformToState(State state){
    mPeriodicIO.driveDemand = state.output;
   }


  public Request hasPieceRequest(boolean timeout) {
    if (!timeout) {
      return new Request() {
        @Override
        public boolean isFinished() {
          return mPeriodicIO.hasPiece;
        }
      };
    }
    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > .7;
      }
    };
  }
  public Request hasNoPieceRequest(boolean timeout) {
    if (!timeout) {
      return new Request() {
        @Override
        public boolean isFinished() {
          return !mPeriodicIO.hasPiece;
        }
      };
    }
    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return !mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > .9;
      }
    };
  }
 public Request hasNoPieceRequest(double timeout) {

    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return !mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > timeout;
      }
    };
  }
  public Request hasPieceRequest(double timeout) {
    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > timeout;
      }
    };
  }
 public Request hasnoPieceRequest(double timeout) {
    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return !mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > timeout;
      }
    };
  }
   public Request stateRequest(double percentage) {
     return new Request() {

       @Override
       public void act() {

         setPercent(percentage);
       }

     };

   }

   public boolean hasPiece(){
     return mPeriodicIO.hasPiece;
   }
   


   public double getStatorCurrent() {
     return mPeriodicIO.statorCurrent;
   }
   public Request setHasPieceRequest(boolean hasPiece){
    return new Request() {
      @Override
      public void act() {
            mPeriodicIO.hasPiece = hasPiece;
      }
    };
  }

  public void setPiece(Boolean piece){
    mPeriodicIO.hasPiece = piece;
  }
   double noteEntryTime = 0;
   boolean noteHalfway = false;
   @Override
   public void writePeriodicOutputs() {

     mPeriodicIO.drivePosition = indexer.getPosition().getValueAsDouble();
     mPeriodicIO.velocity = indexer.getVelocity().getValueAsDouble();
     mPeriodicIO.statorCurrent = indexer.getStatorCurrent().getValueAsDouble();
     mPeriodicIO.rawBeamBreak = beamBreak.get();

    if(lastBeam != beamBreak.get()&& lastBeam==true)
      mPeriodicIO.hasPiece = !mPeriodicIO.hasPiece;
    lastBeam = beamBreak.get();
  }

   @Override
   public void readPeriodicInputs() {
     indexer.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
   }

   @Override
   public void outputTelemetry() {
    Logger.processInputs("Indexer", mPeriodicIO);

   }

   @Override
   public void stop() {
     stateRequest(0);
   }
   @AutoLog
   public static class IndexerIO{
     double drivePosition = 0;
     double velocity = 0;
     double statorCurrent = 0;
     boolean rawBeamBreak = false;
     boolean hasPiece = true;
     double rotationDemand = 0.0;
     double driveDemand = 0.0;
   }
 }
