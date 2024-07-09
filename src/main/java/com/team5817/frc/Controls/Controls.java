// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Controls;
import com.team5817.frc.Ports;
import com.team5817.frc.subsystems.Climb;
import com.team5817.frc.subsystems.Indexer;
import com.team5817.frc.subsystems.SuperStructure;
import com.team5817.frc.subsystems.SuperStructure.Mode;
import com.team5817.frc.subsystems.SuperStructure.SuperState;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;


public class Controls {
    protected SuperStructure s;
    protected Controller Driver;
    protected Controller CoDriver;
    protected SwerveDrive swerve;

    private static Controls instance = null;

    private boolean amp = false;

    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls();
        return instance;
    }

    public Controls() {
        Driver = new Controller(Ports.XBOX_1);
        CoDriver = new Controller(Ports.XBOX_2);
        swerve = SwerveDrive.getInstance();
    }

    double percent = 0;
    public void update() {
        Driver.update();
        CoDriver.update();
        s = SuperStructure.getInstance();
        Driver.rumble(Indexer.getInstance().hasPiece());


        s.setPieceAim(Driver.BButton.isActive());

        if(Driver.DpadUp.isPressed())
            s.offsetPivot(.25);
        if(Driver.DpadDown.isPressed())
            s.offsetPivot(-.25);

        // if(Driver.RightBumper.isActive()){
        //     s.intakePercent(-percent);
        // }
        // else{
        //     s.intakePercent(0);
        // }
        s.setManual(CoDriver.BButton.isActive());
        if(Driver.XButton.isPressed())
            amp = true;
        if(Driver.BButton.isPressed())
            amp = false;

        if (Driver.StartButton.isPressed())
            swerve.fieldzeroSwerve();

        if(Driver.AButton.isActive()||CoDriver.AButton.isActive())
            Climb.getInstance().conformToState(Climb.State.UP);
        else 
            Climb.getInstance().conformToState(Climb.State.Down);
        // else if(Driver.BButton.isPressed()){
        //     s.onTheFlyTrajectoryState(new Pose2d(8,2, Rotation2d.fromDegrees(180)), timestamp);

        if(Driver.LeftBumper.isActive())
            if(amp)
                s.setMode(Mode.AMPOVERIDE);
            else
                s.setMode(Mode.FIRING);
        else if(amp)
            s.setMode(Mode.AMP);
        else
            s.setMode(Mode.SHOOTING);
        
        if(Driver.LeftTrigger.value >0.2){
            s.setState(SuperState.OUTTAKING);    

        }
        else if(Driver.RightTrigger.getValue()>0){ 
            s.setState(SuperState.INTAKING);
        }else if(Driver.RightBumper.isActive()){
            s.setState(SuperState.SCORE);
        } 
        else{
            s.setState(SuperState.IDLE);
        }
        swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), Driver.RightStickX.getValue());
    }}


