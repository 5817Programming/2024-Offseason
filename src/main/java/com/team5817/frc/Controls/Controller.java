// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.Controls;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Controller {
    XboxController Controller;
    

    public ButtonCheck LeftStickX = new ButtonCheck();
    public ButtonCheck LeftStickY = new ButtonCheck();
    public ButtonCheck RightStickX = new ButtonCheck();
    public ButtonCheck RightStickY = new ButtonCheck();
    public ButtonCheck LeftTrigger = new ButtonCheck(.5);
    public ButtonCheck RightTrigger = new ButtonCheck(.5);
    public ButtonCheck LeftBumper = new ButtonCheck();
    public ButtonCheck RightBumper = new ButtonCheck();
    public ButtonCheck LeftStick = new ButtonCheck();
    public ButtonCheck RightStick = new ButtonCheck();
    public ButtonCheck AButton = new ButtonCheck();
    public ButtonCheck XButton = new ButtonCheck();
    public ButtonCheck YButton = new ButtonCheck();
    public ButtonCheck BButton = new ButtonCheck();
    public ButtonCheck RightStickDown = new ButtonCheck();
    public ButtonCheck StartButton = new ButtonCheck();
    public ButtonCheck DpadLeft = new ButtonCheck();
    public ButtonCheck DpadUp = new ButtonCheck();
    public ButtonCheck DpadRight = new ButtonCheck();
    public ButtonCheck DpadDown = new ButtonCheck();
    public ButtonCheck BackButtonDown = new ButtonCheck();


    private boolean lastRumble = false;
    private Timer rumbleTimer = new Timer();
    private double rumbleStart = Double.NEGATIVE_INFINITY;
    public Controller(int port) {
        Controller = new XboxController(port);
        rumbleTimer.start();

    }

    public void rumble(boolean rumble){
        if((rumble != lastRumble) && (rumble == true)){
            rumbleStart = rumbleTimer.get();
        }
        if(rumbleTimer.get() - rumbleStart < 1)
            this.Controller.setRumble(RumbleType.kBothRumble, 1);
        else
            this.Controller.setRumble(RumbleType.kBothRumble, 0);
        lastRumble = rumble;
    }

    public void update() {

        LeftStickX.update(Controller.getLeftX());
        LeftStickY.update(Controller.getLeftY());
        RightStickX.update(Controller.getRightX());
        RightStickY.update(Controller.getRightY());
        LeftTrigger.update(Controller.getLeftTriggerAxis());
        RightTrigger.update(Controller.getRightTriggerAxis());
        LeftBumper.update(Controller.getLeftBumper());
        RightBumper.update(Controller.getRightBumper());
        LeftStick.update(Controller.getLeftStickButton());
        RightStick.update(Controller.getRightStickButton());
        AButton.update(Controller.getAButton());
        XButton.update(Controller.getXButton());
        YButton.update(Controller.getYButton());
        BButton.update(Controller.getBButton());
        RightStickDown.update(Controller.getRightStickButton());
        StartButton.update(Controller.getStartButton());
        DpadLeft.update(Controller.getPOV() == 270);
        DpadUp.update(Controller.getPOV() == 0);
        DpadRight.update(Controller.getPOV() == 90);
        DpadDown.update(Controller.getPOV() == 180);

    }

    public class ButtonCheck {
        double value;
        double threshold;
        Boolean[] input = new Boolean[2];

        public ButtonCheck(double threshold) {
            this.threshold = threshold;
            for (int i = 0; i < 1; i++) {
                input[i] = false;
            }
        }

        public ButtonCheck() {
            this.threshold = 0;
            for (int i = 0; i < 1; i++) {
                input[i] = false;
            }
        }

        public void update(double input) {
            value = input;
        }

        public void update(boolean input) {
            this.input[1] = this.input[0];
            if (input)
                this.input[0] = true;
            else
                this.input[0] = false;
        }

        public boolean isPressed() {
            return input[0] && !input[1];
        }

        public double getValue() {
            if(Math.abs(value) < threshold)
                return 0;
            return value;
        }

        public boolean isReleased() {
            return !input[0] && input[1];
        }

        public boolean isActive() {
            return input[0];
        }

        public void setThreshhold(double threshold) {
            this.threshold = threshold;
        }
    }
}
