// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


/** 
 * This class inherits from "Encoder" see "Encoder.java"
 * We made this class so we can customize the behavior of the code that they wrote for us
 * We also have this class do the setup for all of our encoders automatically
 */
public class InductiveEncoder extends Encoder{
    DutyCycleEncoder encoder;
    private double lowBound = 0.10;
    private double upperBound = 0.90;

    double highestUpperBound = 0;
    double lowestUpperBound = 1;
    public double getHighestBound() {
        return highestUpperBound;
    }
    public double getLowestBound() {
        return lowestUpperBound;
    }

    double offset = 0;
    public InductiveEncoder(int port) {
        encoder = new DutyCycleEncoder(new DigitalInput(port));
        encoder.setDutyCycleRange(lowBound, upperBound);
    }
    public double getOutput() {
        return encoder.getAbsolutePosition();
    }
    public double getBoundedValue() {
        return getOutput();
    }
    public double getAbsolutePosition() {
        return getBoundedValue() * 360.0 - offset;
    }

    public double getRawPosition() {
        return getBoundedValue();
    }
    public double getFrequency() {
        return encoder.getFrequency();
    }
    public boolean isConnected() {
        return encoder.getFrequency() != 0;
    }
}
