// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/** 
 * This class inherits from "Encoder" see "Encoder.java"
 * We made this class so we can customize the behavior of the code that they wrote for us
 */public class MagEncoder extends Encoder {
    DutyCycle encoder;
    public MagEncoder(int port) {
        encoder = new DutyCycle(new DigitalInput(port));
    }
    @Override
    public double getOutput() {
        return encoder.getOutput();
    }
    @Override
    public boolean isConnected() {
        return encoder.getFrequency() != 0;
    }
}
