// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/** 
  * This class allows us to make our own methods for the encoder in the future
  * currently we havent implemented any new methods but simplifyed the use of the prebuilt methods
 */
public class NewEncoder extends Encoder {
    DutyCycle encoder;
    public NewEncoder(int port) {
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
