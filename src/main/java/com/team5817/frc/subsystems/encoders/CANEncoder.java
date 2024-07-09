// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.encoders;

import com.ctre.phoenix6.hardware.CANcoder;

/** 
 * This class inherits from "Encoder" see "Encoder.java"
 * We made this class so we can customize the behavior of the code that they wrote for us
 */
public class CANEncoder extends Encoder{
    CANcoder encoder;
    public CANEncoder(int id) {
        encoder = new CANcoder(id,"Minivore");
    }
    @Override
    public double getOutput() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }
    @Override
    public boolean isConnected() {
        return encoder.getSupplyVoltage().getValueAsDouble() != 0;
    }
    
}
