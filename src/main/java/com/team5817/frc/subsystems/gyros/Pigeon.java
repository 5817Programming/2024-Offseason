// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.gyros;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team5817.frc.Constants;
import com.team5817.frc.Ports;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.math.util.Units;



/** 
 * This class is used as an extention of the original pidgeon class
 * we improved on their methods by offsetting the error on get pitch and by inverting getangle in order to better suit our robots needs
 */
public class Pigeon extends Gyro {
    private static Pigeon instance = null;

    public static Pigeon getInstance() {
        if (instance == null)
            instance = new Pigeon();
        return instance;
    }
    Pigeon2  pigeon = new Pigeon2(Ports.PIGEON, "Minivore");

    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

    public Pigeon() {
        try {
            pigeon.getConfigurator().apply(new Pigeon2Configuration());
            pigeon.getConfigurator().setYaw(0.0);
            yaw.setUpdateFrequency(100.0);
            yawVelocity.setUpdateFrequency(100.0);
            pigeon.optimizeBusUtilization();
        } catch (Exception e) {
            System.out.println(e);
        }
        
    }

    public double getPitch() {
        return pigeon.getRoll().getValue() + 1.7;
    }

    public void setAngle(double angle) {
        
        pigeon.setYaw(angle);
    }
    @Override
    public void update(Rotation2d odomUpdate){
        mPeriodicIO.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        if(mPeriodicIO.connected&&!Constants.currentMode.equals(Constants.Mode.SIM))
            mPeriodicIO.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        else
            mPeriodicIO.yawPosition = mPeriodicIO.yawPosition.rotateBy(odomUpdate.inverse());
        mPeriodicIO.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }

    public Twist2d getVelocity(){
        var x = pigeon.getAccelerationX().getValue()*9.80665;
        var y = pigeon.getAccelerationX().getValue()*9.80665;
        var dt = pigeon.getAngularVelocityZWorld().getValue();
        return new Twist2d(x, y, dt);
    }

    @Override
    public double getAngle() {
        return -mPeriodicIO.yawPosition.getDegrees();
    }
    @Override
    public boolean getConnected(){
        return mPeriodicIO.connected;
    }

}
