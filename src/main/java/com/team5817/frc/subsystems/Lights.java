package com.team5817.frc.subsystems;


import org.littletonrobotics.junction.Logger;

import com.team5817.frc.Ports;
import com.team5817.frc.subsystems.Requests.Request;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
 
public class Lights extends Subsystem{
    Spark lights;
    Lights(){
        lights = new Spark(Ports.Lights);
    }
    public static Lights instance = null;
    public static Lights getInstance() {
        if (instance == null)
          instance = new Lights();
        return instance;
      }
    Color currentColor = Color.IDLE;
    public enum Color{
        INTAKING(.87),
        INTAKED(.87),
        HASPIECE(0.77),
        AIMING(-.11),
        LOCKED(.57),
        SHOOTING(-.99),
        ERROR(0.61),
        IDLE(.93),
        AMP(.69),
        AMPRAMPED(.65);
        
        double output = 0;
        Color(double output){
            this.output = output;
        }
    }
    /*
     * INTAKING,
     * SHOOTING,
     * CLIMB,
     * AMP,
     * SOURCE,
     * OFF,
     * IDLE,
     * AUTO
     * 
     */
    public void update(){
        lights.set(currentColor.output);
    }
    public void setColor(Color color){
        currentColor = color;
    }
    public Request setColorRequest(Color color){
        return new Request() {
            @Override
            public void act() {
                setColor(color);
            }
        };
    }
    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Light Color", currentColor.toString());
    }
    @Override
    public void stop() {
        
    }

}
