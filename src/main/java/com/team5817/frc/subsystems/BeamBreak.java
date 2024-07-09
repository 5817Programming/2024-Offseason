package com.team5817.frc.subsystems;


import edu.wpi.first.wpilibj.AnalogInput;


public class BeamBreak {
    AnalogInput sensors;
    BeamBreak(int port){
        sensors = new AnalogInput(port);
    }

    public boolean get(){
        return sensors.getValue()>1000;
    }

    


}
