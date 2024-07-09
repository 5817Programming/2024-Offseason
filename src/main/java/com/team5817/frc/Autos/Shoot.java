package com.team5817.frc.Autos;



import com.team5817.frc.subsystems.Shooter;
import com.team5817.frc.subsystems.SuperStructure;
public class Shoot extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();


    @Override
    public void auto() {
        Shooter.getInstance().setPercent(0.8);

        s.setPivotState(0.083-.125);
        s.shootState(false);
    }
}