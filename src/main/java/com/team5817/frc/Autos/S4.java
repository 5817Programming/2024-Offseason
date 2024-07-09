package com.team5817.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.team5817.frc.subsystems.Shooter;
import com.team5817.frc.subsystems.SuperStructure;
import com.team5817.frc.subsystems.SuperStructure.SuperState;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class S4 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -60;
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("S4");
    
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(1);
        s.setState(SuperState.AUTO);
        Shooter.getInstance().setPercent(1);
        s.setContinuousShootState(false);
        s.setPivotState(60);
        s.waitState(0.2, false);

        s.shootState(false);
        s.setContinuousShootState(true);

        s.trajectoryState(trajectory, initRotation);
        registerChoreoEvents("S4");

    }}