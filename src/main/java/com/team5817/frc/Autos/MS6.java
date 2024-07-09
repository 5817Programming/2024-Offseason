package com.team5817.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.team5817.frc.subsystems.Shooter;
import com.team5817.frc.subsystems.SuperStructure;
import com.team5817.frc.subsystems.SuperStructure.SuperState;
import com.team5817.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MS6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("M6 Staggered");
    
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(.8);
        s.setState(SuperState.AUTO);
        Shooter.getInstance().setPercent(0.8);
        s.setContinuousShootState(false);
        s.setPivotState(60);
        s.waitState(0.2, false);

        s.shootState(false);
        s.setContinuousShootState(true);

        s.trajectoryState(trajectory, initRotation);
        registerChoreoEvents("M6 Staggered");

    }}