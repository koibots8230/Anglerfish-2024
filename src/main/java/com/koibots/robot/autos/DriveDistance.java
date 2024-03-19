// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {

    private final Pose2d endGoal;
    private final Rotation2d angle;

    public DriveDistance(Pose2d goal) {
        endGoal = goal;

        angle = new Rotation2d(Math.atan(goal.getY()-Swerve.get().getEstimatedPose().getY()/goal.getX()-Swerve.get().getEstimatedPose().getX()));

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        Swerve.get()
                .setModuleStates(
                        new SwerveModuleState[] {
                            new SwerveModuleState(
                                    MetersPerSecond.of(3),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(3),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(3),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(3), Swerve.get().getGyroAngle().minus(angle))
                        });
    }

    @Override
    public boolean isFinished() {
        return ((Swerve.get().getEstimatedPose().getX()
                        <= endGoal.getX() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getX()
                        >= endGoal.getX() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        <= endGoal.getY() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        >= endGoal.getY() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters)));
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.get()
                .setModuleStates(
                        new SwerveModuleState[] {
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                        });
    }
}
