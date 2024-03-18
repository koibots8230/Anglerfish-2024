// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {

    private final Measure<Distance> distance;
    private final Rotation2d angle;

    private final Translation2d endGoal;

    public DriveDistance(Measure<Distance> distance, Rotation2d angle) {
        this.distance = distance;
        this.angle = angle;

        endGoal = new Translation2d(distance.in(Meters), angle);

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        Swerve.get()
                .setModuleStates(
                        new SwerveModuleState[] {
                            new SwerveModuleState(
                                    MetersPerSecond.of(2),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(2),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(2),
                                    Swerve.get().getGyroAngle().minus(angle)),
                            new SwerveModuleState(
                                    MetersPerSecond.of(2), Swerve.get().getGyroAngle().minus(angle))
                        });
    }

    @Override
    public boolean isFinished() {
        System.out.println("Goal X: " + endGoal.getX());
        System.out.println("Goal Y: " + endGoal.getY());
        System.out.println("Current X: " + Swerve.get().getEstimatedPose().getX());
        System.out.println("Current Y: " + Swerve.get().getEstimatedPose().getY());
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
