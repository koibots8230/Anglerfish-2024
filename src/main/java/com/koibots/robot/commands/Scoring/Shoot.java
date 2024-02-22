// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.Constants.*;
import com.koibots.robot.RobotContainer;
import com.koibots.robot.commands.Ploppervator.SetPloppervatorPosition;
import com.koibots.robot.commands.Shooter.SpinUpShooter;
import com.koibots.robot.commands.Swerve.AutoAlign;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Arrays;

public class Shoot extends SequentialCommandGroup {

    public Shoot() {
        Pose2d nearestPoint = new Pose2d(1000, 1000, new Rotation2d());
        int whichDistance = 0;

        for (int a = 0; a < DriveConstants.SHOOT_DISTANCES_METERS.size(); a++) {
            Pose2d nearestPointOnCircle =
                    new Pose2d(
                            DriveConstants.SPEAKER_POSITION.getX()
                                    + (DriveConstants.SHOOT_DISTANCES_METERS.get(a).in(Meters)
                                            * ((Swerve.get().getEstimatedPose().getX()
                                                            - DriveConstants.SPEAKER_POSITION
                                                                    .getX())
                                                    / Math.sqrt(
                                                            Math.pow(
                                                                            Swerve.get()
                                                                                            .getEstimatedPose()
                                                                                            .getX()
                                                                                    - DriveConstants
                                                                                            .SPEAKER_POSITION
                                                                                            .getX(),
                                                                            2)
                                                                    + Math.pow(
                                                                            Swerve.get()
                                                                                            .getEstimatedPose()
                                                                                            .getY()
                                                                                    - DriveConstants
                                                                                            .SPEAKER_POSITION
                                                                                            .getY(),
                                                                            2)))),
                            DriveConstants.SPEAKER_POSITION.getY()
                                    + (DriveConstants.SHOOT_DISTANCES_METERS.get(a).in(Meters)
                                            * ((Swerve.get().getEstimatedPose().getY()
                                                            - DriveConstants.SPEAKER_POSITION
                                                                    .getY())
                                                    / Math.sqrt(
                                                            Math.pow(
                                                                            Swerve.get()
                                                                                            .getEstimatedPose()
                                                                                            .getX()
                                                                                    - DriveConstants
                                                                                            .SPEAKER_POSITION
                                                                                            .getX(),
                                                                            2)
                                                                    + Math.pow(
                                                                            Swerve.get()
                                                                                            .getEstimatedPose()
                                                                                            .getY()
                                                                                    - DriveConstants
                                                                                            .SPEAKER_POSITION
                                                                                            .getY(),
                                                                            2)))),
                            new Rotation2d());
            nearestPoint =
                    Swerve.get()
                            .getEstimatedPose()
                            .nearest(Arrays.asList(nearestPoint, nearestPointOnCircle));
            whichDistance = (nearestPoint == nearestPointOnCircle) ? a : whichDistance;
        }
        if (Math.abs(Swerve.get().getEstimatedPose().getX() - nearestPoint.getX())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_SHOOT.getX()
                && Math.abs(Swerve.get().getEstimatedPose().getY() - nearestPoint.getY())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_SHOOT.getY()) {
            nearestPoint =
                    new Pose2d(
                            nearestPoint.getX(),
                            nearestPoint.getY(),
                            new Rotation2d(
                                    Math.atan(
                                            (nearestPoint.getX()
                                                                    - DriveConstants
                                                                            .SPEAKER_POSITION
                                                                            .getX())
                                                            / (nearestPoint.getY()
                                                                    - DriveConstants
                                                                            .SPEAKER_POSITION
                                                                            .getY())
                                                    - (1.5 * Math.PI))));

            addCommands(
                    new AutoAlign(nearestPoint, Meters.of(0)),
                    new SetPloppervatorPosition(PloppervatorPosition.Shooting),
                    new SpinUpShooter(SetpointConstants.SHOOTER_SPEEDS.get(whichDistance)),
                    new ParallelRaceGroup(
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(SetpointConstants.INDEXER_SHOOT_SPEED),
                                    Indexer.get()),
                            new WaitCommand(2)),
                    new ParallelCommandGroup(
                            new InstantCommand(
                                    () -> Shooter.get().setVelocity(RPM.of(0)), Shooter.get()),
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
        } else {
            addCommands(
                    new InstantCommand(() -> RobotContainer.rumbleController(0.4)),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> RobotContainer.rumbleController(0.0)));
        }
    }

    public Shoot(Measure<Velocity<Angle>> velocity, boolean doPathing) {
        if (doPathing) {
            addCommands(new Shoot());
        } else {
            addCommands(
                    new SetPloppervatorPosition(PloppervatorPosition.Shooting),
                    new SpinUpShooter(velocity),
                    new ParallelRaceGroup(
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(SetpointConstants.INDEXER_SHOOT_SPEED),
                                    Indexer.get()),
                            new WaitCommand(1)),
                    new ParallelCommandGroup(
                            new InstantCommand(
                                    () -> Shooter.get().setVelocity(RPM.of(0)), Shooter.get()),
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
        }
    }
}
