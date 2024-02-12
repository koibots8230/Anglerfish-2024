// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.autoAllign;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.Constants.FieldConstants;
import com.koibots.robot.Constants.IndexerConstants;
import com.koibots.robot.Constants.ShooterConstants;
import com.koibots.robot.subsystems.Subsystems;
import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoAllignToSpeaker extends ParallelCommandGroup {
    Pose2d speaker;
    Pose2d goal;
    PathfindHolonomic pathfinder;
    Translation2d translation;
    Rotation2d rotation;

    public AutoAllignToSpeaker() {
        speaker =
                new Pose2d(
                        new Translation2d(FieldConstants.SPEAKER_X, FieldConstants.SPEAKER_Y),
                        new Rotation2d());
        rotation =
                new Rotation2d(
                        Math.atan(
                                        Subsystems.Swerve.get()
                                                        .getEstimatedPose()
                                                        .relativeTo(speaker)
                                                        .getX()
                                                / Subsystems.Swerve.get()
                                                        .getEstimatedPose()
                                                        .relativeTo(speaker)
                                                        .getY())
                                + Math.PI);
        translation =
                new Translation2d(
                        Meters.of(
                                ShooterConstants.SHOOTING_RANGE
                                        * Math.cos(
                                                Math.atan(
                                                        Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(speaker)
                                                                        .getX()
                                                                / Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(speaker)
                                                                        .getY()))),
                        Meters.of(
                                ShooterConstants.SHOOTING_RANGE
                                        * Math.sin(
                                                Math.atan(
                                                        Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(speaker)
                                                                        .getX()
                                                                / Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(speaker)
                                                                        .getY()))));
        goal = new Pose2d(translation, rotation);
        addCommands(
                new PathfindHolonomic(
                        goal,
                        DriveConstants.CONSTRAINTS,
                        0.0,
                        Subsystems.Swerve.get()::getEstimatedPose,
                        null,
                        null,
                        DriveConstants.PATH_CONFIG,
                        0.0,
                        Subsystems.Swerve.get()),
                new ConditionalCommand(
                        new InstantCommand(
                                () -> Subsystems.Shooter.get().setVelocity(ShooterConstants.SPEED)),
                        null,
                        () ->
                                Math.sqrt(
                                                Math.pow(
                                                                Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(goal)
                                                                        .getX(),
                                                                2)
                                                        + Math.pow(
                                                                Subsystems.Swerve.get()
                                                                        .getEstimatedPose()
                                                                        .relativeTo(goal)
                                                                        .getY(),
                                                                2))
                                        < ShooterConstants.SHOOTING_RANGE),
                new ConditionalCommand(
                        new InstantCommand(
                                () ->
                                        Subsystems.Indexer.get()
                                                .setVelocity(IndexerConstants.SHOOT_SPEED)),
                        null,
                        () ->
                                Subsystems.Shooter.get().getAverageSpeed()
                                        >= ShooterConstants.DOUBLE_SPEED));
    }
}
