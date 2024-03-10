// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.*;
import com.koibots.robot.RobotContainer;
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

        for (int a = 0; a < AlignConstants.SHOOT_DISTANCES_METERS.size(); a++) {
            Pose2d nearestPointOnCircle = // Looks complicated, but is just this https://math.stackexchange.com/a/127615
                    new Pose2d(
                            AlignConstants.SPEAKER_POSITION.getX()
                                    + (AlignConstants.SHOOT_DISTANCES_METERS.get(a).in(Meters)
                                            * ((Swerve.get().getEstimatedPose().getX()
                                                            - AlignConstants.SPEAKER_POSITION.getX())
                                                    / Math.sqrt(
                                                            Math.pow(
                                                                    Swerve.get().getEstimatedPose().getX()
                                                                            - AlignConstants.SPEAKER_POSITION.getX(), 2)
                                                            + Math.pow(
                                                                    Swerve.get().getEstimatedPose().getY()
                                                                            - AlignConstants.SPEAKER_POSITION.getY(), 2)))),
                            AlignConstants.SPEAKER_POSITION.getY()
                                    + (AlignConstants.SHOOT_DISTANCES_METERS.get(a).in(Meters)
                                            * ((Swerve.get().getEstimatedPose().getY()
                                                            - AlignConstants.SPEAKER_POSITION.getY())
                                                    / Math.sqrt(
                                                            Math.pow(
                                                                    Swerve.get().getEstimatedPose().getX()
                                                                            - AlignConstants.SPEAKER_POSITION.getX(), 2)
                                                            + Math.pow(
                                                                    Swerve.get().getEstimatedPose().getY()
                                                                            - AlignConstants.SPEAKER_POSITION.getY(), 2)))),
                            new Rotation2d());

            nearestPoint =
                    Swerve.get()
                            .getEstimatedPose()
                            .nearest(Arrays.asList(nearestPoint, nearestPointOnCircle));
            whichDistance = (nearestPoint == nearestPointOnCircle) ? a : whichDistance;
        }

        if (Math.abs(Swerve.get().getEstimatedPose().getX() - nearestPoint.getX())
                        < AlignConstants.ALLOWED_DISTANCE_FROM_SHOOT.getX()
                && Math.abs(Swerve.get().getEstimatedPose().getY() - nearestPoint.getY())
                        < AlignConstants.ALLOWED_DISTANCE_FROM_SHOOT.getY()) {
            nearestPoint =
                    new Pose2d(
                            nearestPoint.getX(),
                            nearestPoint.getY(),
                            new Rotation2d( // Finds the right angle to point at
                                    Math.atan(
                                            (nearestPoint.getX()
                                                - AlignConstants.SPEAKER_POSITION.getX())
                                        / (nearestPoint.getY()
                                                - AlignConstants.SPEAKER_POSITION.getY())
                                    - (1.5 * Math.PI)))); // Transforms the angle to be in gyro units

            addCommands(
                    new AutoAlign(nearestPoint, Meters.of(0)),
                    new SpinUpShooter(SetpointConstants.SHOOTER_SPEEDS.get(whichDistance).get(0), SetpointConstants.SHOOTER_SPEEDS.get(whichDistance).get(1)),
                    new ParallelRaceGroup(
                            new InstantCommand(
                                    () ->
                                            Indexer.get()
                                                    .setVelocity(
                                                            SetpointConstants.INDEXER_SPEED),
                                    Indexer.get()),
                            new WaitCommand(2)),
                    new ParallelCommandGroup(
                            new InstantCommand(
                                    () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)), Shooter.get()),
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
        } else {
            addCommands(
                    new InstantCommand(() -> RobotContainer.rumbleController(0.4)),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> RobotContainer.rumbleController(0.0)));
        }
    }

    public Shoot(Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed, boolean doPathing) {
        if (doPathing) {
            addCommands(new Shoot());
        } else {
            addCommands(
                    new SpinUpShooter(topSpeed, bottomSpeed),
                    new ParallelRaceGroup(
                            new InstantCommand(
                                    () ->
                                            Indexer.get()
                                                    .setVelocity(
                                                            SetpointConstants.INDEXER_SPEED),
                                    Indexer.get()),
                            new WaitCommand(1)),
                    new ParallelCommandGroup(
                            new InstantCommand(
                                    () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)), Shooter.get()),
                            new InstantCommand(
                                    () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
        }
    }
}
