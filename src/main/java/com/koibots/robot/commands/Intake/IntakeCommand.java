// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Intake;

import static com.koibots.robot.subsystems.Subsystems.Intake;
import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.Constants.IntakeConstants;
import com.koibots.robot.RobotContainer;
import com.koibots.robot.commands.Swerve.AutoAlign;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Arrays;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand() {
        Pose2d nearestNote =
                Swerve.get()
                        .getEstimatedPose()
                        .nearest(Arrays.asList(new Pose2d())); // TODO: Insert note pose getter here

        if (Math.abs(Swerve.get().getEstimatedPose().getX() - nearestNote.getX())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_NOTE.getX()
                && Math.abs(Swerve.get().getEstimatedPose().getY() - nearestNote.getY())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_NOTE.getY()) {
            addCommands(
                    new AutoAlign(nearestNote, Meters.of(0)),
                    new ParallelRaceGroup(
                            new StartEndCommand(
                                    () -> Intake.get().setVelocity(IntakeConstants.TARGET_VELOCITY),
                                    () -> Intake.get().setVelocity(RPM.of(0))),
                            new RunIndexer()));
        } else {
            addCommands(
                    new InstantCommand(() -> RobotContainer.rumbleController(0.4)),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> RobotContainer.rumbleController(0.0)));
        }
    }

    public IntakeCommand(boolean doPathing) {
        if (doPathing) {
            addCommands(new IntakeCommand());
        } else {
            addCommands(
                    new ParallelRaceGroup(
                            new StartEndCommand(
                                    () -> Intake.get().setVelocity(IntakeConstants.TARGET_VELOCITY),
                                    () -> Intake.get().setVelocity(RPM.of(0)),
                                    Intake.get()),
                            new RunIndexer()));
        }
    }
}
