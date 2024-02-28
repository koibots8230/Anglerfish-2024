// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.Elevator;
import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Meters;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Constants.SetpointConstants;
import com.koibots.robot.RobotContainer;
import com.koibots.robot.commands.Swerve.AutoAlign;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SequentialCommandGroup {

    public Climb() {
        if (Swerve.get().getEstimatedPose().getY() < 1) {
            addCommands(
                    new AutoAlign(
                            Swerve.get().getEstimatedPose().nearest(DriveConstants.CLIMB_POSITIONS),
                            Meters.of(0)),
                    new InstantCommand(() -> Elevator.get().setPostion(SetpointConstants.ELEVATOR_TOP_HEIGHT)),
                    new InstantCommand(() -> Elevator.get().setPostion(SetpointConstants.ELEVATOR_BOTTOM_HEIGHT))
            );
        } else {
            addCommands(
                    new InstantCommand(() -> RobotContainer.rumbleController(0.4)),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> RobotContainer.rumbleController(0.0)));
        }
    }

    public Climb(boolean doPathing) {
        if (doPathing) {
            addCommands(new Climb());
        } else {
            addCommands(
                    new InstantCommand(() -> Elevator.get().setPostion(SetpointConstants.ELEVATOR_TOP_HEIGHT)),
                    new InstantCommand(() -> Elevator.get().setPostion(SetpointConstants.ELEVATOR_BOTTOM_HEIGHT)));
        }
    }
}
