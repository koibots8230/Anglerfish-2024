// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.*;

import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Constants.PlopperPivotConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetPlopperPosition extends ParallelCommandGroup {

    public SetPlopperPosition(boolean resting) {
        addCommands(
                new InstantCommand(
                        () ->
                                Elevator.get()
                                        .setPostion(
                                                resting
                                                        ? ElevatorConstants.HANDOFF_POSITION
                                                        : ElevatorConstants.AMP_POSITION)),
                new InstantCommand(
                        () ->
                                PlopperPivot.get()
                                        .setPosition(
                                                resting
                                                        ? PlopperPivotConstants.LOAD_POSITION
                                                        : PlopperPivotConstants.AMP_POSITION)),
                new WaitUntilCommand(() -> Elevator.get().atSetpoint()),
                new WaitUntilCommand(() -> PlopperPivot.get().atSetpoint()));

        addRequirements(Elevator.get(), PlopperPivot.get());
    }
}
