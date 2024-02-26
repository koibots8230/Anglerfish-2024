// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Ploppervator;

import static com.koibots.robot.subsystems.Subsystems.*;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// Sets both plopper and elevator position

public class SetPloppervatorPosition extends ParallelCommandGroup {

    public SetPloppervatorPosition(PloppervatorPosition position) {
        addCommands(
                new InstantCommand(
                        () ->
                                Elevator.get()
                                        .setPostion(
                                                switch (position) {
                                                    case Resting -> ElevatorConstants.LOAD_POSITION;
                                                    case Shooting -> ElevatorConstants
                                                            .SHOOT_POSITION;
                                                    case Amp -> ElevatorConstants.AMP_POSITION;
                                                    case Climbing -> ElevatorConstants.AMP_POSITION;
                                                })),
                new InstantCommand(
                        () ->
                                PlopperPivot.get()
                                        .setPosition(
                                                switch (position) {
                                                    case Resting -> SetpointConstants
                                                            .PLOPPPER_PIVOT_LOAD_POSITION;
                                                    case Shooting -> SetpointConstants
                                                            .PLOPPER_PIVOT_AMP_POSITION;
                                                    case Amp -> SetpointConstants
                                                            .PLOPPER_PIVOT_AMP_POSITION;
                                                    case Climbing -> SetpointConstants
                                                            .PLOPPPER_PIVOT_LOAD_POSITION;
                                                })),
                new WaitUntilCommand(() -> Elevator.get().atSetpoint()),
                new WaitUntilCommand(() -> PlopperPivot.get().atSetpoint()));

        addRequirements(Elevator.get(), PlopperPivot.get());
    }
}
