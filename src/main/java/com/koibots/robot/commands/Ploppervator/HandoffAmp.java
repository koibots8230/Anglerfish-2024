// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Ploppervator;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class HandoffAmp extends SequentialCommandGroup {

    public HandoffAmp() {
        addCommands(
                new ConditionalCommand(
                        new InstantCommand(),
                        new SetPloppervatorPosition(PloppervatorPosition.Amp),
                        () ->
                                ((Elevator.get().atSetpoint()
                                                && Elevator.get().getSetpoint()
                                                        == ElevatorConstants.LOAD_POSITION)
                                        && (PlopperPivot.get().atSetpoint()
                                                && PlopperPivot.get().getSetpoint()
                                                        == SetpointConstants
                                                                .PLOPPPER_PIVOT_LOAD_POSITION))),
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () ->
                                        Indexer.get()
                                                .setVelocity(SetpointConstants.INDEXER_SHOOT_SPEED),
                                () -> Indexer.get().setVelocity(RPM.of(0))),
                        new RunPlopper(true)));

        addRequirements(Indexer.get(), Elevator.get(), PlopperPivot.get(), Plopper.get());
    }
}
