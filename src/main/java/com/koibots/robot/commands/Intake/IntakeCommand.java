// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Intake;

import static com.koibots.robot.subsystems.Subsystems.Intake;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand() {
        addCommands(
                // TODO: Path to note command here
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () -> Intake.get().setVelocity(IntakeConstants.TARGET_VELOCITY),
                                () -> Intake.get().setVelocity(RPM.of(0))),
                        new RunIndexer()));
    }
}
