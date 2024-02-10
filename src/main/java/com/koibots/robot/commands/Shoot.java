// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Measure<Velocity<Angle>> velocity) {
        addCommands(
                new SetPlopperPosition(false),
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () -> Indexer.get().setVelocity(IndexerConstants.SHOOT_SPEED),
                                () -> Indexer.get().setVelocity(RPM.of(0))),
                        new StartEndCommand(
                                () -> Shooter.get().setVelocity(velocity),
                                () -> Shooter.get().setVelocity(RPM.of(0)))));

        addRequirements(Indexer.get(), Shooter.get());
    }
}
