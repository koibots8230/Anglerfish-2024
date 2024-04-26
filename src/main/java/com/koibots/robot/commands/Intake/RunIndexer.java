// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Intake;

import static com.koibots.robot.subsystems.Subsystems.Indexer;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexer extends Command {

    public RunIndexer() {
        addRequirements(Indexer.get());
    }

    @Override
    public void initialize() {
        Indexer.get().setVelocity(SetpointConstants.INTAKE_INDEXER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return Indexer.get().sensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.get().setVelocity(RPM.of(0));
    }
}
