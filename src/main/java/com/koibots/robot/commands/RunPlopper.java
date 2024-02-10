// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.Plopper;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.PlopperConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class RunPlopper extends Command {
    private final boolean load;

    public RunPlopper(boolean load) {
        this.load = load;
    }

    @Override
    public void initialize() {
        Plopper.get().setVelocity(load ? PlopperConstants.LOAD_SPEED : PlopperConstants.PLOP_SPEED);
    }

    @Override
    public boolean isFinished() {
        return load ? Plopper.get().sensorTriggered() : !Plopper.get().sensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        Plopper.get().setVelocity(RPM.of(0));
    }
}
