// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Ploppervator;

import static com.koibots.robot.subsystems.Subsystems.Plopper;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class RunPlopper extends Command {
    private final boolean load;
    private int cyclesToEnd;

    public RunPlopper(boolean load) {
        this.load = load;
        cyclesToEnd = 12;

        addRequirements(Plopper.get());
    }

    @Override
    public void initialize() {
        Plopper.get()
                .setVelocity(
                        load
                                ? SetpointConstants.PLOPPER_LOAD_SPEED
                                : SetpointConstants.PLOPPER_PLOP_SPEED);
    }

    @Override
    public void execute() {
        cyclesToEnd =
                (load && Plopper.get().sensorTriggered())
                                || (!load && !Plopper.get().sensorTriggered())
                        ? cyclesToEnd - 1
                        : cyclesToEnd;
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
