// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.Elevator;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ElevatorControl extends Command {

    DoubleSupplier trigger;

    public ElevatorControl(DoubleSupplier trigger) {
        this.trigger = trigger;
        addRequirements(Elevator.get());
    }

    @Override
    public void execute() {
        Elevator.get().setPostion(Meters.of(adjustTrigger(trigger.getAsDouble())));
    }

    private double adjustTrigger(double trigger) {
        return (trigger > .05) ? trigger * Units.inchesToMeters(9) : 0;
    }
}
