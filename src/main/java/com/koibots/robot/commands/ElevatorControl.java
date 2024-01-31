package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.Elevator;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.koibots.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

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
