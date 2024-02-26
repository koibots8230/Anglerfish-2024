// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Shooter;

import static com.koibots.robot.subsystems.Subsystems.Shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {

    Measure<Voltage> volts;

    public SpinUpShooter(Measure<Voltage> volts) {
        this.volts = volts;
        addRequirements(Shooter.get());
    }

    @Override
    public void initialize() {
        Shooter.get().setVoltage(volts);
    }

    @Override
    public boolean isFinished() {
        return Shooter.get().atSetpoint();
    }
}
