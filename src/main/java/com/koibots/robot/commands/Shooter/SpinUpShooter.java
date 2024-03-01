// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Shooter;

import static com.koibots.robot.subsystems.Subsystems.Shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {

    Measure<Velocity<Angle>> speed;

    public SpinUpShooter(Measure<Velocity<Angle>> speed) {
        this.speed = speed;
        addRequirements(Shooter.get());
    }

    @Override
    public void initialize() {
        Shooter.get().setVelocity(speed);
        ;
    }

    @Override
    public boolean isFinished() {
        return Shooter.get().atSetpoint();
    }
}
