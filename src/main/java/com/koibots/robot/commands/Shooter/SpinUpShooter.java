// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Shooter;

import static com.koibots.robot.subsystems.Subsystems.Shooter;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {

    Measure<Velocity<Angle>> topSpeed;
    Measure<Velocity<Angle>> bottomSpeed;

    public SpinUpShooter(Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        addRequirements(Shooter.get());
    }

    @Override
    public void initialize() {
        Shooter.get().setVelocity(topSpeed, bottomSpeed);
    }

    @Override
    public boolean isFinished() {
        return Shooter.get().atSetpoint();
    }
}
