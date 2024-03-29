// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Robot;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private Measure<Velocity<Angle>> setpoint = RPM.of(0);

    public Intake() {
        io = Robot.isReal() ? new IntakeIOSparkMax() : new IntakeIOSim();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Intake", inputs);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        io.setVelocity(velocity);
    }
}
