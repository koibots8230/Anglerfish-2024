// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class PlopperIOSim implements PlopperIO {

    private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);

    private Measure<Voltage> appliedVolts = Volts.of(0);

    @Override
    public void updateInputs(PlopperIOInputs inputs) {
        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.voltage = appliedVolts;
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        sim.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }
}
