// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

    private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);

    private Measure<Voltage> volts = Volts.of(0);

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.velocity = RotationsPerSecond.of(sim.getAngularVelocityRPM());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        sim.setInputVoltage(volts.in(Volts));
        this.volts = volts;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(sim.getAngularVelocityRPM());
    }

    public Measure<Voltage> getVoltage() {
        return volts;
    }
}
