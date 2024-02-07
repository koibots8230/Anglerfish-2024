// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 25, 0);
    private double volts;

    public IndexerIOSim() {
        volts = 0;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity = RotationsPerSecond.of(sim.getAngularVelocityRPM());
    }

    @Override
    public void setIdle(boolean isBrake) {
        // TODO Auto-generated method stub
    }

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
        volts = voltage;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(sim.getAngularVelocityRPM());
    }

    public double getVoltage() {
        return volts;
    }
}
