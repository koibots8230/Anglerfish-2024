// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PlopperIOSim implements PlopperIO {
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(DCMotor.getNEO(1), 25, 1, 1, 0, Math.PI / 2, true, 0);
    private double voltage = 0;

    @Override
    public void updateInputs(PlopperIOInputs inputs) {
        inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.voltage = Volts.of(voltage);
        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.velocity = RotationsPerSecond.of(sim.getVelocityRadPerSec());
    }

    @Override
    public void setVoltage(double volts) {
        voltage = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setIdleMode(boolean isBrake) {
        // TODO Auto-generated method stub
    }
}
