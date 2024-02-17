// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PlopperPivotIOSim implements PlopperPivotIO {
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(DCMotor.getNEO(1), 25, 1, 1, 0, Math.PI / 2, true, 0);
    private double voltage = 0;

    @Override
    public void updateInputs(PlopperPivotIOInputs inputs) {
        inputs.position = Radians.of(sim.getAngleRads());
        inputs.voltage = Volts.of(voltage);
        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.velocity = RotationsPerSecond.of(sim.getVelocityRadPerSec());
    }

    @Override
    public void setVoltage(double volts) {
        voltage = volts;
        sim.setInputVoltage(volts);
    }
}
