// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.ShooterPivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;

public class ShooterPivotIOSim implements ShooterPivotIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 25, 0, 0, 0, Math.PI * 1/2, true, 0);
    private double voltage = 0;

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
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
