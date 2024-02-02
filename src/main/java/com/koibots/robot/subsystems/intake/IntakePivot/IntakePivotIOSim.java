// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake.IntakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakePivotIOSim implements IntakePivotIO {
    public static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim intakePivotSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

    private double intakePivotAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        intakePivotSim.update(LOOP_PERIOD_SECS);

        inputs.intakePivotPosition = new Rotation2d(intakePivotSim.getAngularPositionRad());
        inputs.intakePivotVelocityRadPerSec = intakePivotSim.getAngularVelocityRadPerSec();
        inputs.intakePivotAppliedVolts = intakePivotAppliedVolts;
        inputs.intakePivotCurrentAmps =
                new double[] {Math.abs(intakePivotSim.getCurrentDrawAmps())};
    }

    @Override
    public void setIntakePivotVoltage(double volts) {
        intakePivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        intakePivotSim.setInputVoltage(intakePivotAppliedVolts);
    }

    public void setPosition() {}
}
