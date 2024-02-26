// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    public static final double LOOP_PERIOD_SECS = 0.02;

    private FlywheelSim simLeft = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    private FlywheelSim simRight = new FlywheelSim(DCMotor.getNEO(1), 1, 1);

    private Measure<Voltage> leftAppliedVolts = Volts.of(0);
    private Measure<Voltage> rightAppliedVolts = Volts.of(0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        simLeft.update(LOOP_PERIOD_SECS);
        simLeft.update(LOOP_PERIOD_SECS);

        inputs.leftVelocity = simLeft.getAngularVelocityRadPerSec() * (60 / (2 * Math.PI));
        inputs.rightVelocity = simRight.getAngularVelocityRadPerSec() * (60 / (2 * Math.PI));

        inputs.leftVoltage = leftAppliedVolts;
        inputs.rightVoltage = rightAppliedVolts;

        inputs.leftCurrent = Amps.of(simLeft.getCurrentDrawAmps());
        inputs.rightCurrent = Amps.of(simRight.getCurrentDrawAmps());
    }

    @Override
    public void setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        simLeft.setInputVoltage(left.in(Volts));
        simRight.setInputVoltage(right.in(Volts));

        leftAppliedVolts = left;
        rightAppliedVolts = right;
    }
}
