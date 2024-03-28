// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    public static final double LOOP_PERIOD_SECS = 0.02;

    private FlywheelSim simTop = new FlywheelSim(DCMotor.getNEO(1), 1, .001);
    private FlywheelSim simBottom = new FlywheelSim(DCMotor.getNEO(1), 1, .001);

    private Measure<Voltage> topAppliedVolts = Volts.of(0);
    private Measure<Voltage> bottomAppliedVolts = Volts.of(0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        simTop.update(LOOP_PERIOD_SECS);
        simBottom.update(LOOP_PERIOD_SECS);

        inputs.topVelocity = simTop.getAngularVelocityRadPerSec() * (60.0 / (2 * Math.PI));
        inputs.bottomVelocity = simBottom.getAngularVelocityRadPerSec() * (60.0 / (2 * Math.PI));

        inputs.topVoltage = topAppliedVolts;
        inputs.bottomVoltage = bottomAppliedVolts;

        inputs.topCurrent = Amps.of(simTop.getCurrentDrawAmps());
        inputs.bottomCurrent = Amps.of(simBottom.getCurrentDrawAmps());
    }

    @Override
    public void setVoltages(Measure<Voltage> top, Measure<Voltage> bottom) {
        simTop.setInputVoltage(top.in(Volts));
        simBottom.setInputVoltage(bottom.in(Volts));

        topAppliedVolts = top;
        bottomAppliedVolts = bottom;
    }
}
