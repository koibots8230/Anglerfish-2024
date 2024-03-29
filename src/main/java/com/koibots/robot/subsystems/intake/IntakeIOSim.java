// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    public static final double LOOP_PERIOD_SECS = 0.02;

    private final PIDController feedback = new PIDController(
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kP,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kI,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kD
    );
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        ControlConstants.SHOOTER_FEEEDFORWARD.ks,
        ControlConstants.SHOOTER_FEEEDFORWARD.kv
    );

    private Measure<Velocity<Angle>> setpoint = RPM.of(0);

    private final DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.update(LOOP_PERIOD_SECS);

        inputs.velocity = sim.getAngularVelocityRPM();

        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.voltage = Volts.of(feedback.calculate(sim.getAngularVelocityRPM(), setpoint.in(RPM)) + feedforward.calculate(setpoint.in(RPM)));

        sim.setInputVoltage(
            inputs.voltage.in(Volts)
        );
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpoint = velocity;
    }
}
