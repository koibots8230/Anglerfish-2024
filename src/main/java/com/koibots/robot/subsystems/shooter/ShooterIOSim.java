// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    private final PIDController topFeedback = new PIDController(
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kP,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kI,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kD
    );
    private final PIDController bottomFeedback = new PIDController(
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kP,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kI,
        ControlConstants.SHOOTER_FEEDBACK_CONSTANTS.kD
    );

    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(
        ControlConstants.SHOOTER_FEEEDFORWARD.ks,
        ControlConstants.SHOOTER_FEEEDFORWARD.kv
    );
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(
        ControlConstants.SHOOTER_FEEEDFORWARD.ks,
        ControlConstants.SHOOTER_FEEEDFORWARD.kv
    );


    public static final double LOOP_PERIOD_SECS = 0.02;

    private final FlywheelSim simTop = new FlywheelSim(DCMotor.getNEO(1), 1, .001);
    private final FlywheelSim simBottom = new FlywheelSim(DCMotor.getNEO(1), 1, .001);

    private Measure<Velocity<Angle>> topSetpoint = RPM.of(0);
    private Measure<Velocity<Angle>> bottomSetpoint = RPM.of(0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        simTop.update(LOOP_PERIOD_SECS);
        simBottom.update(LOOP_PERIOD_SECS);

        inputs.topVelocity = simTop.getAngularVelocityRPM();
        inputs.bottomVelocity = simBottom.getAngularVelocityRPM();

        inputs.topCurrent = Amps.of(simTop.getCurrentDrawAmps());
        inputs.bottomCurrent = Amps.of(simBottom.getCurrentDrawAmps());

        inputs.topVoltage = Volts.of(topFeedback.calculate(simTop.getAngularVelocityRPM(), topSetpoint.in(RPM)) + topFeedforward.calculate(topSetpoint.in(RPM)));
        inputs.bottomVoltage = Volts.of(bottomFeedback.calculate(simBottom.getAngularVelocityRPM(), bottomSetpoint.in(RPM)) + bottomFeedforward.calculate(bottomSetpoint.in(RPM)));

        simTop.setInputVoltage(
            inputs.topVoltage.in(Volts)
        );

        simBottom.setInputVoltage(
            inputs.bottomVoltage.in(Volts)
        );
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> top, Measure<Velocity<Angle>> bottom) {
        topSetpoint = top;
        bottomSetpoint = bottom;
    }
}
