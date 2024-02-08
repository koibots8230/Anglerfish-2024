// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

// import com.koibots.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    public static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim intakeMotorSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotorSim.update(LOOP_PERIOD_SECS);

        inputs.intakeVelocity = intakeMotorSim.getAngularVelocityRPM();
        inputs.intakePosition = new Rotation2d(intakeMotorSim.getAngularPositionRotations());
        inputs.intakeVoltage = 0.0;
    }

    @Override
    public void setVoltage(double volts) {
        // intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    }
}
