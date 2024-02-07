// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake.IntakePivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakePivotIOSparkMax implements IntakePivotIO {
    private final CANSparkMax intakePivotSparkMax;
    private final DutyCycleEncoder intakePivotEncoder;

    private final boolean isIntakePivotMotorInverted = true;

    public IntakePivotIOSparkMax(int intakePivotId) {
        intakePivotSparkMax = new CANSparkMax(intakePivotId, MotorType.kBrushless);
        intakePivotSparkMax.restoreFactoryDefaults();
        intakePivotSparkMax.setCANTimeout(250);
        intakePivotEncoder = new DutyCycleEncoder(0);

        /* Apply position and velocity conversion factors for the pivoting encoder.
         * This will be in radians and radians per second.
         */
        // intakePivotEncoder.setPositionConversionFactor(IntakeConstants.INTAKE_PIVOT_ENCODER_POSITION_FACTOR);
        // intakePivotEncoder.setVelocityConversionFactor(IntakeConstants.INTAKE_PIVOT_ENCODER_VELOCITY_FACTOR);

        intakePivotEncoder.setPositionOffset(0);

        intakePivotSparkMax.setInverted(isIntakePivotMotorInverted);
        intakePivotSparkMax.setSmartCurrentLimit(20);
        intakePivotSparkMax.enableVoltageCompensation(12.0);

        intakePivotSparkMax.setCANTimeout(0);

        intakePivotSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        inputs.intakePivotPosition =
                Rotation2d.fromRotations(intakePivotEncoder.get() * 2 * Math.PI);
        inputs.intakePivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0);
        inputs.intakePivotAppliedVolts =
                intakePivotSparkMax.getAppliedOutput() * intakePivotSparkMax.getBusVoltage();
        inputs.intakePivotCurrentAmps = new double[] {intakePivotSparkMax.getOutputCurrent()};
    }

    @Override
    public void setIntakePivotVoltage(double volts) {
        intakePivotSparkMax.setVoltage(volts);
    }

    @Override
    public void setIntakePivotBrakeMode(boolean enable) {
        intakePivotSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setZeroOffset() {
        // intakePivotEncoder.setZeroOffset(IntakeConstants.INTAKE_PIVOT_ZERO_OFFSET);
    }

    @Override
    public void setPosition(double position) {
        intakePivotSparkMax.set(position);
    }
}