package com.koibots.robot.subsystems.intake.IntakePivot;

import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.koibots.robot.Constants.IntakeConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakePivotIOSparkMax implements IntakePivotIO {
    private final CANSparkMax intakePivotSparkMax;
    private final AbsoluteEncoder intakePivotEncoder;

    private final boolean isIntakePivotMotorInverted = true;

    public IntakePivotIOSparkMax(int intakePivotId) {
        intakePivotSparkMax = new CANSparkMax(intakePivotId, MotorType.kBrushless);
        intakePivotSparkMax.restoreFactoryDefaults();
        intakePivotSparkMax.setCANTimeout(250);
        intakePivotEncoder = intakePivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        /* Apply position and velocity conversion factors for the pivoting encoder.
         * This will be in radians and radians per second.
        */
        intakePivotEncoder.setPositionConversionFactor(IntakeConstants.INTAKE_PIVOT_ENCODER_POSITION_FACTOR);
        intakePivotEncoder.setVelocityConversionFactor(IntakeConstants.INTAKE_PIVOT_ENCODER_VELOCITY_FACTOR);

        intakePivotSparkMax.setInverted(isIntakePivotMotorInverted);
        intakePivotSparkMax.setSmartCurrentLimit(20);
        intakePivotSparkMax.enableVoltageCompensation(12.0);

        intakePivotSparkMax.setCANTimeout(0);

        intakePivotSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        inputs.intakePivotPosition = Rotation2d.fromRotations(intakePivotEncoder.getPosition());
        inputs.intakePivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(intakePivotEncoder.getVelocity());
        inputs.intakePivotAppliedVolts = intakePivotSparkMax.getAppliedOutput() * intakePivotSparkMax.getBusVoltage();
        inputs.intakePivotCurrentAmps = new double[] { intakePivotSparkMax.getOutputCurrent() };
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
        intakePivotEncoder.setZeroOffset(IntakeConstants.INTAKE_PIVOT_ZERO_OFFSET);
    }

    @Override
    public void setPosition(double position) {
        intakePivotSparkMax.set(position);
    }
}
