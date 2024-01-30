// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.ShooterPivot;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ShooterPivotConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {
    private final CANSparkMax shooterPivotMotor;
    private final AbsoluteEncoder shooterPivotEncoder;

    public ShooterPivotIOSparkMax() {
        shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.MOTOR, MotorType.kBrushless);
        shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPivotEncoder.setPositionConversionFactor(
                ShooterPivotConstants.ENCODER_POSITION_FACTOR);
        shooterPivotMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.position = Rotation2d.fromRadians(shooterPivotEncoder.getPosition());
        inputs.voltage = Volts.of(shooterPivotMotor.getBusVoltage());
        inputs.current = Amps.of(shooterPivotMotor.getOutputCurrent());
        inputs.velocity = RadiansPerSecond.of(shooterPivotEncoder.getVelocity());
    }

    public void setMotorSpeed(double desiredPosition) {
        shooterPivotMotor.set(desiredPosition);
    }

    public void setIdleMode(boolean isBrake) {
        shooterPivotMotor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
