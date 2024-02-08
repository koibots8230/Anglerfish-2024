// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.PlopperConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class PlopperIOSparkMax implements PlopperIO {
    private final CANSparkMax shooterPivotMotor;
    private final AbsoluteEncoder shooterPivotEncoder;

    public PlopperIOSparkMax() {
        shooterPivotMotor = new CANSparkMax(PlopperConstants.MOTOR, MotorType.kBrushless);
        shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPivotEncoder.setPositionConversionFactor(PlopperConstants.ENCODER_POSITION_FACTOR);
        shooterPivotMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(PlopperIOInputs inputs) {
        inputs.position = Rotation2d.fromRadians(shooterPivotEncoder.getPosition());
        inputs.voltage = Volts.of(shooterPivotMotor.getBusVoltage());
        inputs.current = Amps.of(shooterPivotMotor.getOutputCurrent());
        inputs.velocity = RotationsPerSecond.of(shooterPivotEncoder.getVelocity());
    }

    public void setVoltage(double volts) {
        shooterPivotMotor.setVoltage(volts);
    }

    public void setIdleMode(boolean isBrake) {
        shooterPivotMotor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
