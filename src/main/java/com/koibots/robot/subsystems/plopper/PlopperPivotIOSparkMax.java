// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.PlopperPivotConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class PlopperPivotIOSparkMax implements PlopperPivotIO {
    private final CANSparkMax plopperPivotMotor;
    private final AbsoluteEncoder plopperPivotEncoder;

    public PlopperPivotIOSparkMax() {
        plopperPivotMotor = new CANSparkMax(PlopperPivotConstants.MOTOR_PORT, MotorType.kBrushless);
        plopperPivotMotor.setIdleMode(IdleMode.kBrake);
        plopperPivotMotor.setSmartCurrentLimit(10, 50, 5670);

        plopperPivotEncoder = plopperPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        plopperPivotEncoder.setPositionConversionFactor(
                PlopperPivotConstants.ENCODER_POSITION_FACTOR);
    }

    @Override
    public void updateInputs(PlopperPivotIOInputs inputs) {
        inputs.position = Radians.of(plopperPivotEncoder.getPosition());
        inputs.voltage = Volts.of(plopperPivotMotor.getBusVoltage());
        inputs.current = Amps.of(plopperPivotMotor.getOutputCurrent());
        inputs.velocity = RotationsPerSecond.of(plopperPivotEncoder.getVelocity());
    }

    public void setVoltage(double volts) {
        plopperPivotMotor.setVoltage(volts);
    }

    public void setIdleMode(boolean isBrake) {
        plopperPivotMotor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
