// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.DeviceIDs;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class ShooterIOSparkMax implements ShooterIO {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder;

    protected ShooterIOSparkMax() {
        leftMotor = new CANSparkMax(
                 DeviceIDs.SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);

        rightMotor = new CANSparkMax(
                DeviceIDs.SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(40, 60, 5676);
        rightMotor.setSmartCurrentLimit(40, 60, 5676);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        leftMotor.clearFaults();
        rightMotor.clearFaults();
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocity = leftEncoder.getVelocity();
        inputs.rightVelocity = rightEncoder.getVelocity();

        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());

        inputs.leftVoltage = Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());
        inputs.rightVoltage = Volts.of(rightMotor.getBusVoltage()).times(rightMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        leftMotor.setVoltage(-left.in(Volts));
        rightMotor.setVoltage(right.in(Volts));
    }
}
