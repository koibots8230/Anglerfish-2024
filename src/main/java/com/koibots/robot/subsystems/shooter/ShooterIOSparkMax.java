// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.DeviceIDs;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

public class ShooterIOSparkMax implements ShooterIO {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    Encoder rightEncoder;
    Encoder leftEncoder;

    protected ShooterIOSparkMax() {
        leftMotor = new CANSparkMax(DeviceIDs.SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);

        rightMotor =
                new CANSparkMax(DeviceIDs.SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(50, 60, 5676);
        rightMotor.setSmartCurrentLimit(50, 60, 5676);

        rightEncoder = new Encoder(1, 2, false, EncodingType.k1X);
        leftEncoder = new Encoder(22, 21, true, EncodingType.k1X);

        rightEncoder.setSamplesToAverage(ControlConstants.ENCODER_SAMPLES_PER_AVERAGE);
        leftEncoder.setSamplesToAverage(ControlConstants.ENCODER_SAMPLES_PER_AVERAGE);

        leftMotor.clearFaults();
        rightMotor.clearFaults();
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocity = leftEncoder.getRate();
        inputs.rightVelocity = rightEncoder.getRate();

        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());

        inputs.leftVoltage =
                Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());
        inputs.rightVoltage =
                Volts.of(rightMotor.getBusVoltage()).times(rightMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        leftMotor.setVoltage(-left.in(Volts));
        rightMotor.setVoltage(right.in(Volts));
    }
}
