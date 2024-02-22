// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;

public class ShooterIOSparkMax implements ShooterIO {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    Encoder rightEncoder;
    Encoder leftEncoder;

    protected ShooterIOSparkMax() {
        leftMotor =
                new CANSparkMax(
                        DeviceIDs.SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);

        rightMotor =
                new CANSparkMax(
                        DeviceIDs.SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        leftMotor.setSmartCurrentLimit(30, 60, 5676);
        rightMotor.setSmartCurrentLimit(30, 60, 5676);

        rightEncoder = new Encoder(2, 3);
        leftEncoder = new Encoder(0, 1);

        leftEncoder.setDistancePerPulse(1 / 8192);
        rightEncoder.setDistancePerPulse(1 / 8192);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocity = RotationsPerSecond.of(leftEncoder.getRate());
        inputs.rightVelocity = RotationsPerSecond.of(rightEncoder.getRate());

        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());

        inputs.leftVoltage =
                Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());
        inputs.rightVoltage =
                Volts.of(rightMotor.getBusVoltage()).times(rightMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        leftMotor.setVoltage(left.in(Volts));
        rightMotor.setVoltage(-right.in(Volts));
        System.out.println("Left: " + left.in(Volts));
        System.out.println("Right: " + right.in(Volts));
    }
}
