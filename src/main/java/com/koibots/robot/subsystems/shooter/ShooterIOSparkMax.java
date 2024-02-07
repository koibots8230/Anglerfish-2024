// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class ShooterIOSparkMax implements ShooterIO {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    protected ShooterIOSparkMax() {
        leftMotor =
                new CANSparkMax(
                        ShooterConstants.shooterMotor1, CANSparkLowLevel.MotorType.kBrushless);
        rightMotor =
                new CANSparkMax(
                        ShooterConstants.shooterMotor2, CANSparkLowLevel.MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightMotorCurrent = Amps.of(rightMotor.getOutputCurrent());

        inputs.leftMotorAppliedVoltage =
                Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());
        inputs.rightMotorAppliedVoltage =
                Volts.of(rightMotor.getBusVoltage()).times(rightMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        leftMotor.setVoltage(left.in(Volts));
        rightMotor.setVoltage(right.in(Volts));
    }
}
