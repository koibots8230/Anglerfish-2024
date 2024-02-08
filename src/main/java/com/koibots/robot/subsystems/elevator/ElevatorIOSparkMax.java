// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.koibots.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private double appliedVolts;

    private final MechanismLigament2d elevatorMech2d;

    public ElevatorIOSparkMax() {
        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

        encoder = leftMotor.getAlternateEncoder(8192);
        encoder.setPositionConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));
        encoder.setVelocityConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));
        encoder.setPosition(0);

        try (Mechanism2d mechanism2d = new Mechanism2d(Units.inchesToMeters(4), 1)) {
            elevatorMech2d =
                    mechanism2d
                            .getRoot("Elevator Root", 10, 0)
                            .append(new MechanismLigament2d("Elevator", encoder.getPosition(), 90));
        }
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        elevatorMech2d.setLength(encoder.getPosition());

        inputs.position = encoder.getPosition();
        inputs.appliedVoltage = appliedVolts;
        inputs.leftAmperage = leftMotor.getOutputCurrent();
        inputs.rightAmperage = rightMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    @Override
    public void setBrake() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
