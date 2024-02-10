// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d elevatorMech2d;

    public ElevatorIOSparkMax() {
        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

        leftMotor.setSmartCurrentLimit(10, 60, 5670);
        rightMotor.setSmartCurrentLimit(10, 60, 5670);

        encoder = leftMotor.getAlternateEncoder(8192);
        encoder.setPositionConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));
        encoder.setVelocityConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));
        encoder.setPosition(0);

        mechanism2d = new Mechanism2d(Units.inchesToMeters(4), Units.inchesToMeters(9));

        elevatorMech2d =
                mechanism2d
                        .getRoot("Elevator Root", 10, 0)
                        .append(new MechanismLigament2d("Elevator", encoder.getPosition(), 90));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        elevatorMech2d.setLength(encoder.getPosition());

        inputs.position = Meters.of(encoder.getPosition());
        inputs.velocity = MetersPerSecond.of(encoder.getVelocity());

        inputs.voltage = Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());
        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        leftMotor.setVoltage(volts.in(Volts));
        rightMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setBrake() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public Measure<Distance> getPosition() {
        return Meters.of(encoder.getPosition());
    }

    @Override
    public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond.of(encoder.getVelocity());
    }

    @Override
    public Mechanism2d getMechanism() {
        return mechanism2d;
    }
}
