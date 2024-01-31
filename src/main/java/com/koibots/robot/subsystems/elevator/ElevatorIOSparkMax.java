package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.koibots.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder encoder;

    private double appliedVolts;

    public ElevatorIOSparkMax() {
        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

        encoder = leftMotor.getAlternateEncoder(8192);

        encoder.setPositionConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));
        encoder.setVelocityConversionFactor(ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters));

        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
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
