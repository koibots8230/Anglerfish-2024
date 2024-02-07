// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.IndexerConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private double voltage;

    public IndexerIOSparkMax() {
        motor = new CANSparkMax(IndexerConstants.MOTOR, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        encoder = motor.getEncoder();
        voltage = 0;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity = RevolutionsPerSecond.of(encoder.getVelocity());
        inputs.mode = motor.getIdleMode() == IdleMode.kBrake;
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
        voltage = volts;
    }

    public void setIdle(boolean isBrake) {
        motor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(encoder.getVelocity());
    }

    public double getVoltage() {
        return voltage;
    }
}
