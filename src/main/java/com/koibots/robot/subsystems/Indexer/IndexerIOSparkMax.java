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
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final DigitalInput proximititySwitch;

    public IndexerIOSparkMax() {
        motor = new CANSparkMax(IndexerConstants.MOTOR, MotorType.kBrushless);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(10, 35, 11000);

        encoder = motor.getEncoder();

        proximititySwitch = new DigitalInput(0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity = RPM.of(encoder.getVelocity());
        inputs.isBrake = motor.getIdleMode() == IdleMode.kBrake;
        inputs.voltage = Volts.of(motor.getBusVoltage()).times(motor.getAppliedOutput());
        inputs.current = Amps.of(motor.getOutputCurrent());
    }

    public void setVoltage(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setIdle(boolean isBrake) {
        motor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(encoder.getVelocity());
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage()).times(motor.getAppliedOutput());
    }

    @Override
    public boolean sensorTriggered() {
        return !proximititySwitch.get();
    }
}
