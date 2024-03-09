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
    CANSparkMax topMotor;
    CANSparkMax bottomMotor;

    Encoder topEncoder;
    Encoder bottomEncoder;

    protected ShooterIOSparkMax() {
        topMotor = new CANSparkMax(DeviceIDs.SHOOTER_TOP, CANSparkLowLevel.MotorType.kBrushless);

        bottomMotor =
                new CANSparkMax(DeviceIDs.SHOOTER_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setSmartCurrentLimit(50, 60, 5676);
        bottomMotor.setSmartCurrentLimit(50, 60, 5676);

        topEncoder = new Encoder(1, 2, false, EncodingType.k1X);
        bottomEncoder = new Encoder(22, 21, true, EncodingType.k1X);

        topEncoder.setSamplesToAverage(ControlConstants.ENCODER_SAMPLES_PER_AVERAGE);
        bottomEncoder.setSamplesToAverage(ControlConstants.ENCODER_SAMPLES_PER_AVERAGE);

        topMotor.clearFaults();
        bottomMotor.clearFaults();
        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topVelocity = topEncoder.getRate();
        inputs.bottomVelocity = bottomEncoder.getRate();

        inputs.topCurrent = Amps.of(topMotor.getOutputCurrent());
        inputs.topCurrent = Amps.of(bottomMotor.getOutputCurrent());

        inputs.topVoltage =
                Volts.of(topMotor.getBusVoltage()).times(topMotor.getAppliedOutput());
        inputs.bottomVoltage =
                Volts.of(bottomMotor.getBusVoltage()).times(bottomMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> top, Measure<Voltage> bottom) {
        topMotor.setVoltage(-top.in(Volts));
        bottomMotor.setVoltage(bottom.in(Volts));
    }
}
