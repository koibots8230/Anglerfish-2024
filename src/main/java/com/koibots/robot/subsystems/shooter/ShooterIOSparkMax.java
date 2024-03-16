// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.MotorConstants;
import com.koibots.robot.Constants.RobotConstants;
import com.koibots.robot.Constants.SensorConstants;
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

        topMotor.setSmartCurrentLimit(MotorConstants.TOP_SHOOTER.currentLimit);
        bottomMotor.setSmartCurrentLimit(MotorConstants.BOTTOM_SHOOTER.currentLimit);

        topMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
        bottomMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

        topMotor.setInverted(MotorConstants.TOP_SHOOTER.inverted);
        bottomMotor.setInverted(MotorConstants.BOTTOM_SHOOTER.inverted);

        topMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Milliseconds));
        bottomMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Milliseconds));

        topEncoder =
                new Encoder(
                        DeviceIDs.TOP_SHOOTER_ENCODER[0],
                        DeviceIDs.TOP_SHOOTER_ENCODER[1],
                        false,
                        EncodingType.k1X);
        bottomEncoder =
                new Encoder(
                        DeviceIDs.BOTTOM_SHOOTER_ENCODER[0],
                        DeviceIDs.BOTTOM_SHOOTER_ENCODER[1],
                        true,
                        EncodingType.k1X);

        topEncoder.setSamplesToAverage(SensorConstants.ENCODER_SAMPLES_PER_AVERAGE);
        bottomEncoder.setSamplesToAverage(SensorConstants.ENCODER_SAMPLES_PER_AVERAGE);

        topEncoder.setDistancePerPulse(-60.0 / 2048.0); // TODO: I did a stupid, this was ints :(
        bottomEncoder.setDistancePerPulse(60.0 / 2048.0);

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

        inputs.topVoltage = Volts.of(topMotor.getBusVoltage()).times(topMotor.getAppliedOutput());
        inputs.bottomVoltage =
                Volts.of(bottomMotor.getBusVoltage()).times(bottomMotor.getAppliedOutput());
    }

    @Override
    public void setVoltages(Measure<Voltage> topVoltage, Measure<Voltage> bottomVoltage) {
        // System.out.println(topVoltage.in(Volts) + ", " + bottomVoltage.in(Volts));
        topMotor.setVoltage(-topVoltage.in(Volts));
        bottomMotor.setVoltage(-bottomVoltage.in(Volts));
    }
}
