// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.PlopperConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class PlopperIOSparkMax implements PlopperIO {

    private final CANSparkMax plopperMotor;
    private final RelativeEncoder plopperEncoder;

    private final DigitalInput noteSwitch;

    public PlopperIOSparkMax() {
        plopperMotor = new CANSparkMax(PlopperConstants.MOTOR_PORT, MotorType.kBrushless);
        plopperMotor.setIdleMode(IdleMode.kBrake);
        plopperMotor.setSmartCurrentLimit(10, 35, 11000);

        plopperEncoder = plopperMotor.getEncoder();

        noteSwitch = new DigitalInput(PlopperConstants.SWITCH_PORT);
    }

    @Override
    public void updateInputs(PlopperIOInputs inputs) {
        inputs.velocity = RotationsPerSecond.of(plopperEncoder.getVelocity() * 60);
        inputs.current = Amps.of(plopperMotor.getOutputCurrent());
        inputs.voltage =
                Volts.of(plopperMotor.getBusVoltage()).times(plopperMotor.getAppliedOutput());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        plopperMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public boolean sensorTriggered() {
        return noteSwitch.get();
    }
}
