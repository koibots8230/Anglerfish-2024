// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.MotorConstants;
import com.koibots.robot.Constants.RobotConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.*;

public class IntakeIOSparkMax implements IntakeIO {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {

        motor = new CANSparkMax(Constants.DeviceIDs.INTAKE, MotorType.kBrushless);

        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(MotorConstants.INTAKE.currentLimit);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

        motor.setInverted(MotorConstants.INTAKE.inverted);

        motor.setIdleMode(MotorConstants.INTAKE.idleMode);

        motor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Milliseconds));

        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocity = RPM.of(encoder.getVelocity());

        inputs.current = Amps.of(motor.getOutputCurrent());
        inputs.voltage = Volts.of(motor.getBusVoltage()).times(motor.getAppliedOutput());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }
}
