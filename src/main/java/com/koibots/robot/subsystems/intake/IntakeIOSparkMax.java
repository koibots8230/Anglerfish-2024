// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.*;

public class IntakeIOSparkMax implements IntakeIO {

    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    public IntakeIOSparkMax() {

        intakeMotor = new CANSparkMax(Constants.DeviceIDs.INTAKE, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(40, 60, 5676);

        intakeEncoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocity = RPM.of(intakeEncoder.getVelocity());

        inputs.current = Amps.of(intakeMotor.getOutputCurrent());
        inputs.voltage =
                Volts.of(intakeMotor.getBusVoltage()).times(intakeMotor.getAppliedOutput());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        intakeMotor.setVoltage(volts.in(Volts));
    }
}
