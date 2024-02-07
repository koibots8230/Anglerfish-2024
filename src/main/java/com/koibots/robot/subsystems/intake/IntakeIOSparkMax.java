// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import com.koibots.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;

    public IntakeIOSparkMax() {

        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();

        intakeEncoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocity = intakeEncoder.getVelocity();
        inputs.intakePosition = new Rotation2d(intakeEncoder.getPosition());
        inputs.intakeVoltage = intakeMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double volts) {

        /*
         *   if this is wrong its not my fault :D (its grant's)
         */

        // rpm = Intake.get().intakeTrueTargetRPM(rpm);
        // double volts = Math.max(Math.min(rpm * (12 / 5676), 12.0), -12.0);

        // System.out.println("voltage thingy - " + volts);
        intakeMotor.setVoltage(-volts);
    }
}
