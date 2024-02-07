package com.koibots.robot.subsystems.intake.Intake;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.koibots.robot.Constants.IntakeConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;

    public IntakeIOSparkMax() {

        intakeMotor = new CANSparkMax(
            IntakeConstants.INTAKE_MOTOR_PORT,
            MotorType.kBrushless
        );

        intakeMotor.restoreFactoryDefaults();
        
        intakeEncoder = intakeMotor.getEncoder();

    }


    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeVelocity = intakeEncoder.getVelocity();
        inputs.intakePosition = new Rotation2d(intakeEncoder.getPosition());
        inputs.intakeVoltage = intakeMotor.getAppliedOutput();
    }


    @Override
    public void setVoltage(double volts) {

        /*
        *   if this is wrong its not my fault :D (its grant's)
        */

        //rpm = Intake.get().intakeTrueTargetRPM(rpm);
        //double volts = Math.max(Math.min(rpm * (12 / 5676), 12.0), -12.0); 

        //System.out.println("voltage thingy - " + volts);
        intakeMotor.setVoltage(-volts);

    }

}
