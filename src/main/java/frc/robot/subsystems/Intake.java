// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDConstants;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase {

    @Log
    private final DigitalInput intakeDigitalInput;
    private final CANSparkMax IntakeMotor;
    @Log
    private final SparkPIDController pidController;
    @Log
    private RelativeEncoder encoder;

    public Intake(boolean isReal) {

        IntakeMotor = new CANSparkMax(MotorConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        intakeDigitalInput = new DigitalInput(1);

        pidController = IntakeMotor.getPIDController();

        pidController.setP(PIDConstants.INTAKE_PID_KP);
        pidController.setI(PIDConstants.INTAKE_PID_KI);
        pidController.setD(PIDConstants.INTAKE_PID_KD);
        pidController.setFF(PIDConstants.INTAKE_FEEDFORWARD_FF);
    }

    public void setIntakeVelocity(double intakeMotorRPM) {
        pidController.setReference(intakeMotorRPM, CANSparkBase.ControlType.kVelocity);
    }

    public boolean intakeNoteDetected() {
        return intakeDigitalInput.get();
    }

    @Override
    public void periodic() {
    }
}