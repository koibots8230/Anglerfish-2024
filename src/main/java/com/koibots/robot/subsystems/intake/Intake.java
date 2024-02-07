// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import com.koibots.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePidController;

    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);

        intakePidController = intakeMotor.getPIDController();
        intakePidController.setP(IntakeConstants.INTAKE_PID_P);
    }

    @Override
    public void periodic() {}

    public void setIntakeMotor(double targetRPM) {

        double robotSpeed = 0;

        double intakeWheelCircumference = 2 * Math.PI * IntakeConstants.INTAKE_WHEEL_RADIUS;
        double targetDistancePerMinute = targetRPM * intakeWheelCircumference;
        double trueDistancePerMinute = targetDistancePerMinute - robotSpeed;
        double trueRPM = trueDistancePerMinute / intakeWheelCircumference;

        intakePidController.setReference(
                Math.max(trueRPM, IntakeConstants.INTAKE_MINIMUM_RPM),
                CANSparkMax.ControlType.kVelocity);
    }
}
