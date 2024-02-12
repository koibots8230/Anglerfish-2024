// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private static CANSparkMax shooterMotor1;
    private static CANSparkMax shooterMotor2;
    RelativeEncoder encoder1;
    RelativeEncoder encoder2;
    SparkPIDController PIDController1;
    SparkPIDController PIDController2;

    public ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1, MotorType.kBrushless);
        encoder1 = shooterMotor1.getEncoder();
        PIDController1 = shooterMotor1.getPIDController();
        PIDController1.setP(ShooterConstants.kP);

        shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2, MotorType.kBrushless);
        encoder2 = shooterMotor2.getEncoder();
        PIDController2 = shooterMotor2.getPIDController();
        PIDController2.setP(ShooterConstants.kP);
    }

    public void setSpeed(double speed) {
        PIDController1.setReference(speed, CANSparkMax.ControlType.kVelocity);
        PIDController2.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public ShooterSubsystem get() {
        return shooterSubsystem;
    }

    public double getPosition1() {
        return encoder1.getPosition();
    }

    public double getPosition2() {
        return encoder2.getPosition();
    }

    public double getAverageSpeed() {
        return (encoder1.getVelocity() + encoder2.getVelocity()) / 2;
    }
}

