// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.koibots.robot.Constants;
import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static CANSparkMax shooterMotor1;
    private static CANSparkMax shooterMotor2;
    RelativeEncoder encoder1;
    RelativeEncoder encoder2;
    PIDController PIDController;

    ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1, MotorType.kBrushless);
        encoder1 = shooterMotor1.getEncoder();
        PIDController = new PIDController(ShooterConstants.kP, 0, 0);///uhm this was autogenrated...

        shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.shooterMotor2, MotorType.kBrushless);
        encoder2 = shooterMotor2.getEncoder();
    }

    public void setSpeed(double speed) {
        shooterMotor1.set(PIDController.calculate(encoder1.getPosition(), speed));
    }

    public void getPosition() {
        encoder1.getPosition();
    }
}
