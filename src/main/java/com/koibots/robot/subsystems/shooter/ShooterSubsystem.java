// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import com.koibots.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem {
    private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private static CANSparkMax shooterMotor1;

    ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1, MotorType.kBrushless);
    }

    public void setSpeed(DoubleSupplier speed) {
        shooterMotor1.set(speed.getAsDouble());
    }

    public ShooterSubsystem get() {
        return shooterSubsystem;
    }

    private final RelativeEncoder encoder1 = shooterMotor1.getEncoder();

    public void getPosition() {
        encoder1.getPosition();
    }
}
