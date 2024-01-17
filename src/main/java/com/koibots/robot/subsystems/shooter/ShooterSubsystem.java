package com.koibots.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem {
    private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private static CANSparkMax shooterMotor1;

    ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.IN_MOTOR_1_PORT, MotorType.kBrushless); //help Motortype is deprecated :(
    }

    public void setSpeed(DoubleSupplier speed) {
        shooterMotor1.set(speed.getAsDouble());
    }

    public ShooterSubsystem get() {
        return shooterSubsystem;
    }
}
