package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;

    private static ShooterPositionSubsystem shooterPositionInstance;
    public static Supplier<ShooterPositionSubsystem> ShooterPositionSubsystem;

    static {
        Swerve = () -> {
            swerveInstance = new Swerve();
            Swerve = () -> swerveInstance;
            return swerveInstance;
        };

        ShooterPositionSubsystem = () -> {
            shooterPositionInstance = new ShooterPositionSubsystem();
            ShooterPositionSubsystem = () -> shooterPositionInstance;
            return shooterPositionInstance;
        };
    }
}