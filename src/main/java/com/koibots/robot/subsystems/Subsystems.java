package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.ShooterPivot.ShooterPivot;
import com.koibots.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;

    private static ShooterPivot shooterPivotInstance;
    public static Supplier<ShooterPivot> ShooterPivotSubsystem;

    static {
        Swerve = () -> {
            swerveInstance = new Swerve();
            Swerve = () -> swerveInstance;
            return swerveInstance;
        };

        ShooterPivotSubsystem = () -> {
            shooterPivotInstance = new ShooterPivot();
            ShooterPivotSubsystem = () -> shooterPivotInstance;
            return shooterPivotInstance;
        };
    }
}