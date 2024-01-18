package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve = () -> {
        swerveInstance = new Swerve();
        Swerve = () -> swerveInstance;
        return swerveInstance;
    };
}