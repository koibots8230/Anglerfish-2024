package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.swerve.Swerve;
import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;

    static {
        Swerve = () -> {
            swerveInstance = new Swerve();
            Swerve = () -> {
                return swerveInstance;
            };

            return swerveInstance;
        };
    }
}