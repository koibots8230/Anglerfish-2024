// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.intake.Intake.Intake;
import com.koibots.robot.subsystems.intake.IntakePivot.IntakePivot;
import com.koibots.robot.subsystems.swerve.Swerve;
import com.koibots.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;
    public static Intake intakeInstance;
    public static Supplier<Intake> Intake;

    static {
        Swerve = () -> {
            swerveInstance = new Swerve();
            Swerve = () -> {
                return swerveInstance;
            };

            return swerveInstance;
        };

        Intake  = () -> {
            intakeInstance = new Intake();
            Intake = () -> intakeInstance;
            return intakeInstance;
        };
    }
}
