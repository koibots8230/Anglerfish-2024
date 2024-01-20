// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.swerve.Swerve;
import com.koibots.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve =
            () -> {
                swerveInstance = new Swerve();
                Swerve = () -> swerveInstance;
                return swerveInstance;
            };

    private static Vision visionInstance;
    public static Supplier<Vision> Vision =
            () -> {
                visionInstance = new Vision();
                Vision = () -> visionInstance;
                return visionInstance;
            };
}
