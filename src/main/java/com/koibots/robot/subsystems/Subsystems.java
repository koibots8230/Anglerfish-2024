// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.Indexer.Indexer;
import com.koibots.robot.subsystems.LED.LEDs;
import com.koibots.robot.subsystems.elevator.Elevator;
import com.koibots.robot.subsystems.intake.Intake;
import com.koibots.robot.subsystems.plopper.Plopper;
import com.koibots.robot.subsystems.shooter.Shooter;
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

    private static Intake intakeInstance;
    public static Supplier<Intake> Intake =
            () -> {
                intakeInstance = new Intake();
                Intake = () -> intakeInstance;
                return intakeInstance;
            };

    private static Indexer indexerInstance;
    public static Supplier<Indexer> Indexer =
            () -> {
                indexerInstance = new Indexer();
                Indexer = () -> indexerInstance;
                return indexerInstance;
            };

    private static Shooter shooterInstance;
    public static Supplier<Shooter> Shooter =
            () -> {
                shooterInstance = new Shooter();
                Shooter = () -> shooterInstance;
                return shooterInstance;
            };

    private static Elevator elevatorInstance;
    public static Supplier<Elevator> Elevator =
            () -> {
                elevatorInstance = new Elevator();
                Elevator = () -> elevatorInstance;
                return elevatorInstance;
            };

    private static Vision visionInstance;
    public static Supplier<Vision> Vision =
            () -> {
                visionInstance = new Vision();
                Vision = () -> visionInstance;
                return visionInstance;
            };

    private static Plopper plopperInstance;
    public static Supplier<Plopper> Plopper =
            () -> {
                plopperInstance = new Plopper();
                Plopper = () -> plopperInstance;
                return plopperInstance;
            };

    private static LEDs ledsInstance;
    public static Supplier<LEDs> LEDs =
            () -> {
                ledsInstance = new LEDs();
                LEDs = () -> ledsInstance;
                return ledsInstance;
            };
}
