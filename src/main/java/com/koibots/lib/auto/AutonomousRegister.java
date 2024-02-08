// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.auto;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.dashboard.Alert;
import com.koibots.lib.util.PIDConstantsIO;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousRegister {
    private static final SendableChooser<Command> autoChooser;
    private static final Alert routineSelectedAlert =
            new Alert("No Autonomous Routine Selected", Alert.AlertType.ERROR);

    static {
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        AutoBuilder.configureHolonomic(
                Swerve.get()::getEstimatedPose,
                Swerve.get()::resetOdometry,
                Swerve.get()::getRelativeSpeeds,
                Swerve.get()::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstantsIO(0, 0, 0, 0, 0, 0),
                        new PIDConstantsIO(0, 0, 0, 0, 0, 0),
                        4.5,
                        0.4,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                Swerve.get());
    }

    public static void registerAutoRoutine(String name, Command auto) {
        autoChooser.addOption(name, auto);
    }

    public static void startSelectedAuto() {
        if (autoChooser.getSelected() != null) {
            autoChooser.getSelected().schedule();
            routineSelectedAlert.set(false);
        } else {
            routineSelectedAlert.set(true);
        }
    }
}
