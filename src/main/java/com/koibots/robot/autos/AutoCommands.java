// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.choreo.lib.Choreo;
import com.koibots.robot.Constants;
import com.koibots.robot.Constants.AutoConstants;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Scoring.ScoreAmp;
import com.koibots.robot.commands.Scoring.Shoot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public enum AutoCommands {
    B1_A_N1_S1(
            followChoreoTrajectory("B1_A_N1_S1.1"),
            new ScoreAmp(),
            new ParallelCommandGroup(followChoreoTrajectory("B1_A_N1_S1.2"), new IntakeCommand()),
            new Shoot(Constants.ShooterConstants.SPEED)),
    S1_N4_S2(
            followChoreoTrajectory("S1_N4_S2.1"),
            new IntakeCommand(),
            followChoreoTrajectory("S1_N4_S2.2"),
            new Shoot(Constants.ShooterConstants.SPEED)),
    S2_N5_S2(
            followChoreoTrajectory("S2_N5_S2.1"),
            new IntakeCommand(),
            followChoreoTrajectory("S2_N5_S2.2"),
            new Shoot(Constants.ShooterConstants.SPEED)),
    S2_N6_S2(
            followChoreoTrajectory("S2_N6_S2.1"),
            new IntakeCommand(),
            new Shoot(Constants.ShooterConstants.SPEED),
            followChoreoTrajectory("S2_N6_S2.2"));

    public final Command command;

    AutoCommands(Command... commands) {
        this.command = new SequentialCommandGroup(commands);
    }

    private static Command followChoreoTrajectory(String trajectory) {
        return Choreo.choreoSwerveCommand(
                Choreo.getTrajectory(trajectory),
                Swerve.get()::getEstimatedPose,
                new PIDController(
                        AutoConstants.VX_CONTROLLER.kP,
                        AutoConstants.VX_CONTROLLER.kI,
                        AutoConstants.VX_CONTROLLER.kD),
                new PIDController(
                        AutoConstants.VY_CONTROLLER.kP,
                        AutoConstants.VY_CONTROLLER.kI,
                        AutoConstants.VY_CONTROLLER.kD),
                new PIDController(
                        AutoConstants.VTHETA_CONTROLLER.kP,
                        AutoConstants.VTHETA_CONTROLLER.kI,
                        AutoConstants.VTHETA_CONTROLLER.kD),
                Swerve.get()::driveRobotRelative,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red)
                            .isPresent();
                },
                Swerve.get());
    }
}
