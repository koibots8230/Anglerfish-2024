// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import com.koibots.robot.Constants.AutoConstants;
import com.koibots.robot.Constants.SetpointConstants;
import com.koibots.robot.Robot;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Scoring.Shoot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoActions {

    public static Command SP() {
        return new Shoot(
                SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed,
                false);
    }

    public static Command LS() {
        return new DriveDistance(true);
    }

    public static Command PL() {
        if (Robot.isReal()) {
            return new ParallelCommandGroup(new DriveDistance(0), new IntakeCommand(false));
        } else {
            return new DriveDistance(0);
        }
    }

    public static Command PC() {
        if (Robot.isReal()) {
            return new ParallelCommandGroup(new DriveDistance(1), new IntakeCommand(false));
        } else {
            return new DriveDistance(1);
        }
    }

    public static Command PR() {
        if (Robot.isReal()) {
            return new ParallelCommandGroup(new DriveDistance(2), new IntakeCommand(false));
        } else {
            return new DriveDistance(2);
        }
    }

    public static Command SL() {
        return new SequentialCommandGroup(
                new DriveDistance(AutoConstants.SCORING_POSITIONS[0]),
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed,
                        false));
    }

    public static Command SC() {
        return new SequentialCommandGroup(
                new DriveDistance(AutoConstants.SCORING_POSITIONS[1]),
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed,
                        false));
    }

    public static Command SR() {
        return new SequentialCommandGroup(
                new DriveDistance(AutoConstants.SCORING_POSITIONS[2]),
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed,
                        false));
    }

    public static Command SA() {
        return new SequentialCommandGroup(
                new DriveDistance(new Pose2d(1.8415, 7.7724, new Rotation2d(-Math.PI / 2))),
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.AMP.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.AMP.bottomSpeed,
                        false));
    }
}
