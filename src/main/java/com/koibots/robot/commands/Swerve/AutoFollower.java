// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Swerve;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class AutoFollower extends Command {
    private final ChoreoTrajectory trajectory;
    private double initialTime;

    public AutoFollower(ChoreoTrajectory trajectory) {
        if (DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent()) {
            this.trajectory = trajectory.flipped();
        } else {
            this.trajectory = trajectory;
        }
    }

    @Override
    public void initialize() {
        initialTime = Logger.getRealTimestamp();
        Swerve.get().resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        var nextState = trajectory.sample(Logger.getRealTimestamp() - initialTime + 20000);
        Pose2d currentPose = Swerve.get().getEstimatedPose();

        Swerve.get()
                .driveRobotRelative(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                (nextState.x - currentPose.getX()) / 0.02,
                                (nextState.y - currentPose.getY()) / 0.02,
                                (nextState.heading - currentPose.getRotation().getRadians()) / 0.02,
                                Swerve.get().getGyroAngle()));
    }
}
