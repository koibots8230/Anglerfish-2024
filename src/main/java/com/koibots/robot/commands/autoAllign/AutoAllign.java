// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.autoAllign;

import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.subsystems.Subsystems;
import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAllign extends Command {
    Pose2d goal;

    public AutoAllign(Pose2d goal) {
        this.goal = goal;
    }

    @Override
    public void initialize() {
        new PathfindHolonomic(
                        goal,
                        DriveConstants.CONSTRAINTS,
                        0.0,
                        Subsystems.Swerve.get()::getEstimatedPose,
                        null,
                        null,
                        DriveConstants.PATH_CONFIG,
                        0.0,
                        Subsystems.Swerve.get())
                .schedule();
    }
}
