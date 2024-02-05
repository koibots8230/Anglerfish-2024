// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.autoAllign;

import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.subsystems.Subsystems;
import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoAllign extends ParallelCommandGroup {
    Pose2d goal;
    PathfindHolonomic pathfinder;

    public AutoAllign(Pose2d goal) {
        addCommands(
            new PathfindHolonomic(
                        goal,
                        DriveConstants.CONSTRAINTS,
                        0.0,
                        Subsystems.Swerve.get()::getEstimatedPose,
                        null,
                        null,
                        DriveConstants.PATH_CONFIG,
                        0.0,
                        Subsystems.Swerve.get()),
            
            new ConditionalCommand(() ->  (),
                                   null,
                                   () -> Math.sqrt(Math.pow(Swerve.get().getEstimatedPose().relativeTo(goal).getX(), 2) + Math.pow(Swerve.get().getEstimatedPose().relativeTo(goal).getY(), 2)) < 1),
            new ConditionalCommand()
        );
    }
}
