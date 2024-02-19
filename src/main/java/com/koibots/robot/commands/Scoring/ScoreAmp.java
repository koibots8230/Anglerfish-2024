// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Meters;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.Constants.DriveConstants;
import com.koibots.robot.RobotContainer;
import com.koibots.robot.commands.Ploppervator.RunPlopper;
import com.koibots.robot.commands.Ploppervator.SetPloppervatorPosition;
import com.koibots.robot.commands.Swerve.AutoAlign;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreAmp extends SequentialCommandGroup {

    public ScoreAmp() {
        if (Math.abs(Swerve.get().getEstimatedPose().minus(DriveConstants.AMP_POSITION).getX())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_AMP.getX()
                && Math.abs(
                                Swerve.get()
                                        .getEstimatedPose()
                                        .minus(DriveConstants.AMP_POSITION)
                                        .getY())
                        < DriveConstants.ALLOWED_DISTANCE_FROM_AMP.getY()) {
            addCommands(
                    new AutoAlign(DriveConstants.AMP_POSITION, Meters.of(0)),
                    new SetPloppervatorPosition(PloppervatorPosition.Amp),
                    new RunPlopper(false));
        } else {
            addCommands(
                    new InstantCommand(() -> RobotContainer.rumbleController(0.4)),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> RobotContainer.rumbleController(0.0)));
        }
    }

    public ScoreAmp(boolean doPathing) {
        if (doPathing) {
            addCommands(new ScoreAmp());
        } else {
            addCommands(
                    new AutoAlign(DriveConstants.AMP_POSITION, Meters.of(0)),
                    new SetPloppervatorPosition(PloppervatorPosition.Amp),
                    new RunPlopper(false));
        }
    }
}
