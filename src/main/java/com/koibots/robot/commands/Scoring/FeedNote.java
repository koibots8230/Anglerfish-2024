// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.Indexer;
import static com.koibots.robot.subsystems.Subsystems.Shooter;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedNote extends Command {

    private boolean shooterAtSpeed = false;

    public FeedNote() {
        addRequirements(Indexer.get());
    }

    @Override
    public void initialize() {
        shooterAtSpeed = false;
    }

    @Override
    public void execute() {
        if (!shooterAtSpeed) {
            shooterAtSpeed = Shooter.get().atSetpoint();
        } else {
            Indexer.get().setVelocity(SetpointConstants.SHOOTER_INDEXER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.get().setVelocity(RPM.of(0));
    }
}