// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Intake;

import static com.koibots.robot.subsystems.Subsystems.Indexer;
import static edu.wpi.first.units.Units.RPM;

import javax.management.InstanceAlreadyExistsException;

import com.koibots.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeShooter extends Command {

    private boolean hasSeen = false;
    private boolean reversed = false;

    public IntakeShooter() {
        addRequirements(Indexer.get());
    }

    @Override
    public void initialize() {
        hasSeen = false;
        reversed = false;
        Indexer.get().setVelocity(RPM.of(-200));
    }

    @Override
    public void execute() {
        if (Indexer.get().sensorTriggered() && !hasSeen) {
            hasSeen = true;
        } else if (!Indexer.get().sensorTriggered() && hasSeen) {
            reversed = true;
            Indexer.get().setVelocity((RPM.of(60)));
        }
    }

    @Override
    public boolean isFinished() {
        return reversed && Indexer.get().sensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.get().setVelocity(RPM.of(0));

        if (!interrupted) {
            System.out.println("Rumble rumble");
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotContainer.rumbleController(0.5)),
                new WaitCommand(0.4),
                new InstantCommand(() -> RobotContainer.rumbleController(0))
            ).schedule();
        }
    }
}
