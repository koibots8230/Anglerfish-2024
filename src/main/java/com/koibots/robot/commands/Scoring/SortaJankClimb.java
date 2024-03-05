// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import static com.koibots.robot.subsystems.Subsystems.Elevator;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class SortaJankClimb extends Command {

    private double direction;

    public SortaJankClimb(boolean up) {
        direction = (up) ? 0.5 : -1;
    }

    @Override
    public void initialize() {
        Elevator.get().setVoltage(Volts.of(5 * direction));
    }

    @Override
    public boolean isFinished() {
        return Elevator.get()
                .getPosition()
                .isNear(
                        (direction > 0)
                                ? SetpointConstants.ELEVATOR_TOP_HEIGHT
                                : SetpointConstants.ELEVATOR_BOTTOM_HEIGHT,
                        0.05);
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.get().setVoltage(Volts.of(0));
    }
}
