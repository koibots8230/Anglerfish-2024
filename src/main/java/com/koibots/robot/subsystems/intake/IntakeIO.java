// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public Rotation2d intakePosition = new Rotation2d();
        public double intakeVelocity = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeVoltage = 0.0;
    }

    /* Updates the set of loggable inputs. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /* Run the pivot motor at the specified voltage. */
    default void setVoltage(double volts) {}
}
