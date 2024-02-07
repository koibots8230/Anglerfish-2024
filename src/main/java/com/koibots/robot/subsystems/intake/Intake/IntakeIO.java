package com.koibots.robot.subsystems.intake.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface IntakeIO {
    
    @AutoLog
    public static class IntakeInputs {
        public Rotation2d intakePosition = new Rotation2d();
        public double intakeVelocity = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeVoltage = 0.0;
    }

    /* Updates the set of loggable inputs. */
    public default void updateInputs(IntakeInputs inputs) {}

    /* Run the pivot motor at the specified voltage. */
    public default void setVoltage(double volts) {}

}
