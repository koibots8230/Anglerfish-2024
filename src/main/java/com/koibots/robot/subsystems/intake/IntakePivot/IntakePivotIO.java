package com.koibots.robot.subsystems.intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
        public Rotation2d intakePivotPosition = new Rotation2d();
        public double intakePivotVelocityRadPerSec = 0.0;
        public double intakePivotAppliedVolts = 0.0;
        public double[] intakePivotCurrentAmps = new double[] {};
    }

    /* Updates the set of loggable inputs. */
    public default void updateInputs(IntakePivotInputs inputs) {}

    /* Run the pivot motor at the specified voltage. */
    public default void setIntakePivotVoltage(double volts) {}

    /* Enable or disable brake mode on the pivot motor. */
    public default void setIntakePivotBrakeMode(boolean enable) {}

    /* Set zero offset for intake pivot encoder. */
    public default void setZeroOffset() {}

    /* Move the intake to a desired position. */
    public default void setPosition(double position) {}
}
