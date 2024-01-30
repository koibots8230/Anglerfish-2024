package com.koibots.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorInputs {
        public double position = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    /* Set the motors to a specified voltage */
    public default void setVoltage(double volts) {}

    /* Set the motors to brake mode */
    public default void setBrake() {}

    /* Get the encoder position */
    public default double getPosition() {
        return 0;
    }

    /* Get the encoder velocity */
    public default double getVelocity() {
        return 0;
    }
} 
