// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorInputs {
        public double position = 0.0;
        public double setpoint = 0.0;
        public double appliedVoltage = 0.0;
        public double leftAmperage = 0.0;
        public double rightAmperage = 0.0;
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

    public default Mechanism2d getMechanism() {
        return null;
    }
}
