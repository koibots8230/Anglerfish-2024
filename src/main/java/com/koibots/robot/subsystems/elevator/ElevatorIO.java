// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorInputs {
        public double position = 0.0;
        public double setpoint = 0.0;
        public double appliedVoltage = 0.0;
        public double leftAmperage = 0.0;
        public double rightAmperage = 0.0;
    }

    default void updateInputs(ElevatorInputs inputs) {}

    /* Set the motors to a specified voltage */
    default void setVoltage(double volts) {}

    /* Set the motors to brake mode */
    default void setBrake() {}

    /* Get the encoder position */
    default double getPosition() {
        return 0;
    }

    /* Get the encoder velocity */
    default double getVelocity() {
        return 0;
    }

    default Mechanism2d getMechanism() {
        return null;
    }
}
