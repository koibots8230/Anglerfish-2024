// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public Measure<Velocity<Angle>> leftFlywheelVelocity;
        public Measure<Velocity<Angle>> rightFlywheelVelocity;
        public Measure<Current> leftMotorCurrent;
        public Measure<Current> rightMotorCurrent;
        public Measure<Voltage> leftMotorAppliedVoltage;
        public Measure<Voltage> rightMotorAppliedVoltage;
    }

    void updateInputs(ShooterIOInputs inputs);

    void setVoltages(Measure<Voltage> left, Measure<Voltage> right);
}
