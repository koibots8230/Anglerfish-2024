// Copyright 2024 (c) FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
    }

    void updateInputs(GyroIOInputs inputs);

    default void zeroYaw() {}
}
