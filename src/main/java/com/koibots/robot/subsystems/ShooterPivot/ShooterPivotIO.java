// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.ShooterPivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
    @AutoLog
    public static class ShooterPivotIOInputs {
        public Rotation2d position = new Rotation2d();
        public Measure<Voltage> voltage = Volts.of(0);
        public Measure<Current> current = Amps.of(0);
        public Measure<Velocity<Angle>> velocity = RadiansPerSecond.of(0);
    }

    void updateInputs(ShooterPivotIOInputs inputs);

    void setMotorSpeed(double desiredPosition);

    void setIdleMode(boolean isBrake);
}
