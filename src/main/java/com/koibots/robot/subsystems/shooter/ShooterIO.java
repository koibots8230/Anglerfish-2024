// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double topVelocity = 0;
        public double bottomVelocity = 0;

        public Measure<Current> topCurrent = Amps.of(0);
        public Measure<Current> bottomCurrent = Amps.of(0);
        public Measure<Voltage> topVoltage = Volts.of(0);
        public Measure<Voltage> bottomVoltage = Volts.of(0);

        public double topSetpoint = 0;
        public double bottomSetpoint = 0;
    }

    void updateInputs(ShooterIOInputs inputs);

    void setVelocity(Measure<Velocity<Angle>> top, Measure<Velocity<Angle>> bottom);
}
