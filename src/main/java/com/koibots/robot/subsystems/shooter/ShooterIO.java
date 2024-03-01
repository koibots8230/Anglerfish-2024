// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double leftVelocity = 0;
        public double rightVelocity = 0;

        public Measure<Current> leftCurrent = Amps.of(0);
        public Measure<Current> rightCurrent = Amps.of(0);
        public Measure<Voltage> leftVoltage = Volts.of(0);
        public Measure<Voltage> rightVoltage = Volts.of(0);

        public Measure<Velocity<Angle>> setpoint = RPM.of(0);
    }

    void updateInputs(ShooterIOInputs inputs);

    void setVoltages(Measure<Voltage> left, Measure<Voltage> right);
}
