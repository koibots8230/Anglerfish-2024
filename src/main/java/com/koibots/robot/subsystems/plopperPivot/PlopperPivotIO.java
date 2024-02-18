// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopperPivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface PlopperPivotIO {
    @AutoLog
    public static class PlopperPivotIOInputs {
        public Measure<Angle> position = Rotations.of(0);
        public Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);

        public Measure<Voltage> voltage = Volts.of(0);
        public Measure<Current> current = Amps.of(0);
    }

    void updateInputs(PlopperPivotIOInputs inputs);

    void setVoltage(double volts);

    default void setIdleMode(boolean isBrake) {}
    ;
}
