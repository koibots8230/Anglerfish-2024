// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface PlopperIO {

    @AutoLog
    public class PlopperIOInputs {
        Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);

        Measure<Current> current = Amps.of(0);
        Measure<Voltage> voltage = Volts.of(0);
    }

    void updateInputs(PlopperIOInputs inputs);

    void setVoltage(Measure<Voltage> volts);

    default boolean sensorTriggered() {
        return false;
    }
}
