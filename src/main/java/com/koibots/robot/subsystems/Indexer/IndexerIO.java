// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);

        public boolean isBrake = true;

        public Measure<Current> current = Amps.of(0);
        public Measure<Voltage> voltage = Volts.of(0);
    }

    void updateInputs(IndexerIOInputs inputs);

    default void setIdle(boolean isBrake) {}
    ;

    void setVoltage(Measure<Voltage> volts);

    Measure<Velocity<Angle>> getVelocity();

    Measure<Voltage> getVoltage();

    default boolean sensorTriggered() {
        return false;
    }
    ;
}
