// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public Measure<Velocity<Angle>> velocity = RevolutionsPerSecond.of(0);
        public boolean mode = true;
    }

    void updateInputs(IndexerIOInputs inputs);

    void setIdle(boolean isBrake);

    void setVoltage(double volts);

    double getVoltage();
}
