package com.koibots.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.*;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public Measure<Velocity<Angle>> velocity = RevolutionsPerSecond.of(0); 
        public boolean mode = true;
    }

    void updateInputs(IndexerIOInputs inputs);
    void setIdle(boolean mode);
    void runMotor(double speed);
}
