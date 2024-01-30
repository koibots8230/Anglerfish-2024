// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import com.koibots.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

    private Indexer() {
        io = (Robot.isReal()) ? new IndexerIOSparkMax() : new IndexerIOSim();
    }

    @Override
    public void periodic() {
        io.updateInputs(indexerInputs);
    }

    public void runIndexer(double speed) {
        io.runMotor(speed);
    }

    public void indexerMode(boolean isBrake) {
        io.setIdle(isBrake);
    }
}
