package com.koibots.robot.subsystems.Indexer;

import com.koibots.robot.Robot;

public class Indexer {
    private final IndexerIO io;

    private Indexer() {
        if(Robot.isReal()) {
            io = new IndexerIOSparkMax();
        } else {
            io = new IndexerIOSim();
        }
    }

    public void runIndexer() {
        io.runMotor();
    }

    public void indexerMode(boolean mode) {
        io.setIdle(mode);
    }
}
