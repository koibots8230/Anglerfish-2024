// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import com.koibots.robot.Robot;
import com.koibots.robot.Constants.IndexerConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;
    private double desiredVolts;

    private Indexer() {
        io = (Robot.isReal()) ? new IndexerIOSparkMax() : new IndexerIOSim();
        feedforward = (Robot.isReal()) ? new SimpleMotorFeedforward(IndexerConstants.SPARKMAX_KS, IndexerConstants.SPARKMAX_KV, IndexerConstants.SPARKMAX_KA) : new SimpleMotorFeedforward(IndexerConstants.SIM_KS, IndexerConstants.SIM_KV, IndexerConstants.SIM_KA);
        pid = (Robot.isReal()) ? new PIDController(IndexerConstants.SPARKMAX_KP, IndexerConstants.SPARKMAX_KI, IndexerConstants.SPARKMAX_KD) : new PIDController(IndexerConstants.SIM_KP, IndexerConstants.SIM_KI, IndexerConstants.SIM_KD);
        desiredVolts = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(indexerInputs);
        io.setVoltage(pid.calculate(io.getVoltage(), desiredVolts) + feedforward.calculate(0, 0, 0));
    }

    public void runIndexer(double input) {
        desiredVolts = input;
    }

    public void indexerMode(boolean isBrake) {
        io.setIdle(isBrake);
    }
}
