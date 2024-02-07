// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import com.koibots.robot.Constants.IndexerConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforwardController;
    private final PIDController feedbackController;
    private double desiredVolts = 0;

    public Indexer() {
        io = (Robot.isReal()) ? new IndexerIOSparkMax() : new IndexerIOSim();
        feedforwardController = new SimpleMotorFeedforward(IndexerConstants.FEEDFORWARD_CONSTANTS.ks, IndexerConstants.FEEDFORWARD_CONSTANTS.kv);
        feedbackController = new PIDController(IndexerConstants.FEEDBACK_CONSTANTS.kP, IndexerConstants.FEEDBACK_CONSTANTS.kI, IndexerConstants.FEEDBACK_CONSTANTS.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(indexerInputs);
        io.setVoltage(
                feedbackController.calculate(io.getVoltage(), desiredVolts) + feedforwardController.calculate(0, 0, 0));
    }

    public void runIndexer(double input) {
        desiredVolts = input;
    }

    public void indexerMode(boolean isBrake) {
        io.setIdle(isBrake);
    }
}
