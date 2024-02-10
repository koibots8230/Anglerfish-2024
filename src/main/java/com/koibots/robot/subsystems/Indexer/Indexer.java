// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.IndexerConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforwardController;
    private final PIDController feedbackController;

    private Measure<Voltage> setpoint = Volts.of(0);

    public Indexer() {
        io = (Robot.isReal()) ? new IndexerIOSparkMax() : new IndexerIOSim();
        feedforwardController =
                new SimpleMotorFeedforward(
                        IndexerConstants.FEEDFORWARD_CONSTANTS.ks,
                        IndexerConstants.FEEDFORWARD_CONSTANTS.kv);
        feedbackController =
                new PIDController(
                        IndexerConstants.FEEDBACK_CONSTANTS.kP,
                        IndexerConstants.FEEDBACK_CONSTANTS.kI,
                        IndexerConstants.FEEDBACK_CONSTANTS.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Indexer", inputs);

        io.setVoltage(
                Volts.of(
                        feedbackController.calculate(io.getVoltage().in(Volts), setpoint.in(Volts))
                                + feedforwardController.calculate(setpoint.in(Volts))));
    }

    public void setVolts(Measure<Voltage> volts) {
        setpoint = volts;
    }

    public void setIdleMode(boolean doBrake) {
        io.setIdle(doBrake);
    }
}
