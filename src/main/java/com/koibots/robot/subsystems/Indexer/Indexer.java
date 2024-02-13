// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.IndexerConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforwardController;
    private final PIDController feedbackController;

    private Measure<Velocity<Angle>> setpoint = RPM.of(0);

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
                        Math.max(
                                Math.min(
                                        (feedbackController.calculate(
                                                                inputs.velocity.in(RPM),
                                                                setpoint.in(RPM))
                                                        + feedforwardController.calculate(
                                                                setpoint.in(RPM)))
                                                * (12.0 / 11000.0),
                                        12.0),
                                -12.0)));
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpoint = velocity;
    }

    public void setIdleMode(boolean doBrake) {
        io.setIdle(doBrake);
    }

    public boolean sensorTriggered() {
        return io.sensorTriggered();
    }
}
