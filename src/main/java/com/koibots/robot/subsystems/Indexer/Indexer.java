// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward;
    private final PIDController feedback;

    private Measure<Velocity<Angle>> setpoint = RPM.of(0);

    private int inverted = 1;
    private boolean sensorEnabled = true;

    public Indexer() {
        io = (Robot.isReal()) ? new IndexerIOSparkMax() : new IndexerIOSim();
        feedforward =
                new SimpleMotorFeedforward(
                        Constants.ControlConstants.INDEXER_FEEDFORWARD_CONSTANTS.ks,
                        Constants.ControlConstants.INDEXER_FEEDFORWARD_CONSTANTS.kv);
        feedback =
                new PIDController(
                        Constants.ControlConstants.INDEXER_FEEDBACK_CONSTANTS.kP,
                        Constants.ControlConstants.INDEXER_FEEDBACK_CONSTANTS.kI,
                        Constants.ControlConstants.INDEXER_FEEDBACK_CONSTANTS.kD);

        SmartDashboard.putData("Indexer/PID", feedback);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.setpoint = setpoint.in(RPM);
        Logger.processInputs("Subsystems/Indexer", inputs);

        io.setVoltage(
                Volts.of(
                        Math.max(
                                Math.min(
                                        (feedback.calculate(inputs.velocity, setpoint.in(RPM))
                                                        + feedforward.calculate(setpoint.in(RPM)))
                                                * (12.0 / 11000.0),
                                        12.0),
                                -12.0)));
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        System.out.println("Indexer: " + velocity.in(RPM));
        setpoint = velocity.times(inverted);
    }

    public void invert() {
        inverted *= -1;
        sensorEnabled = !sensorEnabled;
    }

    public void setVoltage(Measure<Voltage> volts) {
        io.setVoltage(volts);
    }

    public void setIdleMode(boolean doBrake) {
        io.setIdle(doBrake);
    }

    public boolean sensorTriggered() {
        SmartDashboard.putBoolean("Note Detected", io.sensorTriggered());
        return io.sensorTriggered();
    }
}