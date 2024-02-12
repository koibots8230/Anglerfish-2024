// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.PlopperConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Plopper extends SubsystemBase {

    private final PlopperIOInputsAutoLogged inputs = new PlopperIOInputsAutoLogged();

    private final PlopperIO io;

    private final SimpleMotorFeedforward feedforward;
    private final PIDController feedback;

    Measure<Velocity<Angle>> setpoint = RPM.of(0);

    public Plopper() {
        io = Robot.isReal() ? new PlopperIOSparkMax() : new PlopperIOSim();

        feedforward =
                new SimpleMotorFeedforward(
                        PlopperConstants.FEEDFORWARD_CONSTANTS.ks,
                        PlopperConstants.FEEDFORWARD_CONSTANTS.kv);
        feedback =
                new PIDController(
                        PlopperConstants.FEEDBACK_CONSTANTS.kP,
                        PlopperConstants.FEEDBACK_CONSTANTS.kI,
                        PlopperConstants.FEEDBACK_CONSTANTS.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Plopper", inputs);

        io.setVoltage(
                Volts.of(Math.max(Math.min(
                        (feedback.calculate(
                                inputs.velocity.in(RPM),
                                setpoint.in(RPM))
                        + feedforward.calculate(
                                setpoint.in(RPM)))
                        * (12.0 / 11000.0), 12.0), -12.0)));
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpoint = velocity;
    }

    public boolean sensorTriggered() {
        return io.sensorTriggered();
    }
}
