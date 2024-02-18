// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopperPivot;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.PlopperPivotConstants;
import com.koibots.robot.subsystems.plopperPivot.PlopperPivotIOInputsAutoLogged;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PlopperPivot extends SubsystemBase {

    private final PlopperPivotIOInputsAutoLogged inputs = new PlopperPivotIOInputsAutoLogged();

    private final PlopperPivotIO io;

    private final ArmFeedforward feedforwardController;
    private final PIDController feedbackController;

    private Measure<Angle> setpoint = Radians.of(0);

    public PlopperPivot() {
        io = (Robot.isReal()) ? new PlopperPivotIOSparkMax() : new PlopperPivotIOSim();
        feedforwardController =
                new ArmFeedforward(
                        PlopperPivotConstants.FEEDFORWARD_CONSTANTS.ks,
                        PlopperPivotConstants.FEEDFORWARD_CONSTANTS.kv,
                        PlopperPivotConstants.FEEDFORWARD_CONSTANTS.ka,
                        PlopperPivotConstants.FEEDFORWARD_CONSTANTS.kg);

        feedbackController =
                new PIDController(
                        PlopperPivotConstants.FEEDBACK_CONSTANTS.kP,
                        PlopperPivotConstants.FEEDBACK_CONSTANTS.kI,
                        PlopperPivotConstants.FEEDBACK_CONSTANTS.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/PlopperPivot", inputs);

        io.setVoltage(
                feedbackController.calculate(inputs.position.in(Radians), setpoint.in(Radians))
                        + feedforwardController.calculate(0, 0, 0));
    }

    // setters

    public void setMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }

    // commands

    public void setPosition(Measure<Angle> desiredPosition) {
        setpoint = desiredPosition;
    }

    public boolean atSetpoint() {
        return inputs.position.in(Radians)
                        >= PlopperPivotConstants.LOAD_POSITION
                                .minus(PlopperPivotConstants.ALLOWED_ERROR)
                                .in(Radians)
                && inputs.position.in(Radians)
                        <= PlopperPivotConstants.LOAD_POSITION
                                .plus(PlopperPivotConstants.ALLOWED_ERROR)
                                .in(Radians);
    }

    public Measure<Angle> getSetpoint() {
        return setpoint;
    }
}
