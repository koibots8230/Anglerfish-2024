// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopper;

import com.koibots.robot.Constants.PlopperConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Plopper extends SubsystemBase {

    private final PlopperIOInputsAutoLogged pivotInputs = new PlopperIOInputsAutoLogged();

    private final PlopperIO io;

    private final ArmFeedforward feedforwardController;
    private final PIDController feedbackController;

    private double desiredPos = 0;

    public Plopper() {
        io = (Robot.isReal()) ? new PlopperIOSparkMax() : new PlopperIOSim();
        feedforwardController =
                new ArmFeedforward(
                        PlopperConstants.FEEDFORWARD_CONSTANTS.ks,
                        PlopperConstants.FEEDFORWARD_CONSTANTS.kv,
                        PlopperConstants.FEEDFORWARD_CONSTANTS.ka,
                        PlopperConstants.FEEDFORWARD_CONSTANTS.kg);

        feedbackController =
                new PIDController(
                        PlopperConstants.FEEDBACK_CONSTANTS.kP,
                        PlopperConstants.FEEDBACK_CONSTANTS.kI,
                        PlopperConstants.FEEDBACK_CONSTANTS.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(pivotInputs);
        io.setVoltage(
                feedbackController.calculate(pivotInputs.position.getRadians(), desiredPos)
                        + feedforwardController.calculate(0, 0, 0));
    }

    // setters

    public void setShooterPivotMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }

    // commands

    public void setShooterPosition(double desiredPosition) {
        desiredPos = desiredPosition;
    }
}
