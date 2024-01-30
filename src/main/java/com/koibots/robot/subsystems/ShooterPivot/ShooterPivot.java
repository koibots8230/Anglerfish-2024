// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.ShooterPivot;

import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {

    private ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final ShooterPivotIO io;

    private double desiredPos;

    ArmFeedforward Feedforward = new ArmFeedforward(0, 0, 0, 0);
    PIDController PID = new PIDController(0, 0, 0);

    public ShooterPivot() {
        io = (Robot.isReal()) ? new ShooterPivotIOSparkMax() : new ShooterPivotIOSim();
        desiredPos = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(pivotInputs);
        io.setMotorSpeed(
                PID.calculate(pivotInputs.position.getRadians(), desiredPos)
                        + Feedforward.calculate(0, 0, 0));
    }

    // setters

    public void setShooterPivotMode(boolean mode) {
        io.setIdleMode(mode);
    }

    // commands

    public void setShooterPosition(double desiredPosition) {
        desiredPos = desiredPosition;
    }
}
