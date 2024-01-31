// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.ShooterPivot;

import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.koibots.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {

    private ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final ShooterPivotIO io;

    private final ArmFeedforward feedforward;
    private final PIDController PID;

    private double desiredPos;

    
    public ShooterPivot() {
        io = (Robot.isReal()) ? new ShooterPivotIOSparkMax() : new ShooterPivotIOSim();
        desiredPos = 0;
        feedforward = (Robot.isReal()) ? 
            new ArmFeedforward(ShooterPivotConstants.REAL_KS, 
                ShooterPivotConstants.REAL_KG, 
                ShooterPivotConstants.REAL_KV, 
                ShooterPivotConstants.REAL_KA) : 
            new ArmFeedforward(ShooterPivotConstants.SIM_KS, 
                ShooterPivotConstants.SIM_KG, 
                ShooterPivotConstants.SIM_KV, 
                ShooterPivotConstants.SIM_KA);
        PID = (Robot.isReal()) ? 
            new PIDController(
                ShooterPivotConstants.REAL_KP, 
                ShooterPivotConstants.REAL_KI, 
                ShooterPivotConstants.REAL_KD) : 
            new PIDController(
                ShooterPivotConstants.SIM_KP, 
                ShooterPivotConstants.SIM_KI, 
                ShooterPivotConstants.SIM_KD);
    }

    @Override
    public void periodic() {
        io.updateInputs(pivotInputs);
        io.setVoltage(
                PID.calculate(pivotInputs.position.getRadians(), desiredPos)
                        + feedforward.calculate(0, 0, 0));
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
