package com.koibots.robot.subsystems;

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
        if (Robot.isReal()) {
            io = new ShooterPivotIOSparkMax();
        } else {
            io = new ShooterPivotIOSim();
        }
        desiredPos = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(pivotInputs);
        io.setShooterPivotMotorSpeed(PID.calculate(pivotInputs.shooterPivotPosition, desiredPos) + Feedforward.calculate(0, 0, 0));
    }

    //getters

    public double getShooterMotorPosition() {
        return io.getShooterPosition();
    }

    public double getShooterMotorCurrent() {
        return io.getShooterPivotOutputCurrent();
    }

    //setters

    public void resetShooterMotorPosition() {
        io.zeroShooterPositionOffset();
    }

    public void setShooterPositionModeBreak() {
        io.setShooterPivotBrakeMode();
    }

    public void setShooterPositionModeCoast() {
        io.setShooterPivotCoastMode();
    }

    //commands

    public void setShooterPosition(double desiredPosition) {
        desiredPos = desiredPosition;
    }
}