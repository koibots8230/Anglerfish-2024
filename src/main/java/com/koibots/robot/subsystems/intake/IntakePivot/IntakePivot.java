package com.koibots.robot.subsystems.intake.IntakePivot;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Robot;
import com.koibots.robot.Constants.IntakePivotState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakePivot extends SubsystemBase {

    IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();
    private IntakePivotState intakePivotState;

    private final IntakePivotIO io;
    SimpleMotorFeedforward intakePivotFeedForward;
    private final PIDController intakePivotFeedback;
    private Rotation2d angleSetpoint = null;
    private Rotation2d turnRelativeOffset = null;

    public IntakePivot() {
        this.intakePivotState = IntakePivotState.UP;

        if (Robot.isReal()) {
            this.io = new IntakePivotIOSparkMax(0);
            intakePivotFeedForward = new SimpleMotorFeedforward(0, 0);
            intakePivotFeedback = new PIDController(0, 0, 0);
        } else {
            this.io = new IntakePivotIOSim();
            intakePivotFeedForward = new SimpleMotorFeedforward(0, 0);
            intakePivotFeedback = new PIDController(0, 0, 0);
        }

        intakePivotFeedback.enableContinuousInput(0, 0);
        SmartDashboard.putData("IntakePivot PID ", intakePivotFeedback);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Pivot", inputs);

        // Run closed loop pivot control
        if (angleSetpoint != null) {
            io.setIntakePivotVoltage(
                intakePivotFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians())
            );
        }
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.intakePivotPosition.plus(turnRelativeOffset);
        }
    }

    public void stop() {
        io.setIntakePivotVoltage(0.0);
        angleSetpoint = null;
    }

    public IntakePivotState getState() {
        return this.intakePivotState;
    }
}