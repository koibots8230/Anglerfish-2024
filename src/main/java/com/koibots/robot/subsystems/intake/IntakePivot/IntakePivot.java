package com.koibots.robot.subsystems.intake.IntakePivot;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakePivot extends SubsystemBase {

    IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    private final IntakePivotIO io;
    SimpleMotorFeedforward intakePivotFeedForward;
    private final PIDController intakePivotFeedback;
    private Rotation2d turnRelativeOffset = null;
    private double positionSetPoint;

    public IntakePivot() {

        if (Robot.isReal()) {
            positionSetPoint = 0;
            this.io = new IntakePivotIOSparkMax(0);
            intakePivotFeedForward = new SimpleMotorFeedforward(0, 0);
            intakePivotFeedback = new PIDController(0, 0, 0);
        } else {
            positionSetPoint = 0;
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
        io.setPosition(intakePivotFeedback.calculate(inputs.intakePivotPosition.getRadians(), positionSetPoint) + intakePivotFeedForward.calculate(0,0,0));
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
    }

    public void setIntakePivotPosition(double position) {
        this.positionSetPoint = position;
    }
}