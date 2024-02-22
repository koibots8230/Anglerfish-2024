// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopperPivot;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.SetpointConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PlopperPivot extends SubsystemBase {

    private final PlopperPivotIOInputsAutoLogged inputs = new PlopperPivotIOInputsAutoLogged();

    private final PlopperPivotIO io;

    private final ArmFeedforward feedforward;
    private final PIDController feedback;

    private Measure<Angle> setpoint = Radians.of(0);

    public PlopperPivot() {
        io = (Robot.isReal()) ? new PlopperPivotIOSparkMax() : new PlopperPivotIOSim();
        feedforward =
                new ArmFeedforward(
                        ControlConstants.PLOPPER_PIVOT_FEEDFORWARD.ks,
                        ControlConstants.PLOPPER_PIVOT_FEEDFORWARD.kv,
                        ControlConstants.PLOPPER_PIVOT_FEEDFORWARD.ka,
                        ControlConstants.PLOPPER_PIVOT_FEEDFORWARD.kg);

        feedback =
                new PIDController(
                        ControlConstants.PLOPPER_PIVOT_FEEDBACK.kP,
                        ControlConstants.PLOPPER_PIVOT_FEEDBACK.kI,
                        ControlConstants.PLOPPER_PIVOT_FEEDBACK.kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/PlopperPivot", inputs);

        io.setVoltage(
                feedback.calculate(inputs.position.in(Radians), setpoint.in(Radians))
                        + feedforward.calculate(0, 0, 0));

        SmartDashboard.putData("Plopper/Pivot PID", feedback);
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
                        >= SetpointConstants.PLOPPPER_PIVOT_LOAD_POSITION
                                .minus(Degrees.of(2))
                                .in(Radians)
                && inputs.position.in(Radians)
                        <= SetpointConstants.PLOPPPER_PIVOT_LOAD_POSITION
                                .plus(Degrees.of(2))
                                .in(Radians);
    }

    public Measure<Angle> getSetpoint() {
        return setpoint;
    }
}
