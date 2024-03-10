// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.SetpointConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private Measure<Velocity<Angle>> topSetpoint = RPM.of(0);
    private Measure<Velocity<Angle>> bottomSetpoint = RPM.of(0);

    PIDController topFeedback;
    PIDController bottomFeedback;
    SimpleMotorFeedforward topFeedforward;
    SimpleMotorFeedforward bottomFeedforward;

    private int topInverted = 1;
    private int bottomInverted = 0;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();

        topFeedback = new PIDController(ControlConstants.SHOOTER_FEEDBACK.kP, 0, 0);
        bottomFeedback = new PIDController(ControlConstants.SHOOTER_FEEDBACK.kP, 0, 0);

        topFeedforward =
                new SimpleMotorFeedforward(ControlConstants.SHOOTER_FEEEDFORWARD.ks, ControlConstants.SHOOTER_FEEEDFORWARD.kv);
        bottomFeedforward =
                new SimpleMotorFeedforward(ControlConstants.SHOOTER_FEEEDFORWARD.ks, ControlConstants.SHOOTER_FEEEDFORWARD.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.topSetpoint = topSetpoint;
        inputs.bottomSetpoint = bottomSetpoint;
        Logger.processInputs("Subsystems/Shooter", inputs);

        io.setVoltages(
                Volts.of(
                        Math.max(Math.min(
                                (topFeedback.calculate(
                                                inputs.topVelocity,
                                                topSetpoint.in(RotationsPerSecond))
                                + topFeedforward.calculate(
                                                topSetpoint.in(RotationsPerSecond)))
                                * (12.0 / 5676.0), 12.0), -12.0)),
                Volts.of(
                        Math.max(Math.min(
                                (bottomFeedback.calculate(
                                                inputs.bottomVelocity,
                                                bottomSetpoint.in(RotationsPerSecond))
                                + bottomFeedforward.calculate(
                                                bottomSetpoint.in(RotationsPerSecond)))
                                * (12.0 / 5676.0), 12.0), -12.0))
        );

        SmartDashboard.putData("Shooter/Left PID", topFeedback);
        SmartDashboard.putData("Shooter/Right PID", bottomFeedback);
    }

    public void setVelocity(Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
        topSetpoint = topSpeed.times(topInverted);
        bottomSetpoint = bottomSpeed.times(bottomInverted);
    }

    public void setVoltage(Measure<Voltage> volts) {
        //io.setVoltages(volts, volts);
    }

    public boolean atSetpoint() {
        return inputs.topVelocity
                        >= topSetpoint.minus(SetpointConstants.SHOOTER_ALLOWED_ERROR).in(RPM)
                && inputs.topVelocity
                        <= topSetpoint.plus(SetpointConstants.SHOOTER_ALLOWED_ERROR).in(RPM)
                && inputs.bottomVelocity
                        >= bottomSetpoint.minus(SetpointConstants.SHOOTER_ALLOWED_ERROR).in(RPM)
                && inputs.bottomVelocity
                        <= bottomSetpoint.plus(SetpointConstants.SHOOTER_ALLOWED_ERROR).in(RPM);
    }

    public List<Measure<Current>> getCurrent() {
        return Arrays.asList(inputs.topCurrent, inputs.bottomCurrent);
    }

    public void invertTop() {
        topInverted *= -1;
    }
    
    public void invertBottom() {
        bottomInverted *= -1;
    }
}
