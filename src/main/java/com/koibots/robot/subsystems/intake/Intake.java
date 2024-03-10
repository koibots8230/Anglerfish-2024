// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final PIDController feedback;
    private final SimpleMotorFeedforward feedforward;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private Measure<Velocity<Angle>> setpoint = RPM.of(0);

    private int inverted = 1;

    public Intake() {
        io = Robot.isReal() ? new IntakeIOSparkMax() : new IntakeIOSim();
        feedback = new PIDController(Constants.ControlConstants.INTAKE_FEEDBACK_CONSTANTS.kP, 0, 0);
        feedforward =
                new SimpleMotorFeedforward(
                        Constants.ControlConstants.INTAKE_FEEDFORWARD_CONSTANTS.ks,
                        Constants.ControlConstants.INTAKE_FEEDFORWARD_CONSTANTS.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Intake", inputs);

        io.setVoltage(
                Volts.of(
                        Math.max(
                                Math.min(
                                        (feedback.calculate(
                                                                inputs.velocity.in(RPM),
                                                                setpoint.in(RPM))
                                                        + feedforward.calculate(setpoint.in(RPM)))
                                                * (12.0 / 5676.0),
                                        12.0),
                                -12.0)));
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        double robotSpeed = Swerve.get().getModuleStates()[0].speedMetersPerSecond * 60;
        double targetDistancePerMinute =
                velocity.in(RPM) * Constants.RobotConstants.INTAKE_WHEELS.circumfrence.in(Meters);
        double trueDistancePerMinute = targetDistancePerMinute - robotSpeed;

        setpoint =
                RPM.of(
                        trueDistancePerMinute
                                / Constants.RobotConstants.INTAKE_WHEELS.circumfrence.in(Meters));
        setpoint.times(inverted);
    }

    public void invert() {
        inverted *= -1;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        io.setVoltage(voltage);
    }
}
