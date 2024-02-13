// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.IntakeConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final PIDController intakeFeedback;
    private final SimpleMotorFeedforward intakeFeedForward;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeVoltsSetPoint = 0.0;

    public Intake() {
        io = Robot.isReal() ? new IntakeIOSparkMax() : new IntakeIOSim();
        intakeFeedback = new PIDController(IntakeConstants.FEEDBACK_CONSTANTS.kP, 0, 0);
        intakeFeedForward =
                new SimpleMotorFeedforward(
                        IntakeConstants.FEEDFORWARD_CONSTANTS.ks,
                        IntakeConstants.FEEDFORWARD_CONSTANTS.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Intake", inputs);

        io.setVoltage(
                Volts.of(
                        intakeFeedback.calculate(
                                inputs.voltage.in(Volts),
                                intakeVoltsSetPoint
                                        + intakeFeedForward.calculate(intakeVoltsSetPoint))));
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        double robotSpeed = Swerve.get().getModuleStates()[0].speedMetersPerSecond * 60;
        double targetDistancePerMinute =
                velocity.in(RPM) * IntakeConstants.WHEELS.circumfrence.in(Meters);
        double trueDistancePerMinute = targetDistancePerMinute - robotSpeed;

        double trueRPM = trueDistancePerMinute / IntakeConstants.WHEELS.circumfrence.in(Meters);
        intakeVoltsSetPoint = Math.max(Math.min(trueRPM * (12.0 / 5676.0), 12.0), -12.0);
    }
}
