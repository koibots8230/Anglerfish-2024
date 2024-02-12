// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.ShooterConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private Measure<Velocity<Angle>> setpoint = RotationsPerSecond.of(0);

    PIDController leftFlywheelFeedbackController;
    PIDController rightFlywheelFeedbackController;
    SimpleMotorFeedforward leftFlywheelFeedforwardController;
    SimpleMotorFeedforward rightFlywheelFeedforwardController;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();

        leftFlywheelFeedbackController = new PIDController(ShooterConstants.kP, 0, 0);
        rightFlywheelFeedbackController = new PIDController(ShooterConstants.kP, 0, 0);

        leftFlywheelFeedforwardController =
                new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
        rightFlywheelFeedforwardController =
                new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Shooter", inputs);

        io.setVoltages(
                Volts.of(
                        Math.max(Math.min(
                                (leftFlywheelFeedbackController.calculate(
                                                inputs.leftVelocity.in(RotationsPerSecond),
                                                setpoint.in(RotationsPerSecond))
                                + leftFlywheelFeedforwardController.calculate(
                                                setpoint.in(RotationsPerSecond)))
                                * (12.0 / 5676.0), 12.0), -12.0)),
                Volts.of(
                        Math.max(Math.min(
                                (rightFlywheelFeedbackController.calculate(
                                                inputs.rightVelocity.in(RotationsPerSecond),
                                                setpoint.in(RotationsPerSecond))
                                + rightFlywheelFeedforwardController.calculate(
                                                setpoint.in(RotationsPerSecond)))
                                * (12.0 / 5676.0), 12.0), -12.0))
        );
    }

    public void setVelocity(Measure<Velocity<Angle>> speed) {
        setpoint = speed;
    }

    public double getAverageSpeed() {
        return io.getAverageSpeed();
    }
}
