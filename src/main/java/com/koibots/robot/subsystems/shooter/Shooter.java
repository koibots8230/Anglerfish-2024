// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import com.koibots.robot.Constants.ShooterConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private Measure<Velocity<Angle>> setpoint = RotationsPerSecond.of(0);

    PIDController leftFlywheelFeedbackController;
    PIDController rightFlywheelFeedbackController;
    SimpleMotorFeedforward leftFlywheelFeedforwardController;
    SimpleMotorFeedforward rightFlywheelFeedforwardController;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : null;

        leftFlywheelFeedbackController =
                new PIDController(ShooterConstants.kP, 0, 0); // /uhm this was autogenrated...
        rightFlywheelFeedbackController =
                new PIDController(0, 0, 0);

        leftFlywheelFeedforwardController = new SimpleMotorFeedforward(0, 0);
        rightFlywheelFeedforwardController = new SimpleMotorFeedforward(0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        io.setVoltages(
                Volts.of(leftFlywheelFeedbackController.calculate(inputs.leftFlywheelVelocity.in(RotationsPerSecond), setpoint.in(RotationsPerSecond)) + leftFlywheelFeedforwardController.calculate(setpoint.in(RotationsPerSecond))),
                Volts.of(rightFlywheelFeedbackController.calculate(inputs.rightFlywheelVelocity.in(RotationsPerSecond), setpoint.in(RotationsPerSecond)) + rightFlywheelFeedforwardController.calculate(setpoint.in(RotationsPerSecond))
                ));
    }

    public void setSpeed(Measure<Velocity<Angle>> speed) {
        setpoint = speed;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return inputs.leftFlywheelVelocity;
    }
}
