// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;

import com.koibots.robot.Constants.ShooterConstants;
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
    
    private Measure<Velocity<Angle>> setpoint = RotationsPerSecond.of(0);

    PIDController leftFeedback;
    PIDController rightFeedback;
    SimpleMotorFeedforward leftFeedforward;
    SimpleMotorFeedforward rightFeedforward;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();

        leftFeedback = new PIDController(ShooterConstants.kP, 0, 0);
        rightFeedback = new PIDController(ShooterConstants.kP, 0, 0);

        leftFeedforward =
                new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
        rightFeedforward =
                new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystems/Shooter", inputs);

        // io.setVoltages(
        //         Volts.of(
        //                 Math.max(Math.min(
        //                         (leftFeedback.calculate(
        //                                         inputs.leftVelocity.in(RotationsPerSecond),
        //                                         setpoint.in(RotationsPerSecond))
        //                         + leftFeedforward.calculate(
        //                                         setpoint.in(RotationsPerSecond)))
        //                         * (12.0 / 5676.0), 12.0), -12.0)),
        //         Volts.of(
        //                 Math.max(Math.min(
        //                         (rightFeedback.calculate(
        //                                         inputs.rightVelocity.in(RotationsPerSecond),
        //                                         setpoint.in(RotationsPerSecond))
        //                         + rightFeedforward.calculate(
        //                                         setpoint.in(RotationsPerSecond)))
        //                         * (12.0 / 5676.0), 12.0), -12.0))
        // );

        SmartDashboard.putData("Shooter/Left PID", leftFeedback);
        SmartDashboard.putData("Shooter/Right PID", rightFeedback);
    }

    public void setVelocity(Measure<Velocity<Angle>> speed) {
        setpoint = speed;
    }

    public void setVoltage(Measure<Voltage> volts) {
        io.setVoltages(volts, volts);
    }

    public boolean atSetpoint() {
        return inputs.leftVelocity.in(RPM)
                        >= setpoint.minus(ShooterConstants.ALLOWED_ERROR).in(RPM)
                && inputs.leftVelocity.in(RPM)
                        <= setpoint.plus(ShooterConstants.ALLOWED_ERROR).in(RPM)
                && inputs.rightVelocity.in(RPM)
                        >= setpoint.minus(ShooterConstants.ALLOWED_ERROR).in(RPM)
                && inputs.rightVelocity.in(RPM)
                        <= setpoint.plus(ShooterConstants.ALLOWED_ERROR).in(RPM);
    }

    public List<Measure<Current>> getCurrent() {
        return Arrays.asList(inputs.leftCurrent, inputs.rightCurrent);
    }
}
