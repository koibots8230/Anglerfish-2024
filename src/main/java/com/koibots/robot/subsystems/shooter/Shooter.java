// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.SensorConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private Measure<Velocity<Angle>> topSetpoint = RPM.of(0);
    private Measure<Velocity<Angle>> bottomSetpoint = RPM.of(0);

    private final BangBangController topBangBang;
    private final BangBangController bottomBangBang;

    private final SimpleMotorFeedforward topFeedforward;
    private final SimpleMotorFeedforward bottomFeedForward;
    
    private int topInverted = 1;
    private int bottomInverted = 1;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();

        topBangBang = new BangBangController(10);
        bottomBangBang = new BangBangController(10);

        topFeedforward =
                new SimpleMotorFeedforward(
                        ControlConstants.SHOOTER_FEEEDFORWARD.ks,
                        ControlConstants.SHOOTER_FEEEDFORWARD.kv);
        bottomFeedForward =
                new SimpleMotorFeedforward(
                        ControlConstants.SHOOTER_FEEEDFORWARD.ks,
                        ControlConstants.SHOOTER_FEEEDFORWARD.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.topSetpoint = topSetpoint.in(RPM);
        inputs.bottomSetpoint = bottomSetpoint.in(RPM);
        Logger.processInputs("Subsystems/Shooter", inputs);

        io.setVoltages(
                Volts.of(
                        Math.min(
                                Math.max(
                                        3 * topBangBang.calculate(
                                                    inputs.topVelocity,
                                                    topSetpoint.in(RPM))
                                        + (0.9 * topFeedforward.calculate(
                                                    topSetpoint.in(RPM))),
                                        -12.0), 12)),
                Volts.of(
                        Math.min(
                                Math.max(
                                        3 * bottomBangBang.calculate(
                                                    inputs.bottomVelocity,
                                                    bottomSetpoint.in(RPM))
                                                + (0.9 * bottomFeedForward.calculate(
                                                    bottomSetpoint.in(RPM))),
                                        -12.0), 12)));
    }

    public void setVelocity(
            Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
        topSetpoint = topSpeed.times(topInverted);
        bottomSetpoint = bottomSpeed.times(bottomInverted);

        System.out.println("Top Speed - " + topSpeed.in(RPM));
        System.out.println("Bottom Speed - " + bottomSpeed.in(RPM));
        System.out.println("Top Setpoint - " + topSetpoint.in(RPM));
        System.out.println("Bottom Setpoint - " + bottomSetpoint.in(RPM));
    }

    public void setVoltage(Measure<Voltage> volts) {
        io.setVoltages(volts, volts);
    }

    public boolean atSetpoint() {
        return RPM.of(inputs.topVelocity).isNear(topSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR)
                && RPM.of(inputs.bottomVelocity)
                        .isNear(bottomSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR);
    }

    public List<Measure<Current>> getCurrent() {
        return Arrays.asList(inputs.topCurrent, inputs.bottomCurrent);
    }

    public void invert() {
        topInverted *= -1;
        bottomInverted *= -1;
    }
}
