// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;

import com.koibots.robot.Constants.SensorConstants;
import com.koibots.robot.Robot;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private Measure<Velocity<Angle>> topSetpoint = RPM.of(0);
    private Measure<Velocity<Angle>> bottomSetpoint = RPM.of(0);

    private BangBangController topController;
    private BangBangController bottomController;

    private int topInverted = 1;
    private int bottomInverted = 0;

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();

        topController = new BangBangController(10);
        bottomController = new BangBangController(10);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.topSetpoint = topSetpoint;
        inputs.bottomSetpoint = bottomSetpoint;
        Logger.processInputs("Subsystems/Shooter", inputs);

        io.setVoltages(
            Volts.of(-11 * topController.calculate(inputs.topVelocity.in(RPM), topSetpoint.in(RPM))),
            Volts.of(-11 * bottomController.calculate(inputs.bottomVelocity.in(RPM), bottomSetpoint.in(RPM))));
    }

    public void setVelocity(Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
        topSetpoint = topSpeed.times(topInverted);
        bottomSetpoint = bottomSpeed.times(bottomInverted);
    }

    public void setVoltage(Measure<Voltage> volts) {
        io.setVoltages(volts, volts);
    }

    public boolean atSetpoint() {
        return inputs.topVelocity.isNear(topSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR)
                && inputs.bottomVelocity.isNear(bottomSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR);
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
