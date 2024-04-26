// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.SensorConstants;
import com.koibots.robot.Robot;
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

    public Shooter() {
        io = Robot.isReal() ? new ShooterIOSparkMax() : new ShooterIOSim();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.topSetpoint = topSetpoint.in(RPM);
        inputs.bottomSetpoint = bottomSetpoint.in(RPM);
        Logger.processInputs("Subsystems/Shooter", inputs);
    }

    public void setVelocity(
            Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
        io.setVelocity(topSpeed, bottomSpeed);
        topSetpoint = topSpeed;
        bottomSetpoint = bottomSpeed;
        
        System.out.println("Top Speed - " + topSpeed.in(RPM));
        System.out.println("Bottom Speed - " + bottomSpeed.in(RPM));
        System.out.println("Top Setpoint - " + topSetpoint.in(RPM));
        System.out.println("Bottom Setpoint - " + bottomSetpoint.in(RPM));
    }

    public boolean atSetpoint() {
        return RPM.of(inputs.topVelocity).isNear(topSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR)
                && RPM.of(inputs.bottomVelocity)
                        .isNear(bottomSetpoint, SensorConstants.SHOOTER_ALLOWED_ERROR);
    }

    public List<Measure<Current>> getCurrent() {
        return Arrays.asList(inputs.topCurrent, inputs.bottomCurrent);
    }
}
