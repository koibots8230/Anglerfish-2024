// Copyright 2024 (c) FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim driveSim =
            new DCMotorSim(DCMotor.getNEO(1), DriveConstants.DRIVE_GEAR_RATIO, 0.025);
    private final DCMotorSim turnSim =
            new DCMotorSim(DCMotor.getNEO(1), DriveConstants.TURN_GEAR_RATIO, 0.004);

    private Measure<Voltage> driveAppliedVolts = Volts.of(0);
    private Measure<Voltage> turnAppliedVolts = Volts.of(0);

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePosition = DriveConstants.WHEEL_RADIUS.times(driveSim.getAngularPositionRad());
        inputs.driveVelocity =
                DriveConstants.WHEEL_RADIUS
                        .times(driveSim.getAngularVelocityRadPerSec())
                        .per(Second);
        inputs.driveAppliedVoltage = driveAppliedVolts;
        inputs.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

        inputs.turnPosition =
                new Rotation2d(turnSim.getAngularPositionRad() % (2 * Math.PI))
                        .minus(Rotation2d.fromRadians(Math.PI));
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVoltage = turnAppliedVolts;
        inputs.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> voltage) {
        driveAppliedVolts = voltage;
        driveSim.setInputVoltage(driveAppliedVolts.in(Volts));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> voltage) {
        turnAppliedVolts = Volts.of(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0));
        turnSim.setInputVoltage(turnAppliedVolts.in(Volts));
    }
}
