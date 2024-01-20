// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint =
            new Rotation2d(); // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = 0.0; // Setpoint for closed loop control, null for open loop

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        if (Robot.isReal()) {
            driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
            driveFeedback = new PIDController(0.0, 0.0, 0.0);
            turnFeedback = new PIDController(0.0, 0.0, 0.0);
        } else {
            driveFeedforward = new SimpleMotorFeedforward(0, 2.75);
            driveFeedback = new PIDController(28.5, 0.0, 0.0);
            turnFeedback = new PIDController(35.0, 0.0, 0.0);
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedback.disableContinuousInput();

        SmartDashboard.putData("Drive PID " + index, driveFeedback);
        SmartDashboard.putData("Turn PID " + index, turnFeedback);

        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);

        // Run closed loop turn control
        if (Math.abs(angleSetpoint.getDegrees() - inputs.turnPosition.getDegrees()) > 1) {
            io.setTurnVoltage(
                    Volts.of(
                            turnFeedback.calculate(
                                    getAngle().getRadians(), angleSetpoint.getRadians())));
        } else {
            io.setTurnVoltage(Volts.of(0));
        }

        // Run closed loop drive control
        if (speedSetpoint > 0.1 || speedSetpoint < -0.1) {
            io.setDriveVoltage(
                    Volts.of(
                            driveFeedback.calculate(getVelocityMetersPerSec(), speedSetpoint)
                                    + driveFeedforward.calculate(speedSetpoint)));
        } else {
            // System.out.println("Zeroing voltage");
            io.setDriveVoltage(Volts.of(0));
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState setState(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedSetpoint = SwerveModuleState.optimize(state, getAngle());

        // TODO: Reactivate optimization after it works without it

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = state.angle;
        speedSetpoint = state.speedMetersPerSecond;

        return optimizedSetpoint;
    }

    public void setVoltages(Measure<Voltage> driveVolts, Measure<Voltage> turnVolts) {
        io.setDriveVoltage(driveVolts);
        io.setTurnVoltage(turnVolts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(Volts.of(0));
        io.setDriveVoltage(Volts.of(0));

        // Disable closed loop control for turn and drive
        angleSetpoint = getAngle();
        speedSetpoint = 0.0;
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePosition.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity.in(MetersPerSecond);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }
}
