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

import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
    private double lastPositionMeters = 0.0; // Used for delta calculation

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Robot.getMode()) {
            case REAL:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0, 0);
                driveFeedback = new PIDController(0.01, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeedback.enableContinuousInput(0, 2 * Math.PI);

        SmartDashboard.putData("Drive PID " + index , driveFeedback);
        SmartDashboard.putData("Turn PID " + index, turnFeedback);

        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Run closed loop turn control
        if (angleSetpoint != null && Math.abs(angleSetpoint.getRadians() - inputs.turnPosition.getRadians()) < 0.01) {
            io.setTurnVoltage(
                    turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // Scale velocity based on turn error
                //
                // When the error is 90, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;

                if (velocityRadPerSec > 0.01 || velocityRadPerSec < -0.01) {
                    io.setDriveVoltage(
                        driveFeedforward.calculate(velocityRadPerSec)
                                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
                } else {
                    io.setDriveVoltage(0);
                }
            }
        } else {
            io.setTurnVoltage(0);
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized
     * state.
     */
    public SwerveModuleState setState(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public void setTestVoltages() {
        io.setDriveVoltage(3);
        io.setTurnVoltage(3);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module position delta since the last call to this method. */
    public SwerveModulePosition getPositionDelta() {
        var delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
        lastPositionMeters = getPositionMeters();
        return delta;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }
}