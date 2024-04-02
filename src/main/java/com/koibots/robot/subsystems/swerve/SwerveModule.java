// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int index;

    private Rotation2d angleSetpoint =
            new Rotation2d(); // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = 0.0; // Setpoint for closed loop control, null for open loop

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        inputs.setpoint = angleSetpoint.getRadians();
        Logger.processInputs("Subsystems/Drive/Module" + index, inputs);
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState setState(SwerveModuleState state) {

        // if (MathUtil.inputModulus(getAngle().minus(state.angle).getRadians(), -Math.PI, Math.PI)
        // >= Math.toRadians(90)) { // True if error is greater than 110 degrees TODO: Didn't work
        // Optimize state based on current angle
        var optimizedSetpoint = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedSetpoint.angle;
        io.setTurnPosition(angleSetpoint);
        speedSetpoint = optimizedSetpoint.speedMetersPerSecond;
        io.setDriveVelocity(MetersPerSecond.of(speedSetpoint * Math.abs(Math.cos(angleSetpoint.getRadians() - inputs.turnPosition.getRadians()))));
        // Cosine scaling makes it so it won't drive (much) while module is turning

        return optimizedSetpoint;
    }


    /** Disables all outputs to motors. */
    public void stop() {
        // Disable closed loop control for turn and drive
        angleSetpoint = getAngle();
        speedSetpoint = 0.0;
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
