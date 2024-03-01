// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final SimpleMotorFeedforward turnFeedforward;
    private Rotation2d angleSetpoint =
            new Rotation2d(); // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = 0.0; // Setpoint for closed loop control, null for open loop

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveFeedforward =
                new SimpleMotorFeedforward(
                        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.ks,
                        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.kv);
        driveFeedback =
                new PIDController(
                        ControlConstants.DRIVE_PID_CONSTANTS.kP,
                        ControlConstants.DRIVE_PID_CONSTANTS.kI,
                        ControlConstants.DRIVE_PID_CONSTANTS.kD
                        );
        turnFeedback =
                new PIDController(
                        ControlConstants.TURN_PID_CONSTANTS.kP,
                        ControlConstants.TURN_PID_CONSTANTS.kI,
                        ControlConstants.TURN_PID_CONSTANTS.kD);
        turnFeedforward = new SimpleMotorFeedforward(0.175, 0.0025);
        //turnFeedforward = new SimpleMotorFeedforward(0.0, 0.5)

        //turnFeedforward.            
        turnFeedback.enableContinuousInput(0, 2 * Math.PI);

        driveFeedback.disableContinuousInput();

        SmartDashboard.putData("Swerve/Drive PID " + index, driveFeedback);
        SmartDashboard.putData("Swerve/Turn PID " + index, turnFeedback);

        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        inputs.setpoint = angleSetpoint.getRadians();
        Logger.processInputs("Subsystems/Drive/Module" + index, inputs);

        // Run closed loop turn control
        if (Math.abs(angleSetpoint.getRadians() - inputs.turnPosition.getRadians()) > 0.0001) {
            io.setTurnVoltage(
                    Volts.of(
                        turnFeedforward.calculate(angleSetpoint.getRadians()) +
                            turnFeedback.calculate(
                                    getAngle().getRadians(), angleSetpoint.getRadians())));
        } else {
            io.setTurnVoltage(Volts.of(0));
        }

        // Run closed loop drive control
        if (speedSetpoint > 0.1 || speedSetpoint < -0.1) {
            // io.setDriveVoltage(
            //         Volts.of(
            //                 driveFeedback.calculate(getVelocityMetersPerSec(), speedSetpoint)
            //                         + driveFeedforward.calculate(speedSetpoint)));
        } else {
            // System.out.println("Zeroing voltage");
            io.setDriveVoltage(Volts.of(0));
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState setState(SwerveModuleState state) {
        // Optimize state based on current angle
        var optimizedSetpoint = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedSetpoint.angle;
        speedSetpoint = optimizedSetpoint.speedMetersPerSecond;

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
