// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim driveSim =
            new DCMotorSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
    private final DCMotorSim turnSim =
            new DCMotorSim(DCMotor.getNEO(1), RobotConstants.TURN_GEAR_RATIO, 0.004);

    private final PIDController driveFeedback = new PIDController(
        ControlConstants.DRIVE_PID_CONSTANTS.kP,
        ControlConstants.DRIVE_PID_CONSTANTS.kI,
        ControlConstants.DRIVE_PID_CONSTANTS.kD
    );

    private final PIDController turnFeedback = new PIDController(
        ControlConstants.TURN_PID_CONSTANTS.kP,
        ControlConstants.TURN_PID_CONSTANTS.kI,
        ControlConstants.TURN_PID_CONSTANTS.kD
    );

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.ks,
        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.kv
    );
    
    private Measure<Velocity<Distance>> driveSetpoint = MetersPerSecond.of(0);
    private Measure<Angle> turnSetpoint = Radians.of(0);

    public SwerveModuleIOSim() {
        turnFeedback.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePosition =
                RobotConstants.DRIVE_WHEELS.radius.times(driveSim.getAngularPositionRad());
        inputs.driveVelocity =
                RobotConstants.DRIVE_WHEELS
                        .radius
                        .times(driveSim.getAngularVelocityRadPerSec())
                        .per(Second);
        inputs.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

        inputs.turnPosition =
                new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());


        inputs.driveAppliedVoltage = Volts.of(MathUtil.clamp(driveFeedback.calculate(RobotConstants.DRIVE_WHEELS.radius.in(Meters) * driveSim.getAngularVelocityRadPerSec(), driveSetpoint.in(MetersPerSecond)) + driveFeedforward.calculate(driveSetpoint.in(MetersPerSecond)), -12, 12));
        inputs.turnAppliedVoltage = Volts.of(MathUtil.clamp(turnFeedback.calculate(turnSim.getAngularPositionRad(), turnSetpoint.in(Radians)), -12, 12)); 

        driveSim.setInputVoltage(
                inputs.driveAppliedVoltage.in(Volts)
        );
        
        turnSim.setInputVoltage(
                inputs.turnAppliedVoltage.in(Volts)
        );
    }

    @Override
    public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
        System.out.println(velocity);
        driveSetpoint = velocity;
    }
    
    @Override
    public void setTurnPosition(Rotation2d position) {
        turnSetpoint = Radians.of(position.getRadians());

    }
}
