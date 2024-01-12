package com.koibots.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = new double[] { Math.abs(driveSim.getCurrentDrawAmps()) };

        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = new double[] { Math.abs(turnSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}