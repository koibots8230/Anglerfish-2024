package com.koibots.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Constants.DriveConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {

    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final boolean isTurnMotorInverted = true;

    public SwerveModuleIOSparkMax(int driveId, int turnId) {

        driveSparkMax = new CANSparkMax(driveId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(turnId, MotorType.kBrushless);

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveEncoder.setPositionConversionFactor(DriveConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        turnEncoder.setInverted(true);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnEncoder.setPositionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        if (turnSparkMax.getDeviceId() == DriveConstants.FRONT_LEFT_TURN_ID || turnSparkMax.getDeviceId() == DriveConstants.BACK_RIGHT_TURN_ID) {
            turnEncoder.setZeroOffset(Math.PI / 2);
        }

        turnSparkMax.setInverted(isTurnMotorInverted);
        driveSparkMax.setSmartCurrentLimit(40);
        turnSparkMax.setSmartCurrentLimit(20);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition())
                / DriveConstants.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { driveSparkMax.getOutputCurrent() };

        inputs.turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition());
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity());
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };
    }

    @Override
    public void setDriveVoltage(double volts) {
        Logger.recordOutput("Drive Voltage " + driveSparkMax.getDeviceId(), volts);
        MathUtil.clamp(volts, -12, 12);
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        Logger.recordOutput("Turn Voltage " + turnSparkMax.getDeviceId(), volts);
        MathUtil.clamp(volts, -11.4, 12);
        turnSparkMax.setVoltage(volts);;
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {

        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}