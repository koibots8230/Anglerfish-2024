// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.robot.Constants.DriveConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;
    private Rotation2d chassisAngularOffset;

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


        if (turnId == DriveConstants.FRONT_LEFT_TURN_ID) {
            chassisAngularOffset = Rotation2d.fromRadians((3 * Math.PI) / 2);
        } else if (turnId == DriveConstants.FRONT_RIGHT_TURN_ID) {
                chassisAngularOffset = new Rotation2d(Math.PI);
        } else if (turnId == DriveConstants.BACK_LEFT_TURN_ID) {
                chassisAngularOffset = Rotation2d.fromRadians(0);
        } else if (turnId == DriveConstants.BACK_RIGHT_TURN_ID) {
                chassisAngularOffset = Rotation2d.fromRadians(Math.PI / 2); // Rotation2d.fromDegrees((3 * Math.PI) / 2);
        }


        turnSparkMax.setInverted(false);
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
        inputs.drivePosition = Meters.of(driveEncoder.getPosition());
        inputs.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
        inputs.driveAppliedVoltage =
                Volts.of(driveSparkMax.getBusVoltage()).times(driveSparkMax.getAppliedOutput());
        inputs.driveCurrent = Amps.of(driveSparkMax.getOutputCurrent());

        inputs.turnPosition =
                Rotation2d.fromRadians(turnEncoder.getPosition()).plus(chassisAngularOffset).minus(Rotation2d.fromRadians(Math.PI));

        inputs.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
        inputs.turnAppliedVoltage =
                Volts.of(turnSparkMax.getBusVoltage()).times(turnSparkMax.getAppliedOutput());
        inputs.turnCurrent = Amps.of(turnSparkMax.getOutputCurrent());
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> voltage) {
        driveSparkMax.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> voltage) {

        turnSparkMax.setVoltage(voltage.in(Volts));
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
