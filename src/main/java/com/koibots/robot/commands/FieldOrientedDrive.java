// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FieldOrientedDrive extends Command {
    DoubleSupplier vxSupplier;
    DoubleSupplier vySupplier;
    DoubleSupplier vThetaSupplier;
    DoubleSupplier angleSupplier;
    BooleanSupplier crossSupplier;
    double previousTimestamp;

    ProfiledPIDController angleAlignmentController;

    public FieldOrientedDrive(
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            DoubleSupplier vThetaSupplier,
            DoubleSupplier angleSupplier,
            BooleanSupplier crossSupplier) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.vThetaSupplier = vThetaSupplier;
        this.angleSupplier = angleSupplier;
        this.crossSupplier = crossSupplier;

        angleAlignmentController =
                new ProfiledPIDController(
                        0.19,
                        0,
                        0,
                        new Constraints(
                                DriveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                4 * Math.PI),
                        0.02);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);

        SmartDashboard.putData("Angle Alignment Controller", angleAlignmentController);

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        previousTimestamp = Logger.getRealTimestamp();
    }

    @Override
    public void execute() {
        if (!this.crossSupplier.getAsBoolean()) { // Normal Field Oriented Drive
            double linearMagnitude =
                    MathUtil.applyDeadband(
                            Math.hypot(vxSupplier.getAsDouble(), vySupplier.getAsDouble()),
                            Constants.DEADBAND,
                            1);

            Rotation2d linearDirection =
                    new Rotation2d(vxSupplier.getAsDouble(), vySupplier.getAsDouble());

            double angularVelocity;

            if (angleSupplier.getAsDouble() != -1) {
                angularVelocity =
                        angleAlignmentController.calculate(
                                Swerve.get().getEstimatedPose().getRotation().getRadians(),
                                Math.toRadians(angleSupplier.getAsDouble()) - Math.PI);

                Logger.recordOutput(
                        "Angle Alignement Setpoint",
                        Math.toRadians(angleSupplier.getAsDouble()) - Math.PI);
                Logger.recordOutput("Angle Alignement Output", angularVelocity);
            } else {
                angularVelocity =
                        MathUtil.applyDeadband(vThetaSupplier.getAsDouble(), Constants.DEADBAND);
            }

            // Apply Scaling
            linearMagnitude *= linearMagnitude * Math.signum(linearMagnitude);
            angularVelocity *= angularVelocity * angularVelocity;

            ChassisSpeeds speeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearMagnitude
                                    * linearDirection.getCos()
                                    * DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                            linearMagnitude
                                    * linearDirection.getSin()
                                    * DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                            angularVelocity
                                    * DriveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                            Swerve.get().getEstimatedPose().getRotation());

            double periodSeconds = Logger.getRealTimestamp() - previousTimestamp;

            ChassisSpeeds.discretize(speeds, periodSeconds);

            SwerveModuleState[] targetModuleStates =
                    DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

            desaturateWheelSpeeds(targetModuleStates, DriveConstants.MAX_LINEAR_SPEED);

            if (speeds.vxMetersPerSecond == 0.0
                    && speeds.vyMetersPerSecond == 0.0
                    && speeds.omegaRadiansPerSecond == 0) {
                var currentStates = Swerve.get().getModuleStates();
                targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
                targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
                targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
                targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
            }

            previousTimestamp = Logger.getRealTimestamp();

            Swerve.get().setModuleStates(targetModuleStates);
        } else { // Set Cross
            Swerve.get().setCross();
        }
    }
}
