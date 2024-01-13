package com.koibots.robot.commands.teleop;

import com.koibots.robot.Constants;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class SwerveCommand extends Command {
    DoubleSupplier vxSupplier;
    DoubleSupplier vySupplier;
    DoubleSupplier vThetaSupplier;
    DoubleSupplier angleSupplier;
    BooleanSupplier crossSupplier;
    Function<Double, Double> scalingAlgorithm;
    double previousTimestamp;

    ProfiledPIDController angleAlignmentController;

    public SwerveCommand(
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            DoubleSupplier vThetaSupplier,
            DoubleSupplier angleSupplier,
            BooleanSupplier crossSupplier,
            Function<Double, Double> scalingAlgorithm) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.vThetaSupplier = vThetaSupplier;
        this.angleSupplier = angleSupplier;
        this.crossSupplier = crossSupplier;
        this.scalingAlgorithm = scalingAlgorithm;

        angleAlignmentController = new ProfiledPIDController(
                0.2,
                0,
                0,
                new Constraints(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 4 * Math.PI),
                0.02);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        previousTimestamp = Logger.getRealTimestamp();
    }

    @Override
    public void execute() {
        if (!this.crossSupplier.getAsBoolean()) { // Normal Field Oriented Drive
            double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(vxSupplier.getAsDouble(), vySupplier.getAsDouble()),
                    Constants.DEADBAND,
                    1);

            Rotation2d linearDirection = new Rotation2d(vxSupplier.getAsDouble(), vySupplier.getAsDouble());

            double angularVelocity;

            if (angleSupplier.getAsDouble() != -1) {
                angularVelocity = angleAlignmentController.calculate(
                        Swerve.get().getEstimatedPose().getRotation().getRadians(),
                        -Math.toRadians(angleSupplier.getAsDouble()));
            } else {
                angularVelocity = MathUtil.applyDeadband(vThetaSupplier.getAsDouble(),
                        Constants.DEADBAND);
            }

            // Apply Scaling
            linearMagnitude = scalingAlgorithm.apply(linearMagnitude);
            // angularVelocity = scalingFunction.apply(angularVelocity);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearMagnitude * linearDirection.getCos()
                            * Constants.MAX_LINEAR_SPEED_METERS_PER_SECOND,
                    linearMagnitude * linearDirection.getSin()
                            * Constants.MAX_LINEAR_SPEED_METERS_PER_SECOND,
                    angularVelocity * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Swerve.get().getEstimatedPose().getRotation());

            double periodSeconds = Logger.getRealTimestamp() - previousTimestamp;

            ChassisSpeeds.discretize(speeds, periodSeconds);

            SwerveModuleState[] targetModuleStates = Constants.SWERVE_KINEMATICS
                    .toSwerveModuleStates(speeds);

            desaturateWheelSpeeds(targetModuleStates, Constants.MAX_LINEAR_SPEED_METERS_PER_SECOND);

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

            Logger.recordOutput("Module Setpoints", targetModuleStates);

            Swerve.get().setModuleStates(targetModuleStates);
        } else { // Set Cross
            System.out.println("Setting Cross");

            var targetModuleStates = new SwerveModuleState[] {
                        new SwerveModuleState(0,
                                Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0,
                                Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0,
                                Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0,
                                Rotation2d.fromDegrees(45))
                };

                
                Logger.recordOutput("Module Setpoints", targetModuleStates);
            Swerve.get().setModuleStates(targetModuleStates);
                    
        }
    }
}