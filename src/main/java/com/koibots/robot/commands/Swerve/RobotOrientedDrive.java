package com.koibots.robot.commands.Swerve;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotOrientedDrive extends Command {
    DoubleSupplier vxSupplier;
    DoubleSupplier vySupplier;
    DoubleSupplier vThetaSupplier;
    DoubleSupplier angleSupplier;
    BooleanSupplier crossSupplier;

    PIDController angleAlignmentController;

    SlewRateLimiter vxLimiter;
    SlewRateLimiter vyLimiter;
    SlewRateLimiter vThetaLimiter;

    public RobotOrientedDrive(
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
                new PIDController(
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kP,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kI,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kD);
        angleAlignmentController.setTolerance(0.005);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);

        vxLimiter = new SlewRateLimiter(1.75);
        vyLimiter = new SlewRateLimiter(1.75);
        vThetaLimiter = new SlewRateLimiter(6);

        addRequirements(Swerve.get());
    }

    @Override
    public void execute() {
        if (crossSupplier.getAsBoolean()) {
            Swerve.get().setCross();
        } else {
            double vxInput = vxLimiter.calculate(vxSupplier.getAsDouble());
            double vyInput = vyLimiter.calculate(vySupplier.getAsDouble());
            double angularVelocity;
            
            if (angleSupplier.getAsDouble() != -1) {
                angularVelocity =
                        angleAlignmentController.calculate(
                                Swerve.get().getEstimatedPose().getRotation().getRadians(),
                                Math.toRadians(angleSupplier.getAsDouble()));
                System.out.println(
                        "Alignment Setpoint " + Math.toRadians(angleSupplier.getAsDouble()));

                Logger.recordOutput(
                        "Angle Alignment Setpoint",
                        Math.toRadians(angleSupplier.getAsDouble()) - Math.PI);
                Logger.recordOutput("Angle Alignement Output", angularVelocity);
            } else {
                angularVelocity =
                        MathUtil.applyDeadband(
                                vThetaLimiter.calculate(vThetaSupplier.getAsDouble()),
                                ControlConstants.DEADBAND);
            }

            ChassisSpeeds speeds = new ChassisSpeeds(
                vxInput * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                vyInput * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                angularVelocity * RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
            );
        }
    }
}
