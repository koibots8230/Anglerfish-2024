// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.AutoConstants;
import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {

    private Pose2d endGoal;
    private Rotation2d angle;

    private boolean leaving = false;
    private int note = 100;

    private final ProfiledPIDController angleAlignmentController;

    public DriveDistance(Pose2d goal) {
        leaving = false;
        note = 100;
        System.out.println(goal);
        endGoal = goal;

        addRequirements(Swerve.get());

        angleAlignmentController =
                new ProfiledPIDController(
                        3.5,
                        0,
                        0,
                        new Constraints(
                                RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                4 * Math.PI),
                        0.02);
    }

    public DriveDistance(boolean leaving) {
        this.leaving = true;
        endGoal = new Pose2d();
        angle = new Rotation2d();

        angleAlignmentController =
                new ProfiledPIDController(
                        3.5,
                        0,
                        0,
                        new Constraints(
                                RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                4 * Math.PI),
                        0.02);
    }

    public DriveDistance(int note) {
        this.note = note;
        endGoal = new Pose2d();
        angle = new Rotation2d();

        angleAlignmentController =
                new ProfiledPIDController(
                        3,
                        0,
                        0,
                        new Constraints(
                                RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                4 * Math.PI),
                        0.02);
    }

    @Override
    public void initialize() {
        if (leaving) {
            endGoal =
                    new Pose2d(
                            Swerve.get().getEstimatedPose().getX() + 1.27,
                            Swerve.get().getEstimatedPose().getY(),
                            new Rotation2d());
        } else if (note != 100) {
            endGoal =
                    new Pose2d(
                            AutoConstants.NOTE_POSITIONS[note],
                            new Rotation2d(
                                             Swerve.get().getEstimatedPose().getX()
                                                     - AutoConstants.NOTE_POSITIONS[note].getX(),
                                             Swerve.get().getEstimatedPose().getY()
                                                    - AutoConstants.NOTE_POSITIONS[note].getY()).minus(new Rotation2d(Math.PI)));
        }

        angle =
                new Rotation2d(
                        endGoal.getX() - Swerve.get().getEstimatedPose().getX(),
                        endGoal.getY() - Swerve.get().getEstimatedPose().getY());

        System.out.println(endGoal);
    }

    @Override
    public void execute() {
        double angularVelocity =
                angleAlignmentController.calculate(
                        Swerve.get().getGyroAngle().getRadians(),
                        endGoal.getRotation().getRadians());
                    
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        angle.getCos() * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        angle.getSin() * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        angularVelocity * RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                        Swerve.get().getEstimatedPose().getRotation());

        Swerve.get().driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        return ((Swerve.get().getEstimatedPose().getX()
                        <= endGoal.getX() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getX()
                        >= endGoal.getX() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        <= endGoal.getY() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        >= endGoal.getY() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters)));
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.get()
                .setModuleStates(
                        new SwerveModuleState[] {
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                        });
    }
}
