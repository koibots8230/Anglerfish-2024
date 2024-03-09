// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static edu.wpi.first.units.Units.*;
import static java.lang.StrictMath.PI;

import com.koibots.lib.geometry.Wheel;
import com.koibots.lib.util.FeedforwardConstantsIO;
import com.koibots.lib.util.PIDConstantsIO;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import java.util.Arrays;
import java.util.List;

public class Constants {
    public static final double DEADBAND = 0.025;

    public static class DeviceIDs {
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_TURN = 8;
        public static final int BACK_RIGHT_DRIVE = 2;
        public static final int BACK_RIGHT_TURN = 3;
        public static final int FRONT_LEFT_DRIVE = 6;
        public static final int FRONT_LEFT_TURN = 1;
        public static final int INDEXER = 13;
        public static final int SHOOTER_TOP = 14;
        public static final int SHOOTER_BOTTOM = 12;
        public static final int INTAKE = 10;
    }

    public static class ControlConstants {
        public static final double DRIVE_TURN_KS = 0.125;
        public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
                new PIDConstantsIO(0.0015, 0, 0, 28.5, 0, 0);
        public static final PIDConstantsIO TURN_PID_CONSTANTS =
                new PIDConstantsIO(1.7, 0, 0, 35, 0, 0);
        public static final FeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 2.6, 0, 2.75);

        public static final int ENCODER_SAMPLES_PER_AVERAGE = 40;
        public static final PIDConstantsIO SHOOTER_FEEDBACK = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO SHOOTER_FEEEDFORWARD =
                new FeedforwardConstantsIO(0, 0.0275, 0, 0);

        public static final PIDConstantsIO INTAKE_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.01, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO INTAKE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 10.25, 0, 0);

        public static final PIDConstantsIO INDEXER_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.009, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO INDEXER_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 10, 0, 0);

        public static final PIDConstantsIO VX_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final PIDConstantsIO VY_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final PIDConstantsIO VTHETA_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Front Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)), // Front Right
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Back Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)) // Back Right
                        );

        public static final Measure<Distance> REPLANNING_ERROR_THRESHOLD = Meters.of(1);
        public static final Measure<Distance> REPLANNING_ERROR_SPIKE_THRESHOLD = Meters.of(1);

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(
                        RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        RobotConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond),
                        RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                        RobotConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecond.per(Second)));

        public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIG =
                new HolonomicPathFollowerConfig(
                        DRIVE_PID_CONSTANTS,
                        TURN_PID_CONSTANTS,
                        RobotConstants.MAX_MODULE_SPEED.in(MetersPerSecond),
                        Math.sqrt(
                                Math.pow(RobotConstants.ROBOT_LENGTH.in(Meters), 2)
                                        + Math.pow(RobotConstants.ROBOT_WIDTH.in(Meters), 2)),
                        new ReplanningConfig(
                                false,
                                true,
                                REPLANNING_ERROR_THRESHOLD.in(Meters),
                                REPLANNING_ERROR_SPIKE_THRESHOLD.in(Meters)));
    }

    public static final class SetpointConstants {
        public static final Measure<Velocity<Angle>> INTAKE_TARGET_VELOCITY = RPM.of(500);

        public static final Measure<Velocity<Angle>> INDEXER_LOAD_SPEED = RPM.of(100).times(10);
        public static final Measure<Velocity<Angle>> INDEXER_SHOOT_SPEED = RPM.of(110).times(10);

        public static final List<List<Measure<Velocity<Angle>>>> SHOOTER_SPEEDS =
                Arrays.asList(
                    Arrays.asList(RPM.of(5000), RPM.of(5000))
                );
        public static final Measure<Velocity<Angle>> SHOOTER_ALLOWED_ERROR = RPM.of(10);
    }

    public static final class RobotConstants {
        public static final Measure<Distance> WHEEL_RADIUS = Inches.of(1.5);
        private static final Measure<Distance> ROBOT_WIDTH = Inches.of(21.375);
        private static final Measure<Distance> ROBOT_LENGTH = Inches.of(21.375);

        public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4.125);
        public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
                RadiansPerSecond.of(3 * PI);
        public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCELERATION =
                MetersPerSecondPerSecond.of(4);
        public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
                RadiansPerSecond.of(4 * Math.PI).per(Second);

        private static final int DRIVING_PINION_TEETH = 13;
        public static final double DRIVE_GEAR_RATIO =
                (45.0 * 22) / (DRIVING_PINION_TEETH * 15); // 5.07692307692
        public static final double TURN_GEAR_RATIO = (62.0 / 14) * 12; // 53.1428571429

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double DRIVING_ENCODER_POSITION_FACTOR =
                (WHEEL_RADIUS.in(Meters) * 2 * Math.PI) / DRIVE_GEAR_RATIO; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR =
                ((WHEEL_RADIUS.in(Meters) * 2 * Math.PI) / DRIVE_GEAR_RATIO)
                        / 60.0; // meters per second
        public static final Measure<Velocity<Distance>> MAX_MODULE_SPEED = MetersPerSecond.of(4);
        public static final Wheel WHEELS = new Wheel(Inches.of(1.5));
    }

    public static class DriveConstants {
        public static final Pose2d AMP_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_AMP = new Translation2d(2, 2);

        public static final List<Measure<Distance>> SHOOT_DISTANCES_METERS =
                Arrays.asList(Meters.of(4.5));
        public static final Pose2d SPEAKER_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_SHOOT = new Translation2d(2, 2);

        public static final Translation2d ALLOWED_DISTANCE_FROM_NOTE = new Translation2d(2, 2);
    }

    public static class VisionConstants {
        public static final Pose2d[] CAMERA_POSITIONS = {
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(270))),
        }; // x is left, y is forward, counterclockwise on rotation

        public static final String[][] TOPIC_NAMES = {
            {"Cam1Tvec", "Cam1Rvec", "Cam1Ids"},
            {"Cam2Tvec", "Cam2Rvec", "Cam2Ids"},
            {"Cam3Tvec", "Cam3Rvec", "Cam3Ids"},
            {"Cam4Tvec", "Cam4Rvec", "Cam4Ids"}
        };
        public static final double[] VECTOR_DEFAULT_VALUE = {0, 0, 0};
        public static final int ID_DEFAULT_VALUE = 0;

        public static final Measure<Distance> MAX_MEASUREMENT_DIFFERENCE = Meters.of(1);
    }
}
