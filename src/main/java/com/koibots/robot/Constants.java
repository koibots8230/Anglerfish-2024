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
    public static class PlopperPivotConstants {
        public static final int MOTOR_PORT = 10;
        public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI);

        public static final PIDConstantsIO FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 0, 0, 0);

        public static final Measure<Angle> AMP_POSITION = Degrees.of(100);
        public static final Measure<Angle> LOAD_POSITION = Degrees.of(0);

        public static final Measure<Angle> ALLOWED_ERROR = Degrees.of(5);
    }

    public static class PlopperConstants {
        public static final int MOTOR_PORT = 16;
        public static final int SWITCH_PORT = 0;

        public static final PIDConstantsIO FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 0, 0, 0);

        public static final Measure<Velocity<Angle>> LOAD_SPEED = RPM.of(600).times(10);
        public static final Measure<Velocity<Angle>> PLOP_SPEED = RPM.of(400).times(10);
    }

    public static final double DEADBAND = 0.025;

    public static class DriveConstants {
        public static final Measure<Distance> WHEEL_RADIUS = Inches.of(1.5);
        private static final Measure<Distance> ROBOT_WIDTH = Inches.of(21.375);
        private static final Measure<Distance> ROBOT_LENGTH = Inches.of(21.375);

        public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4);
        public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
                RadiansPerSecond.of(2 * PI);
        public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCELERATION =
                MetersPerSecondPerSecond.of(4);
        public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
                RadiansPerSecond.of(Math.PI).per(Second);

        public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
                new PIDConstantsIO(0.4, 0, 0, 28.5, 0, 0);
        public static final PIDConstantsIO TURN_PID_CONSTANTS =
                new PIDConstantsIO(1.9, 0, 0, 35, 0, 0);
        public static final FeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 2, 0, 2.75);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(
                                ROBOT_LENGTH.divide(2), ROBOT_WIDTH.divide(2)), // Front Left
                        new Translation2d(
                                ROBOT_LENGTH.divide(2), ROBOT_WIDTH.divide(-2)), // Front Right
                        new Translation2d(
                                ROBOT_LENGTH.divide(-2), ROBOT_WIDTH.divide(2)), // Back Left
                        new Translation2d(
                                ROBOT_LENGTH.divide(-2), ROBOT_WIDTH.divide(-2)) // Back Right
                        );

        public static final int FRONT_LEFT_DRIVE_ID = 7;
        public static final int FRONT_LEFT_TURN_ID = 8;
        public static final int FRONT_RIGHT_DRIVE_ID = 2;
        public static final int FRONT_RIGHT_TURN_ID = 3;
        public static final int BACK_LEFT_DRIVE_ID = 6;
        public static final int BACK_LEFT_TURN_ID = 1;
        public static final int BACK_RIGHT_DRIVE_ID = 4;
        public static final int BACK_RIGHT_TURN_ID = 5;

        // TODO: make sure this correct for competition bot
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

        public static final Measure<Distance> REPLANNING_ERROR_THRESHOLD = Meters.of(1);
        public static final Measure<Distance> REPLANNING_ERROR_SPIKE_THRESHOLD = Meters.of(1);

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(
                        DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        DriveConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond),
                        DriveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                        DriveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecond.per(Second)));

        public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIG =
                new HolonomicPathFollowerConfig(
                        DRIVE_PID_CONSTANTS,
                        TURN_PID_CONSTANTS,
                        MAX_MODULE_SPEED.in(MetersPerSecond),
                        Math.sqrt(
                                Math.pow(ROBOT_LENGTH.in(Meters), 2)
                                        + Math.pow(ROBOT_WIDTH.in(Meters), 2)),
                        new ReplanningConfig(
                                false,
                                true,
                                REPLANNING_ERROR_THRESHOLD.in(Meters),
                                REPLANNING_ERROR_SPIKE_THRESHOLD.in(Meters)));

        public static final List<Pose2d> CLIMB_POSITIONS =
                Arrays.asList(new Pose2d(), new Pose2d(), new Pose2d());

        public static final Pose2d AMP_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_AMP = new Translation2d(2, 2);

        public static final List<Measure<Distance>> SHOOT_DISTANCES_METERS =
                Arrays.asList(Meters.of(4.5));
        public static final Pose2d SPEAKER_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_SHOOT = new Translation2d(2, 2);

        public static final Translation2d ALLOWED_DISTANCE_FROM_NOTE = new Translation2d(2, 2);
    }

    public static class ShooterConstants {
        public static final int RIGHT_MOTOR_PORT = 14;
        public static final int LEFT_MOTOR_PORT = 12;

        public static final double kP = 0;

        public static final double kS = 0;
        public static final double kV = 0;

        public static final Measure<Velocity<Angle>> SPEED = RPM.of(5000);

        public static final Measure<Velocity<Angle>> ALLOWED_ERROR = RPM.of(20);

        public static final Measure<Current> CURRENT_ON_SHOOT = Amps.of(60);
    }

    public static class ElevatorConstants {

        // ===================================Motors/Encoders===================================
        public static final int LEFT_MOTOR_PORT = 13;
        public static final int RIGHT_MOTOR_PORT = 11;

        public static final Measure<Distance> DISTANCE_PER_REVOLUTION = Inches.of(1.751 * Math.PI);

        // ===================================Linear System===================================

        public static final double GEAR_RATIO = 25;
        public static final Measure<Mass> MASS = Pounds.of(9.006);
        public static final Measure<Distance> DRUM_RADIUS = Inches.of(0.8755);

        public static final LinearSystem<N2, N1, N1> LINEAR_SYS =
                LinearSystemId.createElevatorSystem(
                        DCMotor.getNEO(2), MASS.in(Kilograms), DRUM_RADIUS.in(Meters), GEAR_RATIO);

        // ===================================Profile===================================

        public static final Measure<Velocity<Distance>> MAX_SPEED = InchesPerSecond.of(18);
        public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
                MetersPerSecondPerSecond.of(Units.inchesToMeters(1));

        // ===================================Kalman Filter===================================

        public static final Measure<Distance> STDEV_DISTANCE = Inches.of(5); // TODO: Get
        public static final Measure<Velocity<Distance>> STDEV_VELOCITY =
                InchesPerSecond.of(10); // TODO: Get
        public static final double ENCODER_STDEV = 0.01; // TODO: Get

        public static final KalmanFilter<N2, N1, N1> KALMAN_FILTER =
                new KalmanFilter<>(
                        Nat.N2(),
                        Nat.N1(),
                        LINEAR_SYS,
                        VecBuilder.fill(
                                STDEV_DISTANCE.in(Meters), STDEV_VELOCITY.in(MetersPerSecond)),
                        VecBuilder.fill(ENCODER_STDEV),
                        0.020);

        // ===================================LQR===================================

        public static final Measure<Distance> POSITION_ERROR_TOLERANCE =
                Inches.of(0.01); // TODO: Get
        public static final Measure<Velocity<Distance>> VELOCITY_ERROR_TOLERANCE =
                InchesPerSecond.of(.1); // TODO: Get
        public static final double VOLTAGE_TOLERANCE = 12;

        public static final LinearQuadraticRegulator<N2, N1, N1> LQR =
                new LinearQuadraticRegulator<>(
                        LINEAR_SYS,
                        VecBuilder.fill(
                                POSITION_ERROR_TOLERANCE.in(Meters),
                                VELOCITY_ERROR_TOLERANCE.in(MetersPerSecond)),
                        VecBuilder.fill(VOLTAGE_TOLERANCE),
                        0.020);

        // =================================== Positions ===================================

        public static final Measure<Distance> AMP_POSITION = Inches.of(0); // TODO: Get
        public static final Measure<Distance> LOAD_POSITION = Inches.of(0); // TODO: Get
        public static final Measure<Distance> SHOOT_POSITION = Inches.of(0); // TODO: Get

        public static final Measure<Distance> ALLOWED_ERROR = Meters.of(0.005);
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

    public static class IntakeConstants {
        public static final int MOTOR_PORT = 15;

        public static final PIDConstantsIO FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.01, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 11, 0, 0);

        public static final Wheel WHEELS = new Wheel(Inches.of(1.5));

        public static final Measure<Velocity<Angle>> TARGET_VELOCITY = RPM.of(400);
    }

    public static class IndexerConstants {
        public static final int MOTOR = 9;

        public static final PIDConstantsIO FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.009, 0, 0, 0, 0, 0);
        public static final FeedforwardConstantsIO FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 10, 0, 0);

        public static final Measure<Velocity<Angle>> LOAD_SPEED = RPM.of(500).times(10);
        public static final Measure<Velocity<Angle>> SHOOT_SPEED = RPM.of(1000).times(10);
    }

    public static class AutoConstants {

        public static final PIDConstantsIO VX_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final PIDConstantsIO VY_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
        public static final PIDConstantsIO VTHETA_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
    }
}
