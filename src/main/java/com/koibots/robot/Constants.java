// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static edu.wpi.first.units.Units.*;
import static java.lang.StrictMath.PI;

import com.koibots.lib.util.PIDConstantsIO;
import com.koibots.lib.util.SimpleMotorFeedforwardConstantsIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class Constants {

    public static final double DEADBAND = 0.025;

    public static class DriveConstants {
        public static final Measure<Distance> WHEEL_RADIUS = Inches.of(1.5);
        private static final Measure<Distance> ROBOT_WIDTH_METERS = Inches.of(21.375);
        private static final Measure<Distance> ROBOT_LENGTH_METERS = Inches.of(21.375);

        public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4);
        public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
                RadiansPerSecond.of(2 * PI);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(
                                ROBOT_LENGTH_METERS.divide(2),
                                ROBOT_WIDTH_METERS.divide(2)), // Front Left
                        new Translation2d(
                                ROBOT_LENGTH_METERS.divide(2),
                                ROBOT_WIDTH_METERS.divide(-2)), // Front Right
                        new Translation2d(
                                ROBOT_LENGTH_METERS.divide(-2),
                                ROBOT_WIDTH_METERS.divide(2)), // Back Left
                        new Translation2d(
                                ROBOT_LENGTH_METERS.divide(-2),
                                ROBOT_WIDTH_METERS.divide(-2)) // Back Right
                        );

        // TODO: make sure this correct for competition bot
        private static final int kDrivingMotorPinionTeeth = 13;
        private static final double kWheelDiameterMeters = Units.inchesToMeters(3);
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static int FRONT_LEFT_DRIVE_ID = 7;
        public static int FRONT_LEFT_TURN_ID = 8;
        public static int FRONT_RIGHT_DRIVE_ID = 2;
        public static int FRONT_RIGHT_TURN_ID = 3;
        public static int BACK_LEFT_DRIVE_ID = 6;
        public static int BACK_LEFT_TURN_ID = 1;
        public static int BACK_RIGHT_DRIVE_ID = 4;
        public static int BACK_RIGHT_TURN_ID = 5;

        public static double DRIVE_GEAR_RATIO =
                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); // 5.07692307692
        public static double TURN_GEAR_RATIO = (62.0 / 14) * 12; // 53.1428571429

        public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
                new PIDConstantsIO(0.4, 0, 0, 28.5, 0, 0);
        public static final PIDConstantsIO TURN_PID_CONSTANTS =
                new PIDConstantsIO(1.9, 0, 0, 35, 0, 0);
        public static final SimpleMotorFeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
                new SimpleMotorFeedforwardConstantsIO(0, 2, 0, 2.75);

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double DRIVING_ENCODER_POSITION_FACTOR =
                (WHEEL_RADIUS.in(Meters) * 2 * Math.PI) / kDrivingMotorReduction; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR =
                ((WHEEL_RADIUS.in(Meters) * 2 * Math.PI) / kDrivingMotorReduction)
                        / 60.0; // meters per second
    }

    public static class VisionConstants {
        public static final Pose2d[] CAMERA_DISTANCES_TO_CENTER_METERS = {
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

        public static final Measure<Distance> FIELD_WIDTH = Inches.of(323.25);
        public static final Measure<Distance> FIELD_LENGTH = Inches.of(651.25);

        public static final Measure<Distance> MAX_MEASUREMENT_DIFFERENCE = Meters.of(1);

        public static final Pose2d[] TAG_POSES_METERS = {
            new Pose2d(1.5, .5, new Rotation2d()), new Pose2d(2, .5, new Rotation2d())
        };

        public static final Pose2d[] CAMERA_POSITIONS = null;
    }

    public static class IntakeConstants {
        public static final int INTAKE_PIVOT_MOTOR_PORT = 9;
        public static final int INTAKE_PIVOT_ENCODER_PORT = 10;
        public static final double INTAKE_PIVOT_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians.
        public static final double INTAKE_PIVOT_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second.
        public static final double INTAKE_PIVOT_ZERO_OFFSET = 0;
        public static final int INTAKE_MOTOR_PORT = -1;
        public static final int INTAKE_PID_P = 0;
        public static final int INTAKE_MOTOR_TARGET_RPM = 0;
        public static final int INTAKE_WHEEL_RADIUS = 0;
        public static final int INTAKE_MINIMUM_RPM = 0;
    }

    public static enum IntakePivotState {
        HANDOFF(90),
        DOWN(0);

        public final double angle;
        IntakePivotState(double angle) {
            this.angle = angle;
        }
    };
}
