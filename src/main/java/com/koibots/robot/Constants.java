package com.koibots.robot;

import static java.lang.StrictMath.PI;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Constants {

    public static final int SHOOTER_PIVOT_MOTOR = 0;
    public static final double SHOOTER_PIVOT_MOTOR_SPEED = 0;
    public static final double SHOOTER_PIVOT_MOTOR_REVERSE_SPEED = 0;
    public static final double SHOOTER_PIVOT_ENCODER_POSITION_FACTOR = (2 * Math.PI);
    public static final double AMP_SHOOTER_RADIANS = Math.PI / 2;
    public static final double SPEAKER_SHOOTER_RADIANS = Math.PI * 2 / 3;
    public static final double LOAD_SHOOTER_RADIANS = 0;

    public static final int MAX_LINEAR_SPEED_METERS_PER_SECOND = 4; // Meters per Second
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2 * PI; // Radians per Second

    public static final double DEADBAND = 0.02;

    private static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(30);
    private static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(30);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Front Left
            new Translation2d(ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2), // Front Right
            new Translation2d(-ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Back Left
            new Translation2d(-ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2) // Back Right
    );

    public static final boolean SET_REPLAY = false;
    public static final Field2d FIELD = new Field2d();

    public static class DriveConstants {
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

        public static double DRIVE_GEAR_RATIO = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

        public static double DRIVE_KP = 0;
        public static double DRIVE_KI = 0;
        public static double DRIVE_KD = 0;

        public static double TURN_KP = 0;
        public static double TURN_KI = 0;
        public static double TURN_KD = 0;

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    }
}