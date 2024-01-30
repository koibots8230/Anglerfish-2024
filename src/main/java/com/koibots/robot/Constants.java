package com.koibots.robot;

import static java.lang.StrictMath.PI;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Constants {

    public static final int MAX_LINEAR_SPEED_METERS_PER_SECOND = 4; // Meters per Second
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2 * PI; // Radians per Second

    public static final double DEADBAND = 0.02;

    private static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(21.375);
    private static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(21.375);

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

    public static class ElevatorConstants {

        // ===================================Motors/Encoders===================================
        public static final int LEFT_MOTOR_PORT = 1;
        public static final int RIGHT_MOTOR_PORT = 2;

        public static final Measure<Distance> DISTANCE_PER_REVOLUTION = Inches.of(0); // TODO: Get from cad when it's done

        // ===================================Linear System===================================

        public static final double GEAR_RATIO = 60;
        public static final Measure<Mass> MASS = Pounds.of(0); // TODO: Get from cad when it's done
        public static final Measure<Distance> DRUM_DIAMETER = Inches.of(0); //TODO: Ask cad when they finish

        public static final LinearSystem<N2, N1, N1> LINEAR_SYS = LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(2), MASS.in(Kilograms), DRUM_DIAMETER.in(Meters), GEAR_RATIO);

        // ===================================Profile===================================

        public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(0); // TODO: Get from cad when it's done
        public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION = MetersPerSecondPerSecond.of(0); //TODO: Get from cad when it's done

        public static final TrapezoidProfile PROFILE = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                MAX_SPEED.in(MetersPerSecond),
                MAX_ACCELERATION.in(MetersPerSecondPerSecond))
        );

        // ===================================Kalman Filter===================================

        public static final Measure<Distance> STDEV_DISTANCE = Inches.of(0); // TODO: Get
        public static final Measure<Velocity<Distance>> STDEV_VELOCITY = InchesPerSecond.of(0); //TODO: Get
        public static final double ENCODER_STDEV = 0.001; //TODO: Get

        public static final KalmanFilter<N2, N1, N1> KALMAN_FILTER =
            new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                LINEAR_SYS,
                VecBuilder.fill(STDEV_DISTANCE.in(Meters), STDEV_VELOCITY.in(MetersPerSecond)),
                VecBuilder.fill(ENCODER_STDEV),
                0.020
            );

        // ===================================LQR===================================

        public static final Measure<Distance> POSITION_ERROR_TOLERANCE = Inches.of(0); //TODO: Get
        public static final Measure<Velocity<Distance>> VELOCITY_ERROR_TOLERANCE = InchesPerSecond.of(0); // TODO: Get
        public static final double VOLTAGE_TOLERANCE = 12;

        public static final LinearQuadraticRegulator<N2, N1, N1> LQR =
            new LinearQuadraticRegulator<>(
                LINEAR_SYS,
                VecBuilder.fill(POSITION_ERROR_TOLERANCE.in(Meters), VELOCITY_ERROR_TOLERANCE.in(MetersPerSecond)),
                VecBuilder.fill(VOLTAGE_TOLERANCE),
                0.020
            );

        // ===================================Positions===================================

        public static final Measure<Distance> AMP_POSITION = Inches.of(0); // TODO: Get
        public static final Measure<Distance> HANDOFF_POSITION = Inches.of(0); // TODO: Get
        public static final Measure<Distance> RESTING_POSITION = Inches.of(0); // TODO: Get
    }
}