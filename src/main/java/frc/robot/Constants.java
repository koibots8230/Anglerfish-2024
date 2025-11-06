package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConfig;
import frc.lib.util.PIDGains;
import frc.lib.util.Wheel;

public class Constants {
  public static class RobotConstants {
    public static final Distance WIDTH = Inches.of(26.0);
    public static final Distance LENGTH = Inches.of(26.0);

    public static final Distance BUMPER_WIDTH = Inches.of(39.250);
    public static final Distance BUMPER_LENGTH = Inches.of(39.250);

    public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);

    public static final Time CAN_TIMEOUT = Milliseconds.of(20);

    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  public static class SwerveConstants {

    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(0.75);

    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI);

    public static final AngularVelocity MAX_TURN_VELOCITY =
        RadiansPerSecond.of(20 * Math.PI);
    public static final AngularAcceleration MAX_TURN_ACCELERATION =
        RadiansPerSecond.per(Second).of(Math.PI * 30);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(-2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(-2)));

    public static final PIDGains DRIVE_PID_GAINS = new PIDGains.Builder().kp(0.0005).build();
    public static final FeedforwardGains DRIVE_FF_GAINS =
        new FeedforwardGains.Builder().kv(0.2).build();

    public static final PIDGains TURN_PID_GAINS = new PIDGains.Builder().kp(0.4).build();
    public static final FeedforwardGains TURN_FF_GAINS =
        new FeedforwardGains.Builder().ks(0).kv(0.4).build();

    public static final PIDGains ANGLE_PID_GAINS = new PIDGains.Builder().kp(0).kd(0).build();

    public static final MotorConfig DRIVE_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(60).build();
    public static final MotorConfig TURN_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(30).build();

    public static final Rotation2d[] ANGLE_OFFSETS =
        new Rotation2d[] {
            Rotation2d.fromRadians((Math.PI) / 2.0),
            Rotation2d.fromRadians(Math.PI),
            Rotation2d.fromRadians(0),
            Rotation2d.fromRadians((3 * Math.PI) / 2.0)
        };

    private static final int DRIVING_PINION_TEETH = 13;
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);

    public static final Wheel WHEELS = new Wheel(Inches.of(1.5));

    public static final Angle TURN_ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    public static final AngularVelocity TURN_ENCODER_VELOCITY_FACTOR =
        RadiansPerSecond.of((2 * Math.PI) / 60.0);

    public static final Distance DRIVE_ENCODER_POSITION_FACTOR =
        Inches.of((1.5 * 2 * Math.PI) / DRIVE_GEAR_RATIO);
    public static final LinearVelocity DRIVE_ENCODER_VELOCITY_FACTOR =
        MetersPerSecond.of(((WHEELS.radius.in(Meters) * 2 * Math.PI) / DRIVE_GEAR_RATIO) / 60.0);

    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_TURN_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_TURN_ID = 3;
    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_LEFT_TURN_ID = 5;
    public static final int BACK_RIGHT_DRIVE_ID = 8;
    public static final int BACK_RIGHT_TURN_ID = 7;

    public static final int GYRO_ID = 10;

    // ====================AUTO====================
    public static final Distance REPLANNING_ERROR_THRESHOLD = Meters.of(1);
    public static final Distance REPLANNING_ERROR_SPIKE_THRESHOLD = Meters.of(1);

    public static final PIDGains AUTO_TRANSLATION_PID =
        new PIDGains.Builder().kp(0).ki(0).kd(0).build();
    public static final PIDGains AUTO_ROTATION_PID =
        new PIDGains.Builder().kp(0).ki(0).kd(0).build();
  }
}
