package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase; //:)
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Swerve extends SubsystemBase implements Logged {

  private final SwerveModule[] modules;

  private final Pigeon2 gyro;
  private Rotation2d simStoredAngle;

  private final SwerveDrivePoseEstimator odometry;

  private final PIDController anglePID;

  @Log private double[] setpointStates;
  @Log private double[] measuredStates;

  @Log Rotation2d gyroAngle;
  @Log Pose2d odometryPose;

  public Swerve(boolean isReal) {
    modules = new SwerveModule[4];

    modules[0] =
        new SwerveModule(
            SwerveConstants.FRONT_LEFT_TURN_ID,
            SwerveConstants.FRONT_LEFT_DRIVE_ID,
            isReal ? SwerveConstants.ANGLE_OFFSETS[0] : new Rotation2d());
    modules[1] =
        new SwerveModule(
            SwerveConstants.FRONT_RIGHT_TURN_ID,
            SwerveConstants.FRONT_RIGHT_DRIVE_ID,
            isReal ? SwerveConstants.ANGLE_OFFSETS[1] : new Rotation2d());
    modules[2] =
        new SwerveModule(
            SwerveConstants.BACK_LEFT_TURN_ID,
            SwerveConstants.BACK_LEFT_DRIVE_ID,
            isReal ? SwerveConstants.ANGLE_OFFSETS[2] : new Rotation2d());
    modules[3] =
        new SwerveModule(
            SwerveConstants.BACK_RIGHT_TURN_ID,
            SwerveConstants.BACK_RIGHT_DRIVE_ID,
            isReal ? SwerveConstants.ANGLE_OFFSETS[3] : new Rotation2d());

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    measuredStates = new double[8];
    setpointStates = new double[8];

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS,
            Rotation2d.fromRadians(gyro.getAngle()),
            this.getModulePositions(),
            new Pose2d());

    if (isReal) {
      try (Notifier odometryUpdater =
          new Notifier(
              () -> {
                try {
                  modules[0].periodic();
                  modules[1].periodic();
                  modules[2].periodic();
                  modules[3].periodic();

                  odometry.updateWithTime(
                      Timer.getFPGATimestamp(),
                      (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                          ? gyro.getRotation2d()
                          : gyro.getRotation2d().plus(Rotation2d.fromRadians(Math.PI)),
                      getModulePositions());
                } catch (Exception e) {
                  odometry.updateWithTime(
                      Timer.getFPGATimestamp(), gyro.getRotation2d(), getModulePositions());
                }
              })) {
        odometryUpdater.startPeriodic(1.0 / 200); // Run at 200hz
      }
    }
    anglePID =
        new PIDController(
            SwerveConstants.ANGLE_PID_GAINS.kp,
            SwerveConstants.ANGLE_PID_GAINS.ki,
            SwerveConstants.ANGLE_PID_GAINS.kd);
    anglePID.enableContinuousInput(-Math.PI, Math.PI);

    if (!isReal) {
      simStoredAngle = new Rotation2d();
    }
  }

  @Override
  public void periodic() {
    try {
      odometryPose =
          odometry.update(
              Rotation2d.fromRadians(
                  (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                      ? this.getGyroAngle().getRadians()
                      : this.getGyroAngle().getRadians() + Math.PI),
              getModulePositions());
    } catch (Exception e) {
      odometryPose = odometry.update(Rotation2d.fromRadians(gyro.getAngle()), getModulePositions());
    }

    modules[0].periodic();
    modules[1].periodic();
    modules[2].periodic();
    modules[3].periodic();

    gyroAngle = gyro.getRotation2d();

    measuredStates[0] = modules[0].getState().angle.getRadians();
    measuredStates[1] = modules[0].getState().speedMetersPerSecond;
    measuredStates[2] = modules[1].getState().angle.getRadians();
    measuredStates[3] = modules[1].getState().speedMetersPerSecond;
    measuredStates[4] = modules[2].getState().angle.getRadians();
    measuredStates[5] = modules[2].getState().speedMetersPerSecond;
    measuredStates[6] = modules[3].getState().angle.getRadians();
    measuredStates[7] = modules[3].getState().speedMetersPerSecond;
  }

  @Override
  public void simulationPeriodic() {
    modules[0].simulationPeriodic();
    modules[1].simulationPeriodic();
    modules[2].simulationPeriodic();
    modules[3].simulationPeriodic();

    simStoredAngle =
        Rotation2d.fromRadians(
            SwerveConstants.KINEMATICS.toChassisSpeeds(
                            modules[0].getState(),
                            modules[1].getState(),
                            modules[2].getState(),
                            modules[3].getState())
                        .omegaRadiansPerSecond
                    * 0.02
                + simStoredAngle.getRadians());

    gyroAngle = simStoredAngle;
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond));

    this.setModuleStates(states);
  }

  private double[] applyJoystickScaling(double xInput, double yInput, double thetaInput) {
    double linearMagnitude =
        Math.hypot(
            MathUtil.applyDeadband(xInput, RobotConstants.JOYSTICK_DEADBAND),
            MathUtil.applyDeadband(yInput, RobotConstants.JOYSTICK_DEADBAND));

    Rotation2d linearDirection = new Rotation2d(xInput, yInput);

    double angluarVelocity = MathUtil.applyDeadband(thetaInput, RobotConstants.JOYSTICK_DEADBAND);

    linearMagnitude *= linearMagnitude * Math.signum(linearMagnitude);
    angluarVelocity *= angluarVelocity * angluarVelocity;

    return new double[] {
      linearMagnitude * linearDirection.getCos(),
      linearMagnitude * linearDirection.getSin(),
      angluarVelocity
    };
  }

  private ChassisSpeeds joystickToRobotRelative(
      double xInput,
      double yInput,
      double thetaInput) {
    double[] joysticks = applyJoystickScaling(xInput, yInput, thetaInput);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MetersPerSecond.of(
                joysticks[0] * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            MetersPerSecond.of(
                joysticks[1] * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            RadiansPerSecond.of(
                joysticks[2] * SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)),
            gyroAngle);

    return speeds;
  }

  private ChassisSpeeds getRelativeSpeeds() {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(
        modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }

  private void setModuleStates(SwerveModuleState[] states) {
    setpointStates[0] = states[0].angle.getRadians();
    setpointStates[1] = states[0].speedMetersPerSecond;
    setpointStates[2] = states[1].angle.getRadians();
    setpointStates[3] = states[1].speedMetersPerSecond;
    setpointStates[4] = states[2].angle.getRadians();
    setpointStates[5] = states[2].speedMetersPerSecond;
    setpointStates[6] = states[3].angle.getRadians();
    setpointStates[7] = states[3].speedMetersPerSecond;

    modules[0].setState(states[0]);
    modules[1].setState(states[1]);
    modules[2].setState(states[2]);
    modules[3].setState(states[3]);
  }

  public void addVisionMeasurement(Pose2d measurement, double[] timestampAndStdevs) {
    odometry.addVisionMeasurement(
        measurement,
        timestampAndStdevs[0],
        VecBuilder.fill(timestampAndStdevs[1], timestampAndStdevs[2], timestampAndStdevs[3]));
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  public Pose2d getOdometryPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose2d) {
    odometry.resetPosition(getGyroAngle(), getModulePositions(), getOdometryPose());
  }

  public Rotation2d getGyroAngle() {
    return gyroAngle;
  }

  public Rotation2d getWrappedGyroAngle() {
    return Rotation2d.fromRadians(
        (this.getGyroAngle().getRadians() % (Math.PI * 2.0) + (Math.PI * 2.0)) % (Math.PI * 2.0));
  }

  public Command fieldOrientedCommand(
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      DoubleSupplier thetaInput) {
    return Commands.run(
        () ->
            driveRobotRelative(
                joystickToRobotRelative(
                    -xInput.getAsDouble(),
                    -yInput.getAsDouble(),
                    -thetaInput.getAsDouble())),
        this);
  }
}
