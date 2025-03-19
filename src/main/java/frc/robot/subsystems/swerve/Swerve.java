package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.RobotConstants;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Swerve extends SubsystemBase {

  private final SwerveModule[] modules;

  private final Pigeon2 gyro;
  private Rotation2d simStoredAngle;

  private final SwerveDrivePoseEstimator odometry;

  private final PIDController anglePID;

  private double[] setpointStates;
   private double[] measuredStates;

   Rotation2d gyroAngle;
   Pose2d odometryPose;

  public Swerve(boolean isReal) {
    modules = new SwerveModule[4];

    modules[0] =
        new SwerveModule(
            DeviceIDs.FRONT_LEFT_TURN,
            DeviceIDs.FRONT_LEFT_DRIVE,
            isReal ? new Rotation2d((3 * Math.PI) / 2.0) : new Rotation2d());
    modules[1] =
        new SwerveModule(
            DeviceIDs.FRONT_RIGHT_TURN,
            DeviceIDs.FRONT_RIGHT_DRIVE,
            isReal ? new Rotation2d(Math.PI) : new Rotation2d());
    modules[2] =
        new SwerveModule(
            DeviceIDs.BACK_LEFT_TURN,
            DeviceIDs.BACK_LEFT_DRIVE,
            isReal ? new Rotation2d(0) : new Rotation2d());
    modules[3] =
        new SwerveModule(
            DeviceIDs.BACK_RIGHT_TURN,
            DeviceIDs.BACK_RIGHT_DRIVE,
            isReal ? new Rotation2d(Math.PI / 2.0) : new Rotation2d());

    gyro = new Pigeon2(DeviceIDs.PIGEON);

    measuredStates = new double[8];
    setpointStates = new double[8];

    odometry =
        new SwerveDrivePoseEstimator(
            ControlConstants.SWERVE_KINEMATICS,
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
            ControlConstants.DRIVE_PID_CONSTANTS.kP,
            ControlConstants.DRIVE_PID_CONSTANTS.kI,
            ControlConstants.DRIVE_PID_CONSTANTS.kD);
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
            ControlConstants.SWERVE_KINEMATICS.toChassisSpeeds(
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

    SwerveModuleState[] states = ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond));

    this.setModuleStates(states);
  }

  private double[] applyJoystickScaling(double xInput, double yInput, double thetaInput) {
    double linearMagnitude =
        Math.hypot(
            MathUtil.applyDeadband(xInput, ControlConstants.DEADBAND),
            MathUtil.applyDeadband(yInput, ControlConstants.DEADBAND));

    Rotation2d linearDirection = new Rotation2d(xInput, yInput);

    double angluarVelocity = MathUtil.applyDeadband(thetaInput, ControlConstants.DEADBAND);

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
      double thetaInput,
      BooleanSupplier hasNote,
      Supplier<List<Translation2d>> notePositions) {
    double[] joysticks = applyJoystickScaling(xInput, yInput, thetaInput);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MetersPerSecond.of(
                joysticks[0] * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            MetersPerSecond.of(
                joysticks[1] * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            RadiansPerSecond.of(
                joysticks[2] * RobotConstants.MAX_ANGULAR_VELOCITY),
            gyroAngle);

    return speeds;
  }

  private ChassisSpeeds joystickToRobotRelativePointAtTarget(double xInput, double yInput) {
    double[] joysticks = applyJoystickScaling(xInput, yInput, 0);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MetersPerSecond.of(joysticks[0] * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            MetersPerSecond.of(joysticks[1] * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)),
            RadiansPerSecond.of(
                anglePID.calculate(
                    this.getOdometryPose().getRotation().getRadians())),
            gyroAngle);

    return speeds;
  }

  private ChassisSpeeds getRelativeSpeeds() {
    return ControlConstants.SWERVE_KINEMATICS.toChassisSpeeds(
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
      DoubleSupplier thetaInput,
      BooleanSupplier hasNote,
      Supplier<List<Translation2d>> notePositions) {
    return Commands.run(
        () ->
            driveRobotRelative(
                joystickToRobotRelative(
                    -xInput.getAsDouble(),
                    -yInput.getAsDouble(),
                    -thetaInput.getAsDouble(),
                    hasNote,
                    notePositions)),
        this);
  }

  public Command fieldOrientedWhilePointingCommand(DoubleSupplier xInput, DoubleSupplier yInput) {
    return Commands.run(
        () ->
            driveRobotRelative(
                joystickToRobotRelativePointAtTarget(-xInput.getAsDouble(), -yInput.getAsDouble())),
        this);
  }
}
