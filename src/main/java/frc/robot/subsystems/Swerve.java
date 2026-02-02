package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ReefAlignState;
import frc.lib.util.VisionMeasurement;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {
  private Pose2d estimatedPosition;
  private Pose2d trajectoryToFollow = new Pose2d();
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;

  private double speedScalar;

  private final PIDController xController =
      new PIDController(AutoConstants.X_CONTROLLER.kp, 0.0, 0.0);
  private final PIDController yController =
      new PIDController(AutoConstants.Y_CONTROLLER.kp, 0.0, 0.0);
  private final PIDController headingController =
      new PIDController(AutoConstants.HEADING_CONTROLLER.kp, 0.0, 0.0);

  @Logged
  public class Modules {
    final SwerveModule frontLeft;
    final SwerveModule frontRight;
    final SwerveModule backLeft;
    final SwerveModule backRight;

    public Modules() {
      frontLeft =
          new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
      frontRight =
          new SwerveModule(
              SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
      backLeft =
          new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
      backRight =
          new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);
    }
  }

  private final Modules modules;

  @NotLogged private final SwerveDrivePoseEstimator odometry;

  @NotLogged private final PIDController anglePID;

  private SwerveModuleState[] measuredStates;

  private boolean isBlue;

  private ReefAlignState reefAlignState;
  private Pose2d alignTarget;

  public Swerve() {

    modules = new Modules();

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    estimatedPosition = new Pose2d();
    gyroAngle = gyro.getRotation2d();
    simHeading = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, this.getModulePostitions(), estimatedPosition);

    setpointStates = new SwerveModuleState[4];
    measuredStates = new SwerveModuleState[4];

    // try{
    //   config = RobotConfig.fromGUISettings();
    // }
    // catch (Exception e){
    //   e.printStackTrace();
    // }

    // AutoBuilder.configure(this::getEstimatedPosition, this::setOdometry, this::getChassisSpeeds,
    // this::driveRobotRelative, SwerveConstants.pathPlannerFF, config, () -> setColour(), this);

    anglePID =
        new PIDController(
            AlignConstants.ANGLE_PID.kp, AlignConstants.ANGLE_PID.ki, AlignConstants.ANGLE_PID.kd);

    anglePID.enableContinuousInput(-Math.PI, Math.PI);

    reefAlignState = ReefAlignState.disabled;
    alignTarget = new Pose2d();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    speedScalar = 1;
  }

  public boolean getIsBlue() {
    return isBlue;
  }

  public void setOdometry(Pose2d pose) {
    simHeading = pose.getRotation();
    odometry.resetPose(pose);
  }

  public void setIsBlue(boolean colour) {
    isBlue = colour;
  }

  @Override
  public void periodic() {

    modules.frontLeft.periodic();
    modules.frontRight.periodic();
    modules.backLeft.periodic();
    modules.backRight.periodic();

    estimatedPosition =
        odometry.update(
            isBlue ? gyroAngle : gyroAngle.minus(new Rotation2d(Math.PI)),
            this.getModulePostitions());

    gyroAngle = gyro.getRotation2d();

    measuredStates[0] = modules.frontLeft.getModuleState();
    measuredStates[1] = modules.frontRight.getModuleState();
    measuredStates[2] = modules.backLeft.getModuleState();
    measuredStates[3] = modules.backRight.getModuleState();

    if (reefAlignState == null) {
      reefAlignState = ReefAlignState.disabled;
    }
  }

  @Override
  public void simulationPeriodic() {
    simHeading =
        simHeading.plus(
            new Rotation2d(
                getChassisSpeeds().omegaRadiansPerSecond
                    * RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds)));
    gyroAngle = simHeading;

    modules.frontLeft.simulationPeriodic();
    modules.frontRight.simulationPeriodic();
    modules.backLeft.simulationPeriodic();
    modules.backRight.simulationPeriodic();
  }

  public Pose2d getEstimatedPosition() {
    return odometry.getEstimatedPosition();
  }

  public void addVisionMeasurement(VisionMeasurement measurement) {
    odometry.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        VecBuilder.fill(
            measurement.translationStdev, measurement.translationStdev, measurement.rotationStdev));
  }

  // ===================== Gyro ===================== \\

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  // ===================== Module Positions ===================== \\

  public SwerveModulePosition[] getModulePostitions() {
    return new SwerveModulePosition[] {
      modules.frontLeft.getPosition(),
      modules.frontRight.getPosition(),
      modules.backLeft.getPosition(),
      modules.backRight.getPosition()
    };
  }

  // ===================== Chassis Speeds ===================== \\

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(measuredStates);
  }

  // ===================== Reef Align State ===================== \\

  private void setReefAlignState(ReefAlignState state) {
    this.reefAlignState = state;
  }

  // ===================== Pose Helpers ===================== \\
  private Distance distanceToPose(Pose2d pose) {
    return Meters.of(
        Math.hypot(
            pose.getX() - this.getEstimatedPosition().getX(),
            pose.getY() - this.getEstimatedPosition().getY()));
  }

  private Pose2d getReefSide() {
    Pose2d closest = new Pose2d(999, 999, Rotation2d.kZero);
    for (Pose2d side : AlignConstants.REEF_SIDES) {
      side =
          isBlue
              ? side
              : new Pose2d(
                  side.getX() + AlignConstants.RED_REEF_OFFSET.in(Meters),
                  side.getY(),
                  side.getRotation());
      closest = distanceToPose(closest).lt(distanceToPose(side)) ? closest : side;
    }
    return closest;
  }

  private Translation2d getEffectorOffset(Pose2d side) {
    return new Translation2d(
        (Math.sin(2 * side.getRotation().getRadians())
            * AlignConstants.EFFECTOR_OFFSET.in(Meters)
            * (side.getRotation().getRadians() >= Math.PI / 3.0
                    && side.getRotation().getRadians() <= (2 * Math.PI) / 3.0
                ? 1
                : -1)),
        (Math.cos(2 * side.getRotation().getRadians())
            * AlignConstants.EFFECTOR_OFFSET.in(Meters)
            * (side.getRotation().getRadians() >= Math.PI / 3.0
                    && side.getRotation().getRadians() <= (2 * Math.PI) / 3.0
                ? 1
                : -1)));
  }

  private Translation2d getPoleTranslation(Pose2d side, boolean rightPole) {
    return new Translation2d(
        side.getX()
            + (Math.sin(2 * side.getRotation().getRadians())
                * AlignConstants.POLE_SPACING.in(Meters)
                * (rightPole ? 1 : -1)
                * (isBlue ? 1 : -1)
                * (side.getRotation().getRadians() == Math.PI ? -1 : 1)),
        side.getY()
            + (Math.cos(2 * side.getRotation().getRadians())
                * AlignConstants.POLE_SPACING.in(Meters)
                * (rightPole ? 1 : -1)
                * (isBlue ? 1 : -1)
                * (side.getRotation().getRadians() == Math.PI ? -1 : 1)));
  }

  // ===================== Align Assist ===================== \\

  private Pose2d getAssistVelocity(
      Translation2d targetPose, Rotation2d targetAngle, double xInput, double yInput) {
    Translation2d[] points =
        new Translation2d[] {
          this.getEstimatedPosition().getTranslation(),
          new Translation2d(
              this.getEstimatedPosition().getX() + xInput,
              this.getEstimatedPosition().getY() + yInput)
        };

    Rotation2d angleToTarget =
        Rotation2d.fromRadians(
            Math.atan2(
                this.getEstimatedPosition().getY() - targetPose.getY(),
                this.getEstimatedPosition().getX() - targetPose.getX()));

    Distance distancePerpToVel =
        Meters.of( // Looks complicated, but just the "Line from two points" from this
            // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
            Math.abs(
                    ((points[1].getY() - points[0].getY()) * targetPose.getX())
                        - ((points[1].getX() - points[0].getX()) * targetPose.getY())
                        + (points[1].getX() * points[0].getY())
                        - (points[1].getY() * points[0].getX()))
                / Math.sqrt(
                    Math.pow((points[1].getY() - points[0].getY()), 2)
                        + Math.pow((points[1].getX() - points[0].getX()), 2)));

    LinearVelocity assistVelocity =
        MetersPerSecond.of(
            Math.cbrt(distancePerpToVel.in(Meters)) * AlignConstants.TRANSLATE_PID.kp);

    return new Pose2d(
        assistVelocity.in(MetersPerSecond) * angleToTarget.getCos(),
        assistVelocity.in(MetersPerSecond) * angleToTarget.getSin(),
        new Rotation2d(
            anglePID.calculate(
                gyroAngle.getRadians(), targetAngle.getRadians() + (isBlue ? 0 : Math.PI))));
  }

  private Pose2d reefAlignAssist(double xInput, double yInput, double omega) {
    xInput = isBlue ? xInput : -xInput;
    yInput = isBlue ? yInput : -yInput;
    omega = isBlue ? omega : -omega;

    Pose2d closestSide = getReefSide();

    Pose2d pose;
    switch (reefAlignState) {
      case rightSide:
        pose =
            new Pose2d(
                getPoleTranslation(closestSide, true).plus(getEffectorOffset(closestSide)),
                closestSide.getRotation());
        break;
      case leftSide:
        pose =
            new Pose2d(
                getPoleTranslation(closestSide, false).plus(getEffectorOffset(closestSide)),
                closestSide.getRotation());
        break;
      case disabled:
        alignTarget = Pose2d.kZero;
        return Pose2d.kZero;
      default:
        alignTarget = Pose2d.kZero;
        return Pose2d.kZero;
    }

    if (distanceToPose(pose).gte(AlignConstants.MIN_DISTANCE)) {
      alignTarget = Pose2d.kZero;
      return Pose2d.kZero;
    }

    Rotation2d movementDirection = Rotation2d.fromRadians(Math.atan2(yInput, xInput));

    Rotation2d angleRange =
        Rotation2d.fromRadians(
            AlignConstants.DIRECTION_ANGLE_RANGE_CLOSE.in(Radians)
                - (distanceToPose(pose).in(Meters) * AlignConstants.DISTANCE_ANGLE_RANGE_SCALAR));

    if (Math.abs(
            (movementDirection.getRadians()
                        - pose.getRotation().unaryMinus().getRadians()
                        + Math.PI)
                    % (2 * Math.PI)
                - Math.PI)
        > angleRange.getRadians()) {
      alignTarget = Pose2d.kZero;
      return Pose2d.kZero;
    }

    alignTarget = pose;

    return getAssistVelocity(pose.getTranslation(), pose.getRotation(), xInput, yInput);
  }

  // ===================== Teleop Driving ===================== \\

  private void driveFieldRelativeScaler(double x, double y, double omega) {
    double linearMagnitude = Math.pow(Math.hypot(x, y), SwerveConstants.TRANSLATION_SCALAR);

    Rotation2d direction = new Rotation2d(y, x);

    y =
        linearMagnitude
            * -direction.getCos()
            * SwerveConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    x =
        linearMagnitude
            * -direction.getSin()
            * SwerveConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond)
            * speedScalar;

    omega =
        Math.pow(omega, SwerveConstants.ROTATION_SCALAR)
            * SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
            * speedScalar;

    Pose2d assist = reefAlignAssist(-x, y, omega);

    driveFieldRelative(
        MetersPerSecond.of(
            MathUtil.applyDeadband(
                x + (assist.getX() * Math.sqrt(linearMagnitude) * (isBlue ? -1 : 1)),
                SwerveConstants.DEADBAND)),
        MetersPerSecond.of(
            MathUtil.applyDeadband(
                y + (assist.getY() * Math.sqrt(linearMagnitude) * (isBlue ? -1 : 1)),
                SwerveConstants.DEADBAND)),
        RadiansPerSecond.of(
            MathUtil.applyDeadband(
                -omega + (assist.getRotation().getRadians()), SwerveConstants.DEADBAND)));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, SwerveConstants.MAX_LINEAR_VELOCITY);

    modules.frontLeft.setState(setpointStates[0]);
    modules.frontRight.setState(setpointStates[1]);
    modules.backLeft.setState(setpointStates[2]);
    modules.backRight.setState(setpointStates[3]);
  }

  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond), y.in(MetersPerSecond), omega.in(RadiansPerSecond), gyroAngle);

    driveFieldRelative(speeds);
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {
    driveRobotRelative(speeds);
  }

  

  private void toggleSpeed() {
    speedScalar = (speedScalar == 1) ? 0.5 : 1;
  }

  // ===================== Commands ===================== \\

  public Command driveFieldRelativeCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            driveFieldRelativeScaler(
                MathUtil.applyDeadband(x.getAsDouble(), SwerveConstants.DEADBAND),
                MathUtil.applyDeadband(y.getAsDouble(), SwerveConstants.DEADBAND),
                MathUtil.applyDeadband(omega.getAsDouble(), SwerveConstants.DEADBAND)),
        this);
  }

  public Command zeroGyroCommand(boolean colour) {
    return Commands.runOnce(() -> zeroGyro(), this);
  }

  public Command setReefAlignStateCommand(ReefAlignState state) {
    return Commands.runOnce(() -> this.setReefAlignState(state));
  }

  public Command toggleSpeedCommand() {
    return Commands.runOnce(() -> this.toggleSpeed());
  }
}
