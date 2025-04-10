package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class SwerveModule implements Logged {
  private final SparkMax turnMotor;
  private final SparkMaxConfig turnConfig;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController turnPID;
  private final SimpleMotorFeedforward turnFF;

  private final TrapezoidProfile turnProfile;
  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

  private final SparkFlex driveMotor;
  private final RelativeEncoder driveEncoder;

  private final SparkFlexConfig driveConfig;

  private final SparkClosedLoopController drivePID;

  private final Rotation2d angleOffset;

  @Log private Rotation2d turnSetpoint;
  @Log private Rotation2d turnPosition;
  @Log private double turnVelocity;

  @Log private double turnCurrent;
  @Log private double turnVoltage;

  @Log private double driveSetpoint;
  @Log private double drivePosition;
  private double simStoredPosition;
  @Log private double driveVelocity;

  @Log private double driveCurrent;
  @Log private double driveVoltage;

  public SwerveModule(int turnID, int driveID, Rotation2d angleOffset) {
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    turnMotor.clearFaults();

    turnConfig = new SparkMaxConfig();

    turnConfig.smartCurrentLimit(SwerveConstants.TURN_MOTOR_CONFIG.currentLimit);
    turnConfig.inverted(SwerveConstants.TURN_MOTOR_CONFIG.inverted);
    turnConfig.idleMode(SwerveConstants.TURN_MOTOR_CONFIG.idleMode);
    turnConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    turnMotor.setCANTimeout((int) RobotConstants.CAN_TIMEOUT.in(Milliseconds));

    turnConfig.absoluteEncoder.positionConversionFactor(
        SwerveConstants.TURN_ENCODER_POSITION_FACTOR.in(Radians));
      turnConfig.absoluteEncoder.velocityConversionFactor(
        SwerveConstants.TURN_ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

    turnConfig.absoluteEncoder.inverted(true);

    turnConfig.closedLoop.p(SwerveConstants.TURN_PID_GAINS.kp);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    turnConfig.absoluteEncoder.inverted(true);

    turnConfig.signals.absoluteEncoderPositionPeriodMs(1);

    REVLibError error = turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println(error == REVLibError.kOk);

    turnEncoder = turnMotor.getAbsoluteEncoder();

    turnPID = turnMotor.getClosedLoopController();

    turnFF =
        new SimpleMotorFeedforward(
            SwerveConstants.TURN_FF_GAINS.ks, SwerveConstants.TURN_FF_GAINS.kv);

    turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_VELOCITY.in(RadiansPerSecond),
                SwerveConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecond.per(Second))));

    turnGoalState = new TrapezoidProfile.State(0, 0);
    turnSetpointState = new TrapezoidProfile.State(0, 0);

    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);

    driveMotor.clearFaults();

    driveConfig = new SparkFlexConfig();

    driveConfig.smartCurrentLimit(SwerveConstants.DRIVE_MOTOR_CONFIG.currentLimit);
    driveConfig.inverted(SwerveConstants.DRIVE_MOTOR_CONFIG.inverted);
    driveConfig.idleMode(SwerveConstants.DRIVE_MOTOR_CONFIG.idleMode);
    driveConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    driveMotor.setCANTimeout((int) RobotConstants.CAN_TIMEOUT.in(Milliseconds));


    driveConfig.encoder.positionConversionFactor(
        SwerveConstants.DRIVE_ENCODER_POSITION_FACTOR.in(Meters));
    driveConfig.encoder.velocityConversionFactor(
        SwerveConstants.DRIVE_ENCODER_VELOCITY_FACTOR.in(MetersPerSecond));
    driveConfig.encoder.uvwAverageDepth(2);
    driveConfig.encoder.uvwMeasurementPeriod(8);

    driveConfig.closedLoop.p(SwerveConstants.DRIVE_PID_GAINS.kp);
    driveConfig.closedLoop.velocityFF(SwerveConstants.DRIVE_FF_GAINS.kv);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    drivePID = driveMotor.getClosedLoopController();

    driveEncoder = driveMotor.getEncoder();

    turnSetpoint = Rotation2d.fromRadians(0);
    turnPosition = Rotation2d.fromRadians(0);

    driveSetpoint = 0;
    driveVelocity = 0;

    this.angleOffset = angleOffset;
  }

  public void setState(SwerveModuleState state) {
    // SwerveModuleState optimizedState = SwerveModuleState.optimize(state, turnPosition);
    SwerveModuleState optimizedState = state;
    optimizedState.speedMetersPerSecond =
        optimizedState.speedMetersPerSecond * optimizedState.angle.minus(turnPosition).getCos();

    driveSetpoint = optimizedState.speedMetersPerSecond;

    turnGoalState =
        new TrapezoidProfile.State(optimizedState.angle.getRadians() + angleOffset.getRadians(), 0);

    turnSetpointState = turnProfile.calculate(0.02, turnSetpointState, turnGoalState);

    turnPID.setReference(
        optimizedState.angle.getRadians() + angleOffset.getRadians(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turnFF.calculate(turnSetpointState.velocity));

    drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
  }

  public void periodic() {
    turnSetpoint = Rotation2d.fromRadians(turnGoalState.position);

    turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - angleOffset.getRadians());
    turnVelocity = turnEncoder.getVelocity();
    System.out.println(turnPosition);
    turnCurrent = turnMotor.getOutputCurrent();
    turnVoltage = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();

    drivePosition = driveEncoder.getPosition();
    driveVelocity = driveEncoder.getVelocity();

    driveCurrent = driveMotor.getOutputCurrent();
    driveVoltage = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
  }

  public void simulationPeriodic() {
    turnPosition = Rotation2d.fromRadians(turnSetpointState.position);
    driveVelocity = driveSetpoint;
    simStoredPosition += driveVelocity * 0.02;
    drivePosition = simStoredPosition;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveVelocity, turnPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivePosition, turnPosition);
  }
}
