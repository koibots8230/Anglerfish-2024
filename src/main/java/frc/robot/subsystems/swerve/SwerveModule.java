
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SensorConstants;

public class SwerveModule {
  private final SparkMax turnMotor;
  private final AbsoluteEncoder turnEncoder;

  private SparkMaxConfig turnConfig;

  private final SimpleMotorFeedforward turnFF;

  private final TrapezoidProfile turnProfile;
  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

  private final SparkFlex driveMotor;
  private final RelativeEncoder driveEncoder;

  private SparkFlexConfig driveConfig;

  private final SparkClosedLoopController driveController;

  private final Rotation2d angleOffset;

  private Rotation2d turnSetpoint;
  private Rotation2d turnPosition;
  private double turnVelocity;

  private double turnCurrent;
  private double turnVoltage;

  private double driveSetpoint;
  private double drivePosition;
  private double simStoredPosition;
  private double driveVelocity;

  private double driveCurrent;
  private double driveVoltage;

  public SwerveModule(int turnID, int driveID, Rotation2d angleOffset) {
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    turnMotor.clearFaults();

    turnConfig = new SparkMaxConfig();

    turnConfig.idleMode(IdleMode.kCoast);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ControlConstants.TURN_PID_CONSTANTS.kP, ControlConstants.TURN_PID_CONSTANTS.kI, ControlConstants.TURN_PID_CONSTANTS.kD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);
    turnConfig.smartCurrentLimit(30);
    turnConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    turnMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT);

    turnConfig.absoluteEncoder.positionConversionFactor(
        SensorConstants.TURNING_ENCODER_POSITION_FACTOR);
    turnConfig.absoluteEncoder.velocityConversionFactor(
        SensorConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    turnConfig.absoluteEncoder.inverted(true);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnFF =
        new SimpleMotorFeedforward(
            ControlConstants.TURN_FEEDFORWARD_CONSTANTS.ks, ControlConstants.TURN_FEEDFORWARD_CONSTANTS.kv);

    turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                RobotConstants.MAX_ANGULAR_VELOCITY,
                RobotConstants.MAX_ANGULAR_ACCELERATION));
    
    turnEncoder = turnMotor.getAbsoluteEncoder();

    turnGoalState = new TrapezoidProfile.State(0, 0);
    turnSetpointState = new TrapezoidProfile.State(0, 0);

    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);

    driveMotor.clearFaults();

    driveConfig = new SparkFlexConfig();

    driveConfig.closedLoop.pidf(
        ControlConstants.DRIVE_PID_CONSTANTS.kP,
        ControlConstants.DRIVE_PID_CONSTANTS.kI,
        ControlConstants.DRIVE_PID_CONSTANTS.kD,
        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.kv);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(60);
    driveConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    driveMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT);

    driveConfig.encoder.positionConversionFactor(
        SensorConstants.DRIVING_ENCODER_POSITION_FACTOR.in(Meters));
    driveConfig.encoder.velocityConversionFactor(
        SensorConstants.DRIVING_ENCODER_VELOCITY_FACTOR.in(MetersPerSecond));
    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0);

    driveController = driveMotor.getClosedLoopController();

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

    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
  }

  public void periodic() {
    turnSetpoint = Rotation2d.fromRadians(turnGoalState.position);

    turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - angleOffset.getRadians());
    turnVelocity = turnEncoder.getVelocity();

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
