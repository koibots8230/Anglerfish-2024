package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants;
import com.koibots.robot.Robot;
import com.koibots.robot.Constants.DriveConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    SwerveModule[] swerveModules;
    GyroIO gyro;
    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    SwerveDrivePoseEstimator odometry;

    public Swerve() {
        if(Robot.isReal()) {
            swerveModules = new SwerveModule[] { // FL-FR-BL-BR
                    new SwerveModule(new SwerveModuleIOSparkMax(DriveConstants.FRONT_LEFT_DRIVE_ID,
                            DriveConstants.FRONT_LEFT_TURN_ID), 0),
                    new SwerveModule(new SwerveModuleIOSparkMax(DriveConstants.FRONT_RIGHT_DRIVE_ID,
                            DriveConstants.FRONT_RIGHT_TURN_ID), 1),
                    new SwerveModule(new SwerveModuleIOSparkMax(DriveConstants.BACK_LEFT_DRIVE_ID,
                            DriveConstants.BACK_LEFT_TURN_ID), 2),
                    new SwerveModule(new SwerveModuleIOSparkMax(DriveConstants.BACK_RIGHT_DRIVE_ID,
                            DriveConstants.BACK_RIGHT_TURN_ID), 3),
            };

            gyro = new GyroIONavX();

        } else {
            swerveModules = new SwerveModule[] {
                    new SwerveModule(new SwerveModuleIOSim(), 0),
                    new SwerveModule(new SwerveModuleIOSim(), 1),
                    new SwerveModule(new SwerveModuleIOSim(), 2),
                    new SwerveModule(new SwerveModuleIOSim(), 3)
            };

            gyro = new GyroIOSim();
        }

        odometry = new SwerveDrivePoseEstimator(
                DriveConstants.SWERVE_KINEMATICS,
                new Rotation2d(),
                getModulePositions(),
                new Pose2d());
    }

    @Override
    public void periodic() {

        if (Robot.isSimulation()) {
            simulationPeriodic();
        }

        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        odometry.update(gyroInputs.yawPosition, getModulePositions());

        Logger.recordOutput("Odometry", odometry.getEstimatedPosition());

        swerveModules[0].periodic();
        swerveModules[1].periodic();
        swerveModules[2].periodic();
        swerveModules[3].periodic();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            swerveModules[0].stop();
            swerveModules[1].stop();
            swerveModules[2].stop();
            swerveModules[3].stop();

            // Record blank states
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        }

        // Log measured states
        Logger.recordOutput("SwerveStates/Measured", getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds simSpeeds = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        Constants.FIELD.setRobotPose(getEstimatedPose());
        gyroInputs.yawPosition = gyroInputs.yawPosition
                .plus(Rotation2d.fromRadians(simSpeeds.omegaRadiansPerSecond * 0.02));
        gyroInputs.yawVelocityRadPerSec = simSpeeds.omegaRadiansPerSecond;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        Logger.recordOutput("SwerveStates/Setpoints", states);

        swerveModules[0].setState(states[0]);
        swerveModules[1].setState(states[1]);
        swerveModules[2].setState(states[2]);
        swerveModules[3].setState(states[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
    }

    public void setVoltages(double driveVolts, double turnVolts) {
        swerveModules[0].setVoltages(driveVolts, turnVolts);
        swerveModules[1].setVoltages(driveVolts, turnVolts);
        swerveModules[2].setVoltages(driveVolts, turnVolts);
        swerveModules[3].setVoltages(driveVolts, turnVolts);
    }

    public void setDriveVoltages(double volts) {
        setVoltages(volts, 0);
    }

    public void setTurnVoltages(double volts) {
        setVoltages(0, volts);
    }

    public void stop() {
        swerveModules[0].stop();
        swerveModules[1].stop();
        swerveModules[2].stop();
        swerveModules[3].stop();
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }
}