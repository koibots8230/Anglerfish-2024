package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        ChassisSpeeds speeds = Constants.SWERVE_KINEMATICS.toChassisSpeeds(Swerve.get().getModuleStates());

        Logger.recordOutput(
                "Calculated Speeds",
                new double[] {
                        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
                });

        inputs.yawPosition = inputs.yawPosition
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02));
        inputs.yawVelocityRadPerSec = speeds.omegaRadiansPerSecond;
    }
}