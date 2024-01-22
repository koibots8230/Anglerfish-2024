// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        ChassisSpeeds speeds =
                DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(Swerve.get().getModuleStates());

        inputs.yawPosition =
                inputs.yawPosition.plus(
                        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02));
        inputs.yawVelocityRadPerSec = speeds.omegaRadiansPerSecond;
    }
}
