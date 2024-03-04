// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
    AHRS gyro;

    protected GyroIONavX() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        Rotation2d previousYaw = inputs.yawPosition;
        inputs.yawPosition = Rotation2d.fromDegrees(gyro.getYaw() * -1);
        inputs.yawVelocityRadPerSec = inputs.yawPosition.minus(previousYaw).getRadians() / 0.02;
    }

    @Override
    public void zeroYaw() {
        gyro.zeroYaw();
    }
}
