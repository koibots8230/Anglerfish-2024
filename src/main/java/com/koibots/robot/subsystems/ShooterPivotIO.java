package com.koibots.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
    @AutoLog
    public static class ShooterPivotIOInputs {
        public double shooterPivotPosition = 0.0;
    }

    void updateInputs(ShooterPivotIOInputs inputs);
    double getShooterPosition();
    void zeroShooterPositionOffset();
    double getShooterPivotOutputCurrent();
    void setShooterPivotMotorSpeed(double desiredPosition);
    void setShooterPivotBrakeMode();
    void setShooterPivotCoastMode();
}
