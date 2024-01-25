package com.koibots.robot.subsystems;

import com.koibots.robot.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {
    private final CANSparkMax shooterPivotMotor;
    private final AbsoluteEncoder shooterPivotEncoder;

    public ShooterPivotIOSparkMax() {
        shooterPivotMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
        shooterPivotEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPivotEncoder.setZeroOffset(0);
        shooterPivotEncoder.setPositionConversionFactor(Constants.SHOOTER_PIVOT_ENCODER_POSITION_FACTOR);
        shooterPivotMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterPivotPosition = shooterPivotEncoder.getPosition();
    }

    public double getShooterPosition() {
        return shooterPivotEncoder.getPosition();
    }

    public void zeroShooterPositionOffset() {
        shooterPivotEncoder.setZeroOffset(0);
    }

    public double getShooterPivotOutputCurrent() {
        return shooterPivotMotor.getOutputCurrent();
    }

    public void setShooterPivotMotorSpeed(double desiredPosition){
        shooterPivotMotor.set(desiredPosition);
    }

    public void setShooterPivotBrakeMode() {
        shooterPivotMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setShooterPivotCoastMode() {
        shooterPivotMotor.setIdleMode(IdleMode.kCoast);
    }
}
