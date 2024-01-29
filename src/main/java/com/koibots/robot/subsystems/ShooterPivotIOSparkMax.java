package com.koibots.robot.subsystems;

import com.koibots.robot.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;

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
        inputs.position = Rotation2d.fromRadians(shooterPivotEncoder.getPosition());
        inputs.voltage = Volts.of(shooterPivotMotor.getBusVoltage());
        inputs.current = Amps.of(shooterPivotMotor.getOutputCurrent());
        inputs.velocity = RadiansPerSecond.of(shooterPivotEncoder.getVelocity());
    }

    public void zeroOffset() {
        shooterPivotEncoder.setZeroOffset(Constants.SHOOTER_PIVOT_ZERO_OFFSET);
    }

    public void setMotorSpeed(double desiredPosition){
        shooterPivotMotor.set(desiredPosition);
    }

    public void setBrakeMode() {
        shooterPivotMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode() {
        shooterPivotMotor.setIdleMode(IdleMode.kCoast);
    }
}
