package com.koibots.robot.subsystems;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPositionSubsystem extends SubsystemBase {
    private final CANSparkMax shooterPositionMotor;
    private final AbsoluteEncoder shooterPositionEncoder;

    private double desiredPos;

    ArmFeedforward Feedforward = new ArmFeedforward(0, 0, 0, 0);
    PIDController PID = new PIDController(0, 0, 0);


    public ShooterPositionSubsystem() {
        shooterPositionMotor = new CANSparkMax(Constants.SHOOTER_POSITION_MOTOR, MotorType.kBrushless);
        shooterPositionEncoder = shooterPositionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPositionEncoder.setZeroOffset(0);
        shooterPositionEncoder.setPositionConversionFactor(Constants.SHOOTER_POSITION_ENCODER_POSITION_FACTOR);
        shooterPositionMotor.setIdleMode(IdleMode.kBrake);

        desiredPos = 0;
    }

    @Override
    public void periodic() {
        shooterPositionMotor.set(PID.calculate(shooterPositionEncoder.getPosition(), desiredPos) + Feedforward.calculate(0, 0, 0));
    }

    //getters

    public double getShooterMotorPosition() {
        return shooterPositionEncoder.getPosition();
    }

    public double getShooterMotorCurrent() {
        return shooterPositionMotor.getOutputCurrent();
    }

    //setters

    public void resetShooterMotorPosition() {
        shooterPositionEncoder.setZeroOffset(0);
    }

    public void setShooterPositionModeBreak() {
        shooterPositionMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setShooterPositionModeCoast() {
        shooterPositionMotor.setIdleMode(IdleMode.kCoast);
    }

    //commands

    public void setShooterPosition(double desiredPosition) {
        desiredPos = desiredPosition;
    }
}