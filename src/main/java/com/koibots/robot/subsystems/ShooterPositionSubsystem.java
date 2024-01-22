package com.koibots.robot.subsystems;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import java.lang.Math;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPositionSubsystem extends SubsystemBase {
    private static ShooterPositionSubsystem m_ShooterPositionSubsystem = new ShooterPositionSubsystem();
    private CANSparkMax shooterPositionMotor;
    private AbsoluteEncoder shooterPositionEncoder;

    private boolean canLoad;
    private double desiredPos;


    public ShooterPositionSubsystem() {
        shooterPositionMotor = new CANSparkMax(Constants.SHOOTER_POSITION_MOTOR, MotorType.kBrushless);
        shooterPositionEncoder = shooterPositionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPositionEncoder.setZeroOffset(0);
        shooterPositionEncoder.setPositionConversionFactor(Constants.SHOOTER_POSITION_ENCODER_POSITION_FACTOR);
        shooterPositionMotor.setIdleMode(IdleMode.kBrake);

        canLoad = true;
        desiredPos = 0;

        ArmFeedforward Feedforward = new ArmFeedforward(0, kG, kV, 0);
        PIDController PID = new PIDController(kP, 0, kD);
    }

    @Override
    public void periodic() {
        shooterPositionMotor.set(PID.calculate(shooterPositionEncoder.getPosition(), desiredPos) + Feedforward.calculate());
        super.periodic();
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
        if(desiredPosition == 0) {
            canLoad = true;
        } else {
            canLoad = false;
        }
    }
}