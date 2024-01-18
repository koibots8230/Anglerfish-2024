package com.koibots.robot.subsystems;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPositionSubsystem extends SubsystemBase {
    private static ShooterPositionSubsystem m_ShooterPositionSubsystem = new ShooterPositionSubsystem();
    private CANSparkMax shooterPositionMotor;
    private AbsoluteEncoder shooterPositionEncoder;

    private boolean canLoad;


    ShooterPositionSubsystem() {
        shooterPositionMotor = new CANSparkMax(Constants.SHOOTER_POSITION_MOTOR, MotorType.kBrushless);
        shooterPositionEncoder = shooterPositionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPositionEncoder.setZeroOffset(0);
        shooterPositionMotor.setIdleMode(IdleMode.kBrake);

        canLoad = true;
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
    
    public void loadNote() {
        while(Math.abs(shooterPositionEncoder.getPosition()) != 0) {
        }
    }

    public void setShooterPosition(double desiredPosition) {
        if(shooterPositionEncoder.getPosition() > desiredPosition) {
            shooterPositionMotor.set(Constants.SHOOTER_POSITION_MOTOR_REVERSE_SPEED);
        }
        else if(desiredPosition > shooterPositionEncoder.getPosition()){
            shooterPositionMotor.set(Constants.SHOOTER_POSITION_MOTOR_SPEED);
        }
        else {
            shooterPositionMotor.set(0);
            canLoad = true;
        }
    }
}