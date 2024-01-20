package com.koibots.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class IntakePositionSubsystem extends SubsystemBase {

    private static IntakePositionSubsystem intakePositionSubsystem = new IntakePositionSubsystem();

    public static IntakePositionSubsystem get() {
        return intakePositionSubsystem;
    }


    public final CANSparkMax intakePositionMotor;
    public final RelativeEncoder intakePositionEncoder;

    private IntakePositionSubsystem() {
        intakePositionMotor = new CANSparkMax(
            -1, //Constants.IntakeConstants.INTAKE_POSITION_MOTOR_PORT,
            MotorType.kBrushless
        );

        intakePositionMotor.setIdleMode(IdleMode.kBrake);

        intakePositionEncoder = intakePositionMotor.getEncoder();
        intakePositionEncoder.setPosition(0);
    }


    @Override
    public void periodic() {

    }

}
