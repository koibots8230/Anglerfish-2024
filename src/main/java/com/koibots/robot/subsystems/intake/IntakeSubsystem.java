package com.koibots.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public static IntakeSubsystem get() {
        return intakeSubsystem;
    }

    public final CANSparkMax intakeMotor;

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(
            -1, //Constants.IntakeConstants.INTAKE_MOTOR_PORT,
            MotorType.kBrushless
        );
    }

    @Override
    public void periodic() {

    }

}
