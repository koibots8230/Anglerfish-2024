package com.koibots.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.koibots.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    
    public static IntakeSubsystem get() {
        return intakeSubsystem;
    }
    
    
    public final CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;
    private SparkPIDController intakePidController;
    
    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(
            IntakeConstants.INTAKE_MOTOR_PORT,
            MotorType.kBrushless
        );

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);

        intakePidController = intakeMotor.getPIDController();
        intakePidController.setP(IntakeConstants.INTAKE_PID_P);
    }


    @Override
    public void periodic() {
        
    }


    public void setIntakeMotor(double targetRPM) {

        double robotSpeed = 0;

        double intakeWheelCircumference = 2 * Math.PI * IntakeConstants.INTAKE_WHEEL_RADIUS;
        double targetDistancePerMinute = targetRPM * intakeWheelCircumference;
        double trueDistancePerMinute = targetDistancePerMinute - robotSpeed;
        double trueRPM = trueDistancePerMinute / intakeWheelCircumference;

        intakePidController.setReference(
            Math.max(trueRPM, IntakeConstants.INTAKE_MINIMUM_RPM),
            CANSparkMax.ControlType.kVelocity
        );

    }
 
}
