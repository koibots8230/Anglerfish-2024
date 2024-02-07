package com.koibots.robot.subsystems.intake.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Robot;
import com.koibots.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {    
    //private SparkPIDController intakePidController;

    private final PIDController intakeFeedback;
    private SimpleMotorFeedforward intakeFeedForward;
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    //private double intakeTargetRPM = 0.0;
    private double intakeVoltsSetPoint = 0.0;

    
    public Intake() {

        System.out.println("Intake made");

        if (Robot.isReal()) {
            //intakeTargetRPM = 0;
            this.io = new IntakeIOSparkMax();
            intakeFeedForward = new SimpleMotorFeedforward(0, 1.023);
            intakeFeedback = new PIDController(IntakeConstants.INTAKE_PID_P, 0, 0);
        } else {
            //intakeTargetRPM = 0;
            this.io = new IntakeIOSim();
            intakeFeedForward = new SimpleMotorFeedforward(0, 0);
            intakeFeedback = new PIDController(IntakeConstants.INTAKE_PID_P, 0, 0);
        }
    
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
         Logger.processInputs("Intake", inputs);
        io.setVoltage(
            intakeFeedback.calculate(
                inputs.intakeVoltage,
                Math.max(intakeVoltsSetPoint, IntakeConstants.INTAKE_MINIMUM_VOLTAGE) // wonky?
            ) + intakeFeedForward.calculate(
                    Math.max(intakeVoltsSetPoint, IntakeConstants.INTAKE_MINIMUM_VOLTAGE)
            )
        );
    }


    public void setIntakeVoltsWithTargetRPM(double targetRPM) {

        double robotSpeed = Swerve.get().getModuleStates()[0].speedMetersPerSecond * 60;
        // meters per minute

        double intakeWheelCircumference = 2 * Math.PI * IntakeConstants.INTAKE_WHEEL_RADIUS;
        double targetDistancePerMinute = targetRPM * intakeWheelCircumference;
        double trueDistancePerMinute = targetDistancePerMinute - robotSpeed;

        double trueRPM = trueDistancePerMinute / intakeWheelCircumference;
        
        // if this breaks, blame someone else
        //System.out.println("truerpm: " + trueRPM);
        intakeVoltsSetPoint = Math.max(Math.min(trueRPM * (12.0 / 5676.0), 12.0), -12.0); 

        //System.out.println("setting voltage setpoint: " + intakeVoltsSetPoint);
        //System.out.println("function thingy - " + intakeVoltsSetPoint);

    }
 
}
