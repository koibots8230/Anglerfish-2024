package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private DCMotorSim topSimMotor;
    private DCMotorSim bottomSimMotor;
    @Log
    private RelativeEncoder topEncoder;
    @Log
    private RelativeEncoder bottomEncoder;
    @Log
    private SparkPIDController topShoterPID;
    @Log
    private SparkPIDController bottomShoterPID;
    @Log
    private SimpleMotorFeedforward topSimFF;
    @Log
    private SimpleMotorFeedforward bottomSimFF;
    @Log
    private PIDController topSimPID;
    @Log
    private PIDController bottomSimPID;

    private final boolean isReal;

    @Log
    private double topShoterVelocity;
    @Log
    private double bottomShoterVelocity;

    @Log
    private double  bottomShoterCurrent;
    @Log
    private double  topshoterCurrent;
    @Log
    private double  topAppliedVoltage;
    @Log
    private double bottomAppliedVoltage;
    @Log
    private double topShoterSetpoint;
    @Log
    private double bottomShoterSetpoint;



    public Shooter(boolean isReal) {
        this.isReal = isReal;
        if (Robot.isReal()) {
            topMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
            bottomMotor = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);


            topShoterPID = topMotor.getPIDController();

            bottomShoterPID = bottomMotor.getPIDController();

            topEncoder = topMotor.getEncoder();

            bottomEncoder = bottomMotor.getEncoder();

            topShoterPID.setP(Constants.MotorDefinitions.topShooter.P);
            topShoterPID.setI(Constants.MotorDefinitions.topShooter.I);
            topShoterPID.setD(Constants.MotorDefinitions.topShooter.D);
            topShoterPID.setFF(Constants.MotorDefinitions.topShooter.FF);

            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.P);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.I);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.D);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.FF);
        }

        else {
            topSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            bottomSimMotor = new DCMotorSim(DCMotor.getNEO(1),1,1);

            topSimFF = new SimpleMotorFeedforward(0.002,0.002);
            bottomSimFF = new SimpleMotorFeedforward(0.002,0.002);

            topSimPID = new PIDController(0.002,0.002,0.002);
            bottomSimPID = new PIDController(0.002, 0.002, 0.002);

        }


    }

    @Override
    public void periodic(){
        if(isReal){
            topShoterPID.setReference(topShoterVelocity, CANSparkBase.ControlType.kVelocity);
            bottomShoterPID.setReference(bottomShoterVelocity, CANSparkBase.ControlType.kVelocity);

        }
    }

    @Override
    public void simulationPeriodic(){
        topSimMotor.update(.2);
        bottomSimMotor.update(.2);

        topshoterCurrent = topSimMotor.getCurrentDrawAmps();
        topShoterVelocity = topSimMotor.getAngularVelocityRPM();

        bottomShoterVelocity = bottomSimMotor.getAngularVelocityRPM();
        bottomShoterCurrent = bottomSimMotor.getCurrentDrawAmps();

        topAppliedVoltage =
                topSimPID.calculate(topShoterVelocity, topShoterSetpoint) + topSimFF.calculate(topShoterSetpoint);
        bottomAppliedVoltage = bottomSimPID.calculate(bottomShoterVelocity, bottomShoterSetpoint) + bottomSimFF.calculate(bottomShoterSetpoint);

        topSimMotor.setInputVoltage(topAppliedVoltage);
        bottomSimMotor.setInputVoltage(bottomAppliedVoltage);
    }

    public void setVelocity(double topShooterSetpoint, double bottomShooterSetpoint) {
        this.topShoterSetpoint = topShooterSetpoint;
        this.bottomShoterSetpoint = bottomShooterSetpoint;
                System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
        System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");System.out.println("made it to set velocity");
    }

    public boolean checkVelocity() {
        return Math.abs(topEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_AMP_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_AMP
                &&
                Math.abs(bottomEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_AMP_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_AMP
                ||
                Math.abs(topEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_SPEAKER_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_AMP
                        &&
                        Math.abs(bottomEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_SPEAKER_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_AMP;
    }


}
