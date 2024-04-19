package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import monologue.Logged;
import monologue.Annotations.*;

public class Shooter extends SubsystemBase implements Logged{
    @Log private final CANSparkMax topMotor;
    @Log private final CANSparkMax bottomMotor;
    @Log private final SparkPIDController topShoterPID;
    @Log private final SparkPIDController bottomShoterPID;
    @Log private final RelativeEncoder topShooterEncoder;
    @Log private final RelativeEncoder bottomShooterEncoder;


    public Shooter(){
        topMotor = new CANSparkMax(3, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(4, MotorType.kBrushless);

        topShooterEncoder = topMotor.getEncoder();
        bottomShooterEncoder = bottomMotor.getEncoder();

        topShoterPID = topMotor.getPIDController();

        topShoterPID.setP(PIDConstants.TOP_SHOOTER_PID_KP);
        topShoterPID.setI(PIDConstants.TOP_SHOOTER_PID_KI);
        topShoterPID.setD(PIDConstants.TOP_SHOOTER_PID_KD);

        topShoterPID.setFF(PIDConstants.TOP_SHOOTER_FEEDFORWARD_FF);

        bottomShoterPID = bottomMotor.getPIDController();

        bottomShoterPID.setP(PIDConstants.TOP_SHOOTER_PID_KP);
        bottomShoterPID.setI(PIDConstants.TOP_SHOOTER_PID_KI);
        bottomShoterPID.setD(PIDConstants.TOP_SHOOTER_PID_KD);

        bottomShoterPID.setFF(PIDConstants.TOP_SHOOTER_FEEDFORWARD_FF);
    }
    
    public void setTopShooterVelocity(double topShooterVelocity){
        topShoterPID.setReference(PIDConstants.TOP_SHOOTER_SETPOINT, CANSparkBase.ControlType.kVelocity);
        

    }

    public void setBottomShooterVelocity(double bottomShooterVelovity) {
        bottomShoterPID.setReference(PIDConstants.BOTTOM_SHOOTER_SETPOINT, CANSparkBase.ControlType.kVelocity);
    }

    public boolean shooterAtSpeedAmp(){
        return 
        Math.abs(topShooterEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_AMP && 
        Math.abs(bottomShooterEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_AMP;
    }

        public boolean shooterAtSpeedSpeaker(){
        return 
        Math.abs(topShooterEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_SPEAKER && 
        Math.abs(bottomShooterEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_SPEAKER;
    }


}
