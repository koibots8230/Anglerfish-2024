package frc.robot.IO;


import com.revrobotics.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.RPM;

public class Motor {
    private static Constants.Motors.Motor definition;
    private static Measure<Velocity<Angle>> rawVelocity;
    private static Measure<Velocity<Angle>> velocity;
    private static CANSparkMax canSparkMax;
    private static SparkPIDController sparkPIDController;
    private static RelativeEncoder encoder;

    public Motor(Constants.Motors.Motor motor) {
        Motor.definition = motor;

        switch (definition.motorType) {
            case CANSPARKMAX_BRUSHLESS:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushless);
                return;
            case CANSPARKMAX_BRUSHED:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushed);
                return;
            default:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushless);
        }

        Motor.encoder = Motor.canSparkMax.getEncoder();
        Motor.sparkPIDController = Motor.canSparkMax.getPIDController();
        Motor.sparkPIDController.setP(Motor.definition.P);
        Motor.sparkPIDController.setI(Motor.definition.I);
        Motor.sparkPIDController.setD(Motor.definition.D);
        Motor.sparkPIDController.setIZone(Motor.definition.IZone);
        Motor.sparkPIDController.setFF(Motor.definition.FF);
    }

    public void setRawVelocity(Measure<Velocity<Angle>> rawVelocity) {
        Motor.rawVelocity = rawVelocity;
        Motor.velocity = rawVelocity.divide(definition.gearing);
        Motor.sparkPIDController.setReference(Motor.rawVelocity.in(RPM), CANSparkBase.ControlType.kVelocity);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        Motor.velocity = velocity;
        Motor.rawVelocity = velocity.times(definition.gearing);
        Motor.sparkPIDController.setReference(rawVelocity.in(RPM), CANSparkBase.ControlType.kVelocity);
    }
}
