package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final SparkMax topMotor;
    private final SparkMax bottomMotor;

    private final SparkMaxConfig config;


    public Shooter() {
        topMotor = new SparkMax(11, MotorType.kBrushless);

        bottomMotor = new SparkMax(13, MotorType.kBrushless);

        config = new SparkMaxConfig();

        config.smartCurrentLimit(60);

        config.inverted(true);
        
        topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setSpeeds(double top, double bottom) {
        topMotor.set(top);
        bottomMotor.set(bottom);
    }

    public Command setSpeedCommand(double top, double bottom) {
        return Commands.runOnce(() -> this.setSpeeds(top, bottom), this);
    }
}
