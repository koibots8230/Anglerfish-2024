package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private SparkMax topMotor;
    private SparkMax bottomMotor;
    private SparkMaxConfig topConfig;
    private SparkMaxConfig bottomConfig;

    public Shooter() {
        topMotor = new SparkMax(20, MotorType.kBrushless);
        bottomMotor = new SparkMax(21, MotorType.kBrushless);

        topConfig = new SparkMaxConfig();
        topConfig.inverted(false);
        topConfig.smartCurrentLimit(80);
        topConfig.idleMode(IdleMode.kBrake);

        bottomConfig = new SparkMaxConfig();
        bottomConfig.inverted(true);
        bottomConfig.smartCurrentLimit(80);
        bottomConfig.follow(topMotor);
        bottomConfig.idleMode(IdleMode.kBrake);

        topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setVelocity(double speed) {
        topMotor.set(speed);
    }

    public Command setVelocityCommand(double speed) {
        return Commands.runOnce(() -> setVelocity(speed), this);
    }
}
