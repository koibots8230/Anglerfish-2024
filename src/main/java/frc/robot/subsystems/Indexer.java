package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final SparkMax motor;

    private final SparkMaxConfig config;

    private final DigitalInput distanceSwitch;

    public Indexer() {
        motor = new SparkMax(0, MotorType.kBrushless);

        config = new SparkMaxConfig();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        distanceSwitch = new DigitalInput(0);
    }

    private void setSpeed(double speed) {
        motor.set(speed);
    }

    public boolean sensorTriggered() {
        return !distanceSwitch.get();
    }

    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> this.setSpeed(speed), this);
    }
}
