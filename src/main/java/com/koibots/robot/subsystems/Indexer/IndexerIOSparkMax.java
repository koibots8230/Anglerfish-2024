package com.koibots.robot.subsystems.Indexer;

import com.koibots.robot.Constants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import static edu.wpi.first.units.Units.*;

public class IndexerIOSparkMax implements IndexerIO{
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public IndexerIOSparkMax() {
        motor = new CANSparkMax(Constants.INDEXER_MOTOR, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity = MetersPerSecond.of(encoder.getVelocity());
        if(motor.getIdleMode() == IdleMode.kBrake) {
            inputs.mode = true;
        } else {
            inputs.mode = false;
        }
    }

    public void runMotor() {
        motor.set(Constants.INDEXER_SPEED);
    }

    public void setIdle(boolean mode) {
        if(mode == true) {
            motor.setIdleMode(IdleMode.kBrake);
        } else {
            motor.setIdleMode(IdleMode.kCoast);
        }
    } 
}
