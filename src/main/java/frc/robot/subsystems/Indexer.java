package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import monologue.Annotations.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Indexer extends SubsystemBase {

    @Log
    private final DigitalInput indexerDigitalInput;
    @Log
    private final CANSparkMax indexerMotor;
    @Log
    private final SparkPIDController indexerPID;

    public Indexer() {
        indexerDigitalInput = new DigitalInput(0);
        indexerMotor = new CANSparkMax(2, MotorType.kBrushless);
        indexerPID = indexerMotor.getPIDController();

        indexerPID.setP(PIDConstants.INDEXER_PID_KP);
        indexerPID.setI(PIDConstants.INDEXER_PID_KI);
        indexerPID.setD(PIDConstants.INDEXER_PID_KD);

        indexerPID.setFF(PIDConstants.INDEXER_FEEDFORWARD_FF);
    }

    public void setIndexerVelocity(double IndexerVelocity) {
        indexerPID.setReference(IndexerVelocity, CANSparkBase.ControlType.kVelocity);
    }

    @Log
    public boolean indexerNoteDetected() {
        return indexerDigitalInput.get();
    }

}
