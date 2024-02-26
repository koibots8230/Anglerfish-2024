// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.plopperPivot;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.RobotConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PlopperPivotIOSparkMax implements PlopperPivotIO {
    private final CANSparkMax plopperPivotMotor;
    private final DutyCycleEncoder plopperPivotEncoder;

    public PlopperPivotIOSparkMax() {
        plopperPivotMotor = new CANSparkMax(Constants.DeviceIDs.PLOPPER_PIVOT, MotorType.kBrushless);
        plopperPivotMotor.setIdleMode(IdleMode.kBrake);
        plopperPivotMotor.setSmartCurrentLimit(30, 60, 5670);

        plopperPivotMotor.restoreFactoryDefaults();
        plopperPivotMotor.burnFlash();

        plopperPivotEncoder = new DutyCycleEncoder(4);
        plopperPivotEncoder.setDistancePerRotation(
                RobotConstants.PLOPPER_PIVOT_ENCODER_POSITION_FACTOR);
        plopperPivotEncoder.reset();
    }

    @Override
    public void updateInputs(PlopperPivotIOInputs inputs) {
        inputs.position = Radians.of(plopperPivotEncoder.getDistance());
        inputs.voltage = Volts.of(plopperPivotMotor.getBusVoltage());
        inputs.current = Amps.of(plopperPivotMotor.getOutputCurrent());
    }

    public void setVoltage(double volts) {
        plopperPivotMotor.setVoltage(-volts);
    }

    public void setIdleMode(boolean isBrake) {
        plopperPivotMotor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
