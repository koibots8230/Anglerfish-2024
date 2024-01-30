package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.koibots.robot.Constants.ElevatorConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    
    private DCMotorSim leftMotor;
    private DCMotorSim rightMotor;

    private double appliedVolts;

    public ElevatorIOSim() {
        leftMotor = new DCMotorSim(DCMotor.getNEO(1), 60, 0);
        rightMotor = new DCMotorSim(DCMotor.getNEO(1), 60, 0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        leftMotor.update(0.020);
        rightMotor.update(0.020);

        inputs.position = leftMotor.getAngularPositionRad() / (2 * Math.PI) * ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters);
        inputs.appliedVoltage = appliedVolts;
        inputs.leftAmperage = leftMotor.getCurrentDrawAmps();
        inputs.rightAmperage = rightMotor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        appliedVolts = volts;
        leftMotor.setInputVoltage(volts);
        rightMotor.setInputVoltage(volts);
    }

    @Override
    public double getPosition() {
        return leftMotor.getAngularPositionRad() / (2 * Math.PI) * ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters);
    }

    @Override
    public double getVelocity() {
        return leftMotor.getAngularVelocityRPM() * ElevatorConstants.DISTANCE_PER_REVOLUTION.in(Meters);
    }
}
