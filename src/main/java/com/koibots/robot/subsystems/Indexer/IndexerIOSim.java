// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage; 
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

    private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);

    private Measure<Voltage> volts = Volts.of(0);

    private final PIDController feedback = new PIDController(
        ControlConstants.INTAKE_FEEDBACK_CONSTANTS.kP,
        ControlConstants.INTAKE_FEEDBACK_CONSTANTS.kI,
        ControlConstants.INTAKE_FEEDBACK_CONSTANTS.kD
    );

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        ControlConstants.INTAKE_FEEDFORWARD_CONSTANTS.ks,
        ControlConstants.INTAKE_FEEDFORWARD_CONSTANTS.kv
    );

   private Measure<Velocity<Angle>> setpoint = RPM.of(0);
    

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocity = sim.getAngularVelocityRPM();


        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.voltage = Volts.of(feedback.calculate(sim.getAngularVelocityRPM(), setpoint.in(RPM)) + feedforward.calculate(setpoint.in(RPM)));

        sim.setInputVoltage(
            inputs.voltage.in(Volts)
        );
    }


    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpoint = velocity;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(sim.getAngularVelocityRPM());
    }

    public Measure<Voltage> getVoltage() {
        return volts;
    }
}