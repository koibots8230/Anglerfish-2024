package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;

import com.koibots.robot.Robot;
import com.koibots.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private final LinearSystemLoop<N2, N1, N1> linearSysLoop;
    private final TrapezoidProfile profile;

    private TrapezoidProfile.State lastProfiledReference;

    private TrapezoidProfile.State targetState;

    private double setpoint;

    private double volts;

    public Elevator() {
        System.out.println("Elevator initialized");

        linearSysLoop =
            new LinearSystemLoop<>(
                ElevatorConstants.LINEAR_SYS, 
                ElevatorConstants.LQR, 
                ElevatorConstants.KALMAN_FILTER, 
                12.0, 
                0.020
            );

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_SPEED.in(MetersPerSecond),
                ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond))
        );

        targetState = new TrapezoidProfile.State(0, 0.0);

        io = Robot.isReal() ? new ElevatorIOSparkMax() : new ElevatorIOSim();

        io.setBrake();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        inputs.setpoint = setpoint;

        Logger.processInputs("Elevator/Inputs", inputs);

        Logger.recordOutput("Elevator/SimMechanism", io.getMechanism());
        
        lastProfiledReference = profile.calculate(0.020, lastProfiledReference, targetState);

        linearSysLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
        linearSysLoop.correct(VecBuilder.fill(io.getPosition()));
        linearSysLoop.predict(0.020);
        
        volts = linearSysLoop.getU(0);
        volts = Robot.isReal() ? (Math.abs(volts) > 0.5) ? volts : 0 : volts;
        
        io.setVoltage(volts);
    }

    @Override
    public void simulationPeriodic() {
        
    }

    public void reset() {
        linearSysLoop.reset(VecBuilder.fill(io.getPosition(), io.getVelocity()));

        lastProfiledReference =
            new TrapezoidProfile.State(io.getPosition(), io.getVelocity());
    }

    public void setPostion(Measure<Distance> position) {
        setpoint = position.in(Inches);
        targetState = new TrapezoidProfile.State(position.in(Meters), 0.0);
    }
}
