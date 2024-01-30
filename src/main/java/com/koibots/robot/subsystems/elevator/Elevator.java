package com.koibots.robot.subsystems.elevator;

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

    public Elevator() {
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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        lastProfiledReference = profile.calculate(0.020, lastProfiledReference, targetState);

        linearSysLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
        linearSysLoop.correct(VecBuilder.fill(io.getPosition()));
        linearSysLoop.predict(0.020);

        io.setVoltage(linearSysLoop.getU(0));
    }

    public void reset() {
        linearSysLoop.reset(VecBuilder.fill(io.getPosition(), io.getVelocity()));

        lastProfiledReference =
            new TrapezoidProfile.State(io.getPosition(), io.getVelocity());
    }

    public void setPostion(Measure<Distance> position) {
        targetState = new TrapezoidProfile.State(position.in(Meters), 0.0);
    }
}
