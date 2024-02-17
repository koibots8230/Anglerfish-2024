// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import static com.koibots.robot.subsystems.Subsystems.PlopperPivot;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private final LinearSystemLoop<N2, N1, N1> linearSysLoop;
    private final TrapezoidProfile profile;

    private TrapezoidProfile.State lastProfiledReference;

    private TrapezoidProfile.State targetState;

    private Measure<Distance> setpoint = Meters.of(0);

    private boolean isSysID = false;

    private Measure<Voltage> volts;

    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d elevatorMech2d;
    private final MechanismLigament2d plopperMech2d;

    public Elevator() {
        System.out.println("Elevator initialized");

        linearSysLoop =
                new LinearSystemLoop<>(
                        ElevatorConstants.LINEAR_SYS,
                        ElevatorConstants.LQR,
                        ElevatorConstants.KALMAN_FILTER,
                        12.0,
                        0.020);

        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                ElevatorConstants.MAX_SPEED.in(MetersPerSecond),
                                ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

        targetState = new TrapezoidProfile.State(0, 0.0);

        io = Robot.isReal() ? new ElevatorIOSparkMax() : new ElevatorIOSim();

        io.setBrake();

        mechanism2d = new Mechanism2d(.5, 1);

        elevatorMech2d =
                mechanism2d
                        .getRoot("Elevator Root", 10, 0)
                        .append(new MechanismLigament2d("Elevator", io.getPosition().in(Meters), 90));
        
        plopperMech2d = elevatorMech2d.append(
            new MechanismLigament2d("Plopper", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    }

    @Override
    public void periodic() {
        inputs.setpoint = setpoint;

        io.updateInputs(inputs);

        elevatorMech2d.setLength(io.getPosition().in(Meters));

        Logger.processInputs("Subsystems/Elevator", inputs);

        Logger.recordOutput("Elevator Mechanism", mechanism2d);

        lastProfiledReference = profile.calculate(0.020, lastProfiledReference, targetState);

        linearSysLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
        linearSysLoop.correct(VecBuilder.fill(io.getPosition().in(Meters)));
        linearSysLoop.predict(0.020);

        if (!isSysID) {
            volts = Volts.of(linearSysLoop.getU(0));
            volts =
                    Robot.isReal()
                            ? (Math.abs(volts.in(Volts)) > 0.5) ? volts : Volts.of(0)
                            : volts;
        }

        io.setVoltage(volts);
    }

    public void reset() {
        linearSysLoop.reset(
                VecBuilder.fill(io.getPosition().in(Meters), io.getVelocity().in(MetersPerSecond)));

        lastProfiledReference = new TrapezoidProfile.State(io.getPosition(), io.getVelocity());
    }

    public void setPostion(Measure<Distance> position) {
        setpoint = position;
        targetState = new TrapezoidProfile.State(position.in(Meters), 0.0);
    }

    public void setVoltage(Measure<Voltage> volts) {
        this.volts = volts;
        isSysID = true;
    }

    public boolean atSetpoint() {
        return inputs.position.in(Meters)
                        >= setpoint.minus(ElevatorConstants.ALLOWED_ERROR).in(Meters)
                && inputs.position.in(Meters)
                        <= setpoint.plus(ElevatorConstants.ALLOWED_ERROR).in(Meters);
    }

    public Measure<Distance> getSetpoint() {
        return setpoint;
    }
}
