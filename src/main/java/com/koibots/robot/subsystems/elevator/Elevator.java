// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.ElevatorConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_SPEED, ElevatorConstants.MAX_ACCELERATION);
    private final ProfiledPIDController feedback;
    private final ElevatorFeedforward feedforward;

    private Measure<Distance> setpoint = Meters.of(0);
    private TrapezoidProfile.State targetState =
            new TrapezoidProfile.State(Meters.of(0), MetersPerSecond.of(0));

    private boolean isSysID = false;

    private Measure<Voltage> volts;

    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d elevatorMech2d;

    public Elevator() {
        feedback =
                new ProfiledPIDController(
                        ControlConstants.ELEVATOR_FEEDBACK_CONSTANTS.kP,
                        ControlConstants.ELEVATOR_FEEDBACK_CONSTANTS.kI,
                        ControlConstants.ELEVATOR_FEEDBACK_CONSTANTS.kD,
                        constraints,
                        0.020);
        feedforward =
                new ElevatorFeedforward(
                        ControlConstants.ELEVATOR_FEEDFORWARD_CONSTANTS.ks,
                        ControlConstants.ELEVATOR_FEEDFORWARD_CONSTANTS.kg,
                        ControlConstants.ELEVATOR_FEEDFORWARD_CONSTANTS.kv,
                        ControlConstants.ELEVATOR_FEEDFORWARD_CONSTANTS.ka);

        io = Robot.isReal() ? new ElevatorIOSparkMax() : new ElevatorIOSim();

        io.setBrake();

        mechanism2d = new Mechanism2d(.5, 1);

        elevatorMech2d =
                mechanism2d
                        .getRoot("Elevator Root", 10, 0)
                        .append(
                                new MechanismLigament2d(
                                        "Elevator", io.getPosition().in(Meters), 90));

        elevatorMech2d.append(
                new MechanismLigament2d("Plopper", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    }

    @Override
    public void periodic() {
        inputs.setpoint = setpoint;

        io.updateInputs(inputs);

        elevatorMech2d.setLength(io.getPosition().in(Meters));

        Logger.processInputs("Subsystems/Elevator", inputs);

        Logger.recordOutput("Elevator Mechanism", mechanism2d);

        if (!isSysID) {
            feedback.setGoal(targetState);

            volts =
                    Volts.of(
                            feedback.calculate(io.getPosition().in(Meters))
                                    + feedforward.calculate(feedback.getSetpoint().velocity));
        }

        io.setVoltage(volts);
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

    public Measure<Distance> getPosition() {
        return io.getPosition();
    }

    public Measure<Distance> getSetpoint() {
        return setpoint;
    }

    public boolean atCurrentCap() {
        return inputs.leftCurrent.in(Amps) > 50;
    }
}
