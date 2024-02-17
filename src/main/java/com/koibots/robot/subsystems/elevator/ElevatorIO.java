// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorInputs {
        public Measure<Distance> position = Meters.of(0);
        public Measure<Distance> setpoint = Meters.of(0);
        public Measure<Velocity<Distance>> velocity = MetersPerSecond.of(0);

        public Measure<Voltage> voltage = Volts.of(0);
        public Measure<Current> leftCurrent = Amps.of(0);
        public Measure<Current> rightCurrent = Amps.of(0);
    }

    void updateInputs(ElevatorInputs inputs);

    /* Set the motors to a specified voltage */
    void setVoltage(Measure<Voltage> volts);

    /* Set the motors to brake mode */
    default void setBrake() {}

    /* Get the encoder position */
    Measure<Distance> getPosition();

    /* Get the encoder velocity */
    Measure<Velocity<Distance>> getVelocity();
}
