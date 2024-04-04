// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public double setpoint = 0;
        public double velocity = 0;

        public Measure<Current> current = Amps.of(0);
        public Measure<Voltage> voltage = Volts.of(0);
    }

    /* Updates the set of loggable inputs. */
    void updateInputs(IntakeIOInputs inputs);

    /* Run the pivot motor at the specified voltage. */
    void setVoltage(Measure<Voltage> volts);
}
