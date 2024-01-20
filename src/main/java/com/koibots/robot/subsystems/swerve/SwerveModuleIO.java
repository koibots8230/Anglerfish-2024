// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleInputs {
        public Measure<Distance> drivePosition = Meters.of(0);
        public Measure<Velocity<Distance>> driveVelocity = MetersPerSecond.of(0);
        public Measure<Voltage> driveAppliedVoltage = Volts.of(0);
        public Measure<Current> driveCurrent = Amps.of(0);

        public Rotation2d turnPosition = new Rotation2d();
        public Measure<Velocity<Angle>> turnVelocity = RadiansPerSecond.of(0);
        public Measure<Voltage> turnAppliedVoltage = Volts.of(0);
        public Measure<Current> turnCurrent = Amps.of(0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(SwerveModuleInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    default void setDriveVoltage(Measure<Voltage> volts) {}

    /** Run the turn motor at the specified voltage. */
    default void setTurnVoltage(Measure<Voltage> volts) {}

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    default void setTurnBrakeMode(boolean enable) {}
}
