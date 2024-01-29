package com.koibots.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;

public interface ShooterPivotIO {
    @AutoLog
    public static class ShooterPivotIOInputs {
        public Rotation2d position = new Rotation2d();
        public Measure<Voltage> voltage = Volts.of(0);
        public Measure<Current> current = Amps.of(0);
        public Measure<Velocity<Angle>> velocity = RadiansPerSecond.of(0);
    }

    void updateInputs(ShooterPivotIOInputs inputs);
    void zeroOffset();
    void setMotorSpeed(double desiredPosition);
    void setBrakeMode();
    void setCoastMode();
}
