package frc.robot.subsystems;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Motor;

public class Intake extends SubsystemBase {
    private final static Intake INSTANCE = new Intake();

    public static Intake getInstance() {
        return INSTANCE;
    }

    private final Motor intakeMotor;

    private Intake() {
        intakeMotor = new Motor(Constants.Motors.Intake);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        intakeMotor.setVelocity(velocity);
    }
}

