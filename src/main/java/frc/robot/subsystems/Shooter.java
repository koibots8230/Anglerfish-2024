package frc.robot.subsystems;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO.Motor;

public class Shooter extends SubsystemBase {
    private final static Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private final Motor shooterMotorTop;
    private final Motor shooterMotorBottom;

    private Shooter() {
        shooterMotorTop = new Motor(Constants.Motors.ShooterTop);
        shooterMotorBottom = new Motor(Constants.Motors.ShooterBottom);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocityTop, Measure<Velocity<Angle>> velocityBottom) {
        shooterMotorTop.setVelocity(velocityTop);
        shooterMotorBottom.setVelocity(velocityBottom);
    }
}
