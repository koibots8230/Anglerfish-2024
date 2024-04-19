package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import java.util.concurrent.TimeUnit;
import monologue.Logged;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PIDConstants{
    //----------intake----------//

    public static final double INTAKE_PID_KP = 0.0;
    public static final double INTAKE_PID_KI = 0.0;
    public static final double INTAKE_PID_KD = 0.0;

    public static final double INTAKE_FEEDFORWARD_FF = 0.0;

    public static final double INTAKE_SETPOINT = 0.0;


    //----------indexer----------//

    public static final double INDEXER_PID_KP = 0.0;
    public static final double INDEXER_PID_KI = 0.0;
    public static final double INDEXER_PID_KD = 0.0;

    public static final double INDEXER_FEEDFORWARD_FF = 0.0;

    public static final double INDEXER_SETPOINT = 0.0;
    public static final double OPERATOR_INDEXER_SETPOINT = 0.0;

    //----------shooter----------//

    public static final double TOP_SHOOTER_PID_KP = 0.0;
    public static final double TOP_SHOOTER_PID_KI = 0.0;
    public static final double TOP_SHOOTER_PID_KD = 0.0;

    public static final double TOP_SHOOTER_FEEDFORWARD_FF = 0.0;

    public static final double TOP_SHOOTER_SETPOINT = 0.0; // should be negitive 
    public static final double TOP_SHOOTER_VELOCITY_RANGE_AMP = 20;
    public static final double TOP_SHOOTER_VELOCITY_RANGE_SPEAKER = 20;

    public static final double BOTTOM_SHOOTER_PID_KP = 0.0;
    public static final double BOTTOM_SHOOTER_PID_KI = 0.0;
    public static final double BOTTOM_SHOOTER_PID_KD = 0.0;

    public static final double BOTTOM_SHOOTER_FEEDFORWARD_FF = 0.0;

    public static final double BOTTOM_SHOOTER_SETPOINT = 0.0;  
    public static final double BOTTOM_SHOOTER_VELOCITY_RANGE_AMP = 20;
    public static final double BOTTOM_SHOOTER_VELOCITY_RANGE_SPEAKER = 20;

    public static final double SEND_TO_SHOOTER_SETPOINT = 0.0;
    

  }

  public static class MotorConstants{
    public static final int INTAKE_MOTOR_PORT = 1; 
  }

  public static class controller{

    public static final XboxController CONTROLLER = new XboxController(1);

    public static final GenericHID OPERATOR_CONTROLLER = new GenericHID(0);

  }

  public static class LoggerConstants{
    public static final boolean FILEONLY = false;
    public static final boolean LAZYLOGGING = false;
  }


  
}
