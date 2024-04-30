package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

    public static class MotorDefinition{
        public double P;
        public double I;
        public double D;
        public double FF;




        public MotorDefinition(double P, double I, double D, double FF){
            this.P = P;
            this.I = I;
            this.D = D;
            this.FF = FF;
        }
    }

    public static class MotorDefinitions {

        public static MotorDefinition topShooter = new MotorDefinition(0.0, 0.0, 0.0,0.0);

        public static MotorDefinition bottomShooter = new MotorDefinition(0.0, 0.0,0.0,0.0);




    }

    public static class PIDConstants {
        // ----------intake----------//

        public static final double INTAKE_PID_KP = 0.0;
        public static final double INTAKE_PID_KI = 0.0;
        public static final double INTAKE_PID_KD = 0.0;

        public static final double INTAKE_FEEDFORWARD_FF = 0.0;

        public static final double INTAKE_SETPOINT = 0.0;

        // ----------indexer----------//

        public static final double INDEXER_PID_KP = 0.0;
        public static final double INDEXER_PID_KI = 0.0;
        public static final double INDEXER_PID_KD = 0.0;

        public static final double INDEXER_FEEDFORWARD_FF = 0.0;

        public static final double INDEXER_SETPOINT = 0.0;

        // ----------shooter----------//


        public static final double TOP_SHOOTER_SPEAKER_SETPOINT = 0.0; // should be negitive
        public static final double TOP_SHOOTER_AMP_SETPOINT = 0.0; // should be negitive
        public static final double TOP_SHOOTER_VELOCITY_RANGE_AMP = 20;



        public static final double BOTTOM_SHOOTER_SPEAKER_SETPOINT = 0.0;
        public static final double BOTTOM_SHOOTER_AMP_SETPOINT = 0.0;
        public static final double BOTTOM_SHOOTER_VELOCITY_RANGE_AMP = 20;


        public static final double SEND_TO_SHOOTER_SETPOINT = 0.0;


    }

    public static class MotorConstants {
        public static final int INTAKE_MOTOR_PORT = 1;


    }

    public static class controller {

        public static final XboxController CONTROLLER = new XboxController(1);

        public static final GenericHID OPERATOR_CONTROLLER = new GenericHID(0);

    }

    public static class LoggerConstants {
        public static final boolean FILEONLY = false;
        public static final boolean LAZYLOGGING = false;
    }

}
