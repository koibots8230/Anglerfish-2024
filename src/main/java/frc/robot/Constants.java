package frc.robot;

public final class Constants {
    public static class Motors {
        public static Motor Intake = new Motor(
                12, 0.0001, 0, 1, 0, 0, 1.0, Motor.MotorType.CANSPARKMAX_BRUSHLESS);
        public static Motor Indexer = new Motor(
                13, 0.0001, 0, 1, 0, 0, 1.0, Motor.MotorType.CANSPARKMAX_BRUSHLESS);
        public static Motor ShooterTop = new Motor(
                14, 0.0001, 0, 1, 0, 0, 1.0, Motor.MotorType.CANSPARKMAX_BRUSHLESS);
        public static Motor ShooterBottom = new Motor(
                15, 0.0001, 0, 1, 0, 0, 1.0, Motor.MotorType.CANSPARKMAX_BRUSHLESS);

        public static class Motor {
            public final int CANID;
            public final double P;
            public final double I;
            public final double D;
            public final double IZone;
            public final double FF;
            public final MotorType motorType;
            public final double gearing;
            Motor(int CANID, double P, double I, double D, double IZone, double FF, double gearing, MotorType motorType) {
                this.CANID = CANID;
                this.P = P;
                this.I = I;
                this.D = D;
                this.IZone = IZone;
                this.FF = FF;
                this.gearing = gearing;
                this.motorType = motorType;
            }

            public enum MotorType {
                CANSPARKMAX_BRUSHLESS,
                CANSPARKMAX_BRUSHED
            }
        }
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
