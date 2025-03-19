package frc.lib.util;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorConstantsIO {
    public final boolean inverted;
    public final int currentLimit;
    public final IdleMode idleMode;

    public MotorConstantsIO(boolean inverted, int currentLimit, IdleMode idleMode) {
        this.inverted = inverted;
        this.currentLimit = currentLimit;
        this.idleMode = idleMode;
    }

    public MotorConstantsIO(boolean inverted, int currentLimit) {
        this.inverted = inverted;
        this.currentLimit = currentLimit;
        this.idleMode = IdleMode.kCoast;
    }
}