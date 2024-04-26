// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.util;

import com.revrobotics.CANSparkBase.IdleMode;

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
